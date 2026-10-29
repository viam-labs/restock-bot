package restockbot

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"time"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

const (
	// freedriveFor bounds manual mode so the arm leaves freedrive even if the module dies mid-demo.
	freedriveFor = 5 * time.Minute
	// Uploads get their own deadline so ending a session never cancels one mid-flight.
	uploadTimeout = 10 * time.Second
	// cleanupTimeout bounds turning freedrive off, independent of the caller's context.
	cleanupTimeout = 10 * time.Second
	maxSampleHz    = 1000
)

type session struct {
	motion      string
	taught      string // "freedrive" or "jog"
	freedrive   bool
	start       time.Time
	markerWorld spatialmath.Pose
	cancel      context.CancelFunc
	done        chan struct{}

	mu         sync.Mutex
	pending    []Sample // captured, not yet uploaded
	count      int
	first      time.Time
	last       time.Time
	marks      []Sample
	hasMark    bool
	markLabel  string
	markLinear bool
	uploadErr  error
}

func (s *fixtureTeacher) startTeach(ctx context.Context, motionName string) map[string]any {
	if motionName == "" {
		return fail(errNoMotion)
	}
	if err := s.claim("teach"); err != nil {
		return fail(err)
	}
	markerWorld, err := s.localize(ctx)
	if err != nil {
		s.release()
		return fail(err)
	}
	sess := &session{motion: motionName, taught: "jog", start: time.Now(), markerWorld: markerWorld, done: make(chan struct{})}
	// Fail fast if the TCP can't be sampled, rather than after a whole hand-guided demo.
	if err := s.captureSample(ctx, sess); err != nil {
		s.release()
		return fail(fmt.Errorf("sampling %s: %w", s.cfg.TCP, err))
	}
	if props, err := s.arm.Properties(ctx, nil); err == nil && props.SupportManualMode {
		if err := s.arm.SetManualMode(ctx, true, freedriveFor, nil); err != nil {
			s.release()
			return fail(fmt.Errorf("enabling freedrive: %w", err))
		}
		sess.taught, sess.freedrive = "freedrive", true
	}
	s.mu.Lock()
	if s.closed {
		// Close ran while we were localizing; don't start a sampler nobody will stop.
		s.mu.Unlock()
		if sess.freedrive {
			_ = s.arm.SetManualMode(ctx, false, 0, nil)
		}
		s.release()
		return fail(errors.New("service is closing"))
	}
	loopCtx, cancel := context.WithCancel(context.Background())
	sess.cancel = cancel
	s.session = sess
	s.mu.Unlock()
	go s.sampleLoop(loopCtx, sess)
	s.logger.Infof("teaching %s/%s (%s)", s.fixture, motionName, sess.taught)
	return map[string]any{"ok": true, "motion": motionName, "taught": sess.taught}
}

// sampleLoop samples at sample_hz; uploads run in a separate goroutine so they never stall sampling.
func (s *fixtureTeacher) sampleLoop(ctx context.Context, sess *session) {
	defer close(sess.done)
	var wg sync.WaitGroup
	wg.Add(1)
	go func() {
		defer wg.Done()
		s.flushLoop(ctx, sess)
	}()
	defer wg.Wait()
	tick := time.NewTicker(time.Duration(float64(time.Second) / s.cfg.SampleHz))
	defer tick.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-tick.C:
			if err := s.captureSample(ctx, sess); err != nil {
				s.logger.Debugf("sample failed: %v", err)
			}
		}
	}
}

func (s *fixtureTeacher) flushLoop(ctx context.Context, sess *session) {
	tick := time.NewTicker(time.Second)
	defer tick.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-tick.C:
			s.flushWithTimeout(sess)
		}
	}
}

// flushWithTimeout uploads on its own deadline so a caller's cancellation never loses a batch.
func (s *fixtureTeacher) flushWithTimeout(sess *session) {
	ctx, cancel := context.WithTimeout(context.Background(), uploadTimeout)
	defer cancel()
	s.flush(ctx, sess)
}

func (s *fixtureTeacher) captureSample(ctx context.Context, sess *session) error {
	pif, err := s.motion.GetPose(ctx, s.cfg.TCP, referenceframe.World, nil, nil)
	if err != nil {
		return err
	}
	now := time.Now()
	smp := Sample{At: now, T: now.Sub(sess.start).Seconds(), Pose: toMarker(sess.markerWorld, pif.Pose())}
	sess.mu.Lock()
	defer sess.mu.Unlock()
	if sess.hasMark {
		smp.Mark, smp.Linear = sess.markLabel, sess.markLinear
		sess.hasMark = false
		sess.marks = append(sess.marks, smp)
	}
	sess.pending = append(sess.pending, smp)
	sess.count++
	if sess.first.IsZero() {
		sess.first = now
	}
	sess.last = now
	return nil
}

func (s *fixtureTeacher) flush(ctx context.Context, sess *session) {
	sess.mu.Lock()
	batch := sess.pending
	sess.pending = nil
	sess.mu.Unlock()
	if len(batch) == 0 {
		return
	}
	if err := s.store.upload(ctx, batch); err != nil {
		s.logger.Errorf("uploading %d samples: %v", len(batch), err)
		sess.mu.Lock()
		if sess.uploadErr == nil {
			sess.uploadErr = err
		}
		sess.mu.Unlock()
	}
}

func (s *fixtureTeacher) mark(label string, linear bool) map[string]any {
	if label == "" {
		return fail(errors.New("mark requires a 'label'"))
	}
	// Hold s.mu so a mark can't land on a session end_teach has already taken.
	s.mu.Lock()
	defer s.mu.Unlock()
	sess := s.session
	if sess == nil {
		return fail(errors.New("no teaching session active"))
	}
	sess.mu.Lock()
	sess.hasMark, sess.markLabel, sess.markLinear = true, label, linear
	sess.mu.Unlock()
	return map[string]any{"ok": true}
}

// endSession turns freedrive off and frees the service; shared by end_teach and Close.
func (s *fixtureTeacher) endSession(sess *session) error {
	var err error
	if sess.freedrive {
		ctx, cancel := context.WithTimeout(context.Background(), cleanupTimeout)
		if err = s.arm.SetManualMode(ctx, false, 0, nil); err != nil {
			s.logger.Errorf("disabling freedrive: %v", err)
		}
		cancel()
	}
	s.mu.Lock()
	s.session = nil
	s.busy = ""
	s.mu.Unlock()
	return err
}

func (s *fixtureTeacher) endTeach(ctx context.Context) map[string]any {
	// Take the session atomically so a double end_teach (or Close) can't finish it twice.
	s.mu.Lock()
	sess := s.session
	s.session = nil
	s.mu.Unlock()
	if sess == nil {
		return fail(errors.New("no teaching session active"))
	}
	sess.cancel()
	<-sess.done
	finalErr := s.captureSample(ctx, sess) // so a pending mark is never dropped
	freedriveErr := s.endSession(sess)
	s.flushWithTimeout(sess)
	res := s.finishTeach(ctx, sess, finalErr)
	if freedriveErr != nil {
		res["ok"] = false
		res["error"] = fmt.Sprintf("disabling freedrive failed, the arm may still be in manual mode: %v", freedriveErr)
	}
	return res
}

// finishTeach checks the ended session and creates its sequence.
func (s *fixtureTeacher) finishTeach(ctx context.Context, sess *session, finalErr error) map[string]any {
	sess.mu.Lock()
	defer sess.mu.Unlock()
	switch {
	case sess.hasMark:
		return fail(fmt.Errorf("mark %q was not captured: %v", sess.markLabel, finalErr))
	case sess.uploadErr != nil:
		return fail(fmt.Errorf("upload failed, no sequence created: %w", sess.uploadErr))
	case sess.count == 0:
		return fail(errors.New("no samples captured"))
	}
	tags := []string{"fixture:" + s.fixture, "motion:" + sess.motion, "taught:" + sess.taught}
	id, err := s.store.createSequence(ctx, tags, sess.first, sess.last)
	if err != nil {
		return fail(fmt.Errorf("creating sequence: %w", err))
	}
	marks := make([]any, 0, len(sess.marks))
	for _, m := range sess.marks {
		marks = append(marks, map[string]any{"label": m.Mark, "linear": m.Linear, "pose": poseToMap(m.Pose)})
	}
	s.logger.Infof("taught %s/%s as sequence %s (%d samples)", s.fixture, sess.motion, id, sess.count)
	return map[string]any{"ok": true, "sequence_id": id, "samples": sess.count, "marks": marks}
}
