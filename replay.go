package restockbot

import (
	"context"
	"errors"
	"fmt"

	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

const (
	lineToleranceMM          = 5.0
	orientationToleranceDegs = 5.0
)

// waypoints returns the marked samples; samples are already sorted by T.
func waypoints(samples []Sample) []Sample {
	var out []Sample
	for _, smp := range samples {
		if smp.Mark != "" {
			out = append(out, smp)
		}
	}
	return out
}

// worldState places the marker frame in world and the fixture's obstacles in the marker frame.
func (s *fixtureTeacher) worldState(markerWorld spatialmath.Pose) (*referenceframe.WorldState, error) {
	var obstacles []*referenceframe.GeometriesInFrame
	if len(s.obstacles) > 0 {
		obstacles = append(obstacles, referenceframe.NewGeometriesInFrame(s.markerFrame(), s.obstacles))
	}
	return referenceframe.NewWorldState(obstacles,
		[]*referenceframe.LinkInFrame{referenceframe.NewLinkInFrame(referenceframe.World, markerWorld, s.markerFrame(), nil)})
}

func (s *fixtureTeacher) run(ctx context.Context, motionName string, dryRun bool) map[string]any {
	if motionName == "" {
		return fail(errNoMotion)
	}
	ctx, cancel := context.WithCancel(ctx)
	runDone := make(chan struct{})
	s.mu.Lock()
	if err := s.claimLocked("run"); err != nil {
		s.mu.Unlock()
		cancel()
		return fail(err)
	}
	// Set under the same lock as the claim so stop and Close can never miss an active run.
	s.cancelRun, s.runDone = cancel, runDone
	s.mu.Unlock()
	defer func() {
		cancel()
		s.mu.Lock()
		s.cancelRun, s.runDone = nil, nil
		s.busy = ""
		s.mu.Unlock()
		close(runDone)
	}()

	seqID, samples, err := s.store.resolve(ctx, s.fixture, motionName)
	if err != nil {
		return fail(err)
	}
	wps := waypoints(samples)
	if len(wps) == 0 {
		return fail(fmt.Errorf("sequence %s has no marked waypoints", seqID))
	}
	markerWorld, err := s.localize(ctx)
	if err != nil {
		return fail(err)
	}

	if dryRun {
		out := make([]any, 0, len(wps))
		for _, wp := range wps {
			out = append(out, map[string]any{"label": wp.Mark, "linear": wp.Linear, "world": poseToMap(toWorld(markerWorld, wp.Pose))})
		}
		obstacles := make([]any, 0, len(s.obstacles))
		for _, g := range s.obstacles {
			obstacles = append(obstacles, map[string]any{"label": g.Label(), "world": poseToMap(toWorld(markerWorld, g.Pose()))})
		}
		return map[string]any{"ok": true, "sequence_id": seqID, "waypoints": out, "obstacles": obstacles}
	}

	ws, err := s.worldState(markerWorld)
	if err != nil {
		return fail(err)
	}
	steps := make([]any, 0, len(wps))
	for _, wp := range wps {
		var constraints *motionplan.Constraints
		if wp.Linear {
			constraints = &motionplan.Constraints{LinearConstraint: []motionplan.LinearConstraint{
				{LineToleranceMm: lineToleranceMM, OrientationToleranceDegs: orientationToleranceDegs},
			}}
		}
		s.logger.Infof("run %s/%s: moving to %s", s.fixture, motionName, wp.Mark)
		ok, err := s.motion.Move(ctx, motion.MoveReq{
			ComponentName: s.cfg.TCP,
			Destination:   referenceframe.NewPoseInFrame(s.markerFrame(), wp.Pose),
			WorldState:    ws,
			Constraints:   constraints,
		})
		step := map[string]any{"label": wp.Mark, "ok": err == nil && ok, "target": poseToMap(wp.Pose)}
		if err == nil && !ok {
			err = errors.New("motion service reported the move did not succeed")
		}
		if err != nil {
			step["error"] = explainMoveError(err)
			steps = append(steps, step)
			return map[string]any{"ok": false, "sequence_id": seqID, "steps": steps, "failed_step": wp.Mark}
		}
		steps = append(steps, step)
	}
	return map[string]any{"ok": true, "sequence_id": seqID, "steps": steps}
}

func (s *fixtureTeacher) stop(ctx context.Context) map[string]any {
	s.mu.Lock()
	cancel := s.cancelRun
	s.mu.Unlock()
	if cancel != nil {
		cancel()
	}
	if err := s.arm.Stop(ctx, nil); err != nil {
		return fail(err)
	}
	return map[string]any{"ok": true}
}
