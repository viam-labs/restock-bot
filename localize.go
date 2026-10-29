package restockbot

import (
	"context"
	"fmt"
	"sort"
	"strings"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

const (
	localizeReadings  = 5
	viewSwitchTimeout = 60 * time.Second
	// arm-position-saver: 2 moves to the saved pose. 1 would OVERWRITE the saved pose.
	viewSwitchGoTo = 2
	// Tunable: let the arm and camera settle after the view move, and space readings so they aren't one buffered frame.
	settleAfterView = 500 * time.Millisecond
	readingGap      = 100 * time.Millisecond
)

func (s *fixtureTeacher) markerFrame() string {
	return s.fixture + "-marker"
}

// findMarker returns the first tracker key (sorted) matching "<marker>" or "<prefix>-<marker>".
func findMarker(readings referenceframe.FrameSystemPoses, marker string) (string, bool) {
	for _, k := range sortedKeys(readings) {
		if k == marker || strings.HasSuffix(k, "-"+marker) {
			return k, true
		}
	}
	return "", false
}

func sortedKeys(readings referenceframe.FrameSystemPoses) []string {
	keys := make([]string, 0, len(readings))
	for k := range readings {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	return keys
}

// toMarker expresses a world pose in the marker frame.
func toMarker(markerWorld, world spatialmath.Pose) spatialmath.Pose {
	return spatialmath.Compose(spatialmath.PoseInverse(markerWorld), world)
}

// toWorld expresses a marker-frame pose in the world frame.
func toWorld(markerWorld, local spatialmath.Pose) spatialmath.Pose {
	return spatialmath.Compose(markerWorld, local)
}

// averagePoses averages positions and sign-aligned quaternions.
func averagePoses(poses []spatialmath.Pose) spatialmath.Pose {
	var sum r3.Vector
	var acc quat.Number
	q0 := poses[0].Orientation().Quaternion()
	for _, p := range poses {
		sum = sum.Add(p.Point())
		q := p.Orientation().Quaternion()
		if q.Real*q0.Real+q.Imag*q0.Imag+q.Jmag*q0.Jmag+q.Kmag*q0.Kmag < 0 {
			q = quat.Scale(-1, q)
		}
		acc = quat.Add(acc, q)
	}
	o := spatialmath.Quaternion(quat.Scale(1/quat.Abs(acc), acc))
	return spatialmath.NewPose(sum.Mul(1/float64(len(poses))), &o)
}

// sleepCtx waits for d or until ctx is done.
func sleepCtx(ctx context.Context, d time.Duration) error {
	t := time.NewTimer(d)
	defer t.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-t.C:
		return nil
	}
}

// localize moves to the view pose (if configured), averages several marker readings in world, and caches the result.
func (s *fixtureTeacher) localize(ctx context.Context) (spatialmath.Pose, error) {
	if s.viewSwitch != nil {
		swCtx, cancel := context.WithTimeout(ctx, viewSwitchTimeout)
		err := s.viewSwitch.SetPosition(swCtx, viewSwitchGoTo, nil)
		cancel()
		if err != nil {
			return nil, fmt.Errorf("moving to view pose: %w", err)
		}
		if err := sleepCtx(ctx, settleAfterView); err != nil {
			return nil, err
		}
	}
	var poses []spatialmath.Pose
	var lastKeys []string
	for i := 0; i < localizeReadings; i++ {
		if i > 0 {
			if err := sleepCtx(ctx, readingGap); err != nil {
				return nil, err
			}
		}
		readings, err := s.tracker.Poses(ctx, nil, nil)
		if err != nil {
			return nil, err
		}
		key, ok := findMarker(readings, s.marker)
		if !ok {
			lastKeys = sortedKeys(readings)
			continue
		}
		pif := readings[key]
		link := referenceframe.NewLinkInFrame(orDefault(pif.Parent(), "cam"), pif.Pose(), s.markerFrame(), nil)
		world, err := s.motion.GetPose(ctx, s.markerFrame(), referenceframe.World, []*referenceframe.LinkInFrame{link}, nil)
		if err != nil {
			return nil, err
		}
		poses = append(poses, world.Pose())
	}
	if len(poses) == 0 {
		return nil, fmt.Errorf("marker %q not seen; pose tracker reported keys: %v", s.marker, lastKeys)
	}
	avg := averagePoses(poses)
	s.mu.Lock()
	s.markerWorld = avg
	s.mu.Unlock()
	s.logger.Infof("localized %s from %d readings: %v", s.markerFrame(), len(poses), poseToMap(avg))
	return avg, nil
}
