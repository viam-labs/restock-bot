package restockbot

import (
	"context"
	"errors"
	"fmt"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"google.golang.org/protobuf/encoding/protojson"
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

// checkGripperSteps fails before anything moves if the motion needs a gripper this service lacks.
func checkGripperSteps(wps []Sample, haveGripper bool) error {
	if haveGripper {
		return nil
	}
	for _, wp := range wps {
		if wp.Gripper != "" {
			return fmt.Errorf("waypoint %q has a gripper action but no 'gripper' is configured", wp.Mark)
		}
	}
	return nil
}

func (s *fixtureTeacher) run(ctx context.Context, motionName, seqID string, dryRun bool, speed *arm.MoveOptions) map[string]any {
	if motionName == "" && seqID == "" {
		return fail(errors.New("run requires a 'motion' or 'sequence_id'"))
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

	var samples []Sample
	var err error
	if seqID != "" {
		samples, motionName, err = s.resolvePinned(ctx, seqID, motionName)
	} else {
		seqID, samples, err = s.store.resolve(ctx, s.fixture, motionName)
	}
	if err != nil {
		return fail(err)
	}
	wps := waypoints(samples)
	if len(wps) == 0 {
		return fail(fmt.Errorf("sequence %s has no marked waypoints", seqID))
	}
	if err := checkGripperSteps(wps, s.gripper != nil); err != nil {
		return fail(err)
	}
	markerWorld, err := s.localize(ctx)
	if err != nil {
		return fail(err)
	}

	if dryRun {
		out := make([]any, 0, len(wps))
		for _, wp := range wps {
			entry := map[string]any{"label": wp.Mark, "linear": wp.Linear, "world": poseToMap(toWorld(markerWorld, wp.Pose))}
			if wp.Gripper != "" {
				entry["gripper"] = wp.Gripper
			}
			out = append(out, entry)
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
		err := s.moveTo(ctx, motion.MoveReq{
			ComponentName: s.cfg.TCP,
			Destination:   referenceframe.NewPoseInFrame(s.markerFrame(), wp.Pose),
			WorldState:    ws,
			Constraints:   constraints,
		}, speed)
		step := map[string]any{"label": wp.Mark, "ok": err == nil, "target": poseToMap(wp.Pose)}
		if err != nil {
			step["error"] = explainMoveError(err)
		} else if wp.Gripper != "" {
			step["gripper"] = wp.Gripper
			holding, gerr := s.actuate(ctx, wp.Gripper)
			if wp.Gripper == gripperGrab {
				step["holding"] = holding // reported, not checked: hooks hold nothing
			}
			if gerr != nil {
				step["ok"], step["error"] = false, fmt.Sprintf("gripper %s: %v", wp.Gripper, gerr)
			}
		}
		steps = append(steps, step)
		if step["ok"] != true {
			return map[string]any{"ok": false, "sequence_id": seqID, "steps": steps, "failed_step": wp.Mark}
		}
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
	// Stop both even if one fails.
	var errs []error
	if err := s.arm.Stop(ctx, nil); err != nil {
		errs = append(errs, fmt.Errorf("arm: %w", err))
	}
	if s.gripper != nil {
		if err := s.gripper.Stop(ctx, nil); err != nil {
			errs = append(errs, fmt.Errorf("gripper: %w", err))
		}
	}
	if err := errors.Join(errs...); err != nil {
		return fail(err)
	}
	return map[string]any{"ok": true}
}

// moveTo moves via motion.Move, or, with speed limits, plans with the motion service and
// executes on the arm, since motion.Move takes no speed.
func (s *fixtureTeacher) moveTo(ctx context.Context, req motion.MoveReq, opts *arm.MoveOptions) error {
	if opts == nil {
		ok, err := s.motion.Move(ctx, req)
		if err == nil && !ok {
			err = errors.New("motion service reported the move did not succeed")
		}
		return err
	}
	pbReq, err := req.ToProto(s.motion.Name().ShortName())
	if err != nil {
		return err
	}
	js, err := protojson.Marshal(pbReq)
	if err != nil {
		return err
	}
	resp, err := s.motion.DoCommand(ctx, map[string]any{"plan": string(js)})
	if err != nil {
		return fmt.Errorf("planning: %w", err)
	}
	path, err := jointPath(resp, s.cfg.Arm)
	if err != nil {
		return err
	}
	return s.arm.MoveThroughJointPositions(ctx, path, opts, nil)
}
