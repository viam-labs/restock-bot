package restockbot

import (
	"context"
	"errors"
	"strings"
	"testing"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	injectmotion "go.viam.com/rdk/testutils/inject/motion"
)

func TestWaypoints(t *testing.T) {
	p := spatialmath.NewZeroPose()
	samples := []Sample{{T: 0, Pose: p}, {T: 1, Pose: p, Mark: "hooked"}, {T: 2, Pose: p}, {T: 3, Pose: p, Mark: "pulled", Linear: true}}
	got := waypoints(samples)
	if len(got) != 2 || got[0].Mark != "hooked" || got[1].Mark != "pulled" || !got[1].Linear {
		t.Fatalf("got %+v", got)
	}
}

func TestCheckGripperSteps(t *testing.T) {
	p := spatialmath.NewZeroPose()
	wps := []Sample{{Pose: p, Mark: "approach"}, {Pose: p, Mark: "pick", Gripper: gripperGrab}}
	if err := checkGripperSteps(wps, false); err == nil || !strings.Contains(err.Error(), `"pick"`) {
		t.Fatalf("got %v", err)
	}
	if err := checkGripperSteps(wps, true); err != nil {
		t.Fatal(err)
	}
	if err := checkGripperSteps(wps[:1], false); err != nil {
		t.Fatal(err)
	}
}

func TestStopReportsBoth(t *testing.T) {
	a := inject.NewArm("arm")
	a.StopFunc = func(context.Context, map[string]interface{}) error { return errors.New("estop") }
	g := inject.NewGripper("gripper")
	g.StopFunc = func(context.Context, map[string]interface{}) error { return errors.New("stuck") }
	res := (&fixtureTeacher{arm: a, gripper: g}).stop(context.Background())
	msg, _ := res["error"].(string)
	if res["ok"] != false || !strings.Contains(msg, "arm: estop") || !strings.Contains(msg, "gripper: stuck") {
		t.Fatalf("got %v", res)
	}
	a.StopFunc = func(context.Context, map[string]interface{}) error { return nil }
	g.StopFunc = func(context.Context, map[string]interface{}) error { return nil }
	if res := (&fixtureTeacher{arm: a, gripper: g}).stop(context.Background()); res["ok"] != true {
		t.Fatalf("got %v", res)
	}
}

func TestMoveToWithSpeed(t *testing.T) {
	ms := injectmotion.NewMotionService("builtin")
	var planReq any
	ms.DoCommandFunc = func(_ context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
		planReq = cmd["plan"]
		return map[string]any{"plan": motionplan.Trajectory{{"arm": {0, 0}}, {"arm": {1, 1}}}}, nil
	}
	ms.MoveFunc = func(context.Context, motion.MoveReq) (bool, error) {
		t.Fatal("Move called despite a speed")
		return false, nil
	}
	a := inject.NewArm("arm")
	var gotPath [][]referenceframe.Input
	var gotOpts *arm.MoveOptions
	a.MoveThroughJointPositionsFunc = func(_ context.Context, p [][]referenceframe.Input, o *arm.MoveOptions, _ map[string]interface{}) error {
		gotPath, gotOpts = p, o
		return nil
	}
	s := &fixtureTeacher{arm: a, motion: ms, cfg: Config{Arm: "arm"}}
	req := motion.MoveReq{ComponentName: "tcp", Destination: referenceframe.NewPoseInFrame("world", spatialmath.NewZeroPose())}
	opts := &arm.MoveOptions{MaxVelRads: 0.5}
	if err := s.moveTo(context.Background(), req, opts); err != nil {
		t.Fatal(err)
	}
	if js, ok := planReq.(string); !ok || !strings.Contains(js, `"componentName":"tcp"`) {
		t.Fatalf("plan request %v", planReq)
	}
	if len(gotPath) != 2 || gotOpts != opts {
		t.Fatalf("path %v opts %v", gotPath, gotOpts)
	}
}

func TestMoveToWithoutSpeed(t *testing.T) {
	ms := injectmotion.NewMotionService("builtin")
	ms.MoveFunc = func(context.Context, motion.MoveReq) (bool, error) { return false, nil }
	s := &fixtureTeacher{motion: ms}
	if err := s.moveTo(context.Background(), motion.MoveReq{}, nil); err == nil {
		t.Fatal("expected error when Move reports failure")
	}
}
