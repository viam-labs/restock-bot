package restockbot

import (
	"math"
	"strings"
	"testing"

	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/utils/protoutils"
)

func TestParseSpeed(t *testing.T) {
	opts, err := parseSpeed(map[string]any{})
	if err != nil || opts != nil {
		t.Fatalf("absent: got %v, %v", opts, err)
	}
	opts, err = parseSpeed(map[string]any{"speed_degs_per_sec": 180.0, "accel_degs_per_sec2": 90.0})
	if err != nil || math.Abs(opts.MaxVelRads-math.Pi) > 1e-9 || math.Abs(opts.MaxAccRads-math.Pi/2) > 1e-9 {
		t.Fatalf("valid: got %+v, %v", opts, err)
	}
	opts, err = parseSpeed(map[string]any{"speed_degs_per_sec": 90.0})
	if err != nil || opts.MaxAccRads != 0 {
		t.Fatalf("speed only: got %+v, %v", opts, err)
	}
	for _, bad := range []any{0.0, -5.0, "20", math.NaN(), math.Inf(1)} {
		if _, err := parseSpeed(map[string]any{"speed_degs_per_sec": bad}); err == nil {
			t.Errorf("speed %v accepted", bad)
		}
		if _, err := parseSpeed(map[string]any{"speed_degs_per_sec": 20.0, "accel_degs_per_sec2": bad}); err == nil {
			t.Errorf("accel %v accepted", bad)
		}
	}
	if _, err := parseSpeed(map[string]any{"accel_degs_per_sec2": 50.0}); err == nil || !strings.Contains(err.Error(), "requires") {
		t.Fatalf("accel without speed: %v", err)
	}
}

// wire converts a builtin-style "plan" response to what a gRPC client receives.
func wire(t *testing.T, resp map[string]any) map[string]any {
	t.Helper()
	pb, err := protoutils.StructToStructPb(resp)
	if err != nil {
		t.Fatal(err)
	}
	return pb.AsMap()
}

func TestJointPath(t *testing.T) {
	traj := motionplan.Trajectory{
		{"arm": {0, 0.1, 0.2}, "gripper": {}},
		{"arm": {0.3, 0.4, 0.5}, "gripper": {}},
	}
	path, err := jointPath(wire(t, map[string]any{"plan": traj}), "arm")
	if err != nil {
		t.Fatal(err)
	}
	want := [][]referenceframe.Input{{0, 0.1, 0.2}, {0.3, 0.4, 0.5}}
	if len(path) != 2 || path[0][1] != want[0][1] || path[1][2] != want[1][2] {
		t.Fatalf("got %v", path)
	}

	cases := map[string]map[string]any{
		"partial":   {"plan": traj, "plan_partialwp": 1},
		"no plan":   {},
		"no arm":    {"plan": motionplan.Trajectory{{"arm": {0}}, {"gripper": {}}}},
		"empty arm": {"plan": motionplan.Trajectory{{"arm": {0}}, {"arm": {}}}},
	}
	for name, resp := range cases {
		if _, err := jointPath(wire(t, resp), "arm"); err == nil {
			t.Errorf("%s: expected error", name)
		}
	}
}
