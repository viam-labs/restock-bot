package restockbot

import (
	"context"
	"strings"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
)

func TestConstructorAcceptsRegisteredConfigType(t *testing.T) {
	// viam-server hands the constructor the type registered in init(); empty deps should
	// fail on dependency lookup, not on config type.
	conf := resource.Config{Name: "d", API: generic.API, Model: FixtureTeacher, ConvertedAttributes: Config{Arm: "arm"}}
	_, err := newFixtureTeacher(context.Background(), resource.Dependencies{}, conf, logging.NewTestLogger(t))
	if err == nil || strings.Contains(err.Error(), "incorrect config type") {
		t.Fatalf("unexpected error: %v", err)
	}
}

func TestValidate(t *testing.T) {
	cfg := Config{Arm: "arm", TCP: "gripper-tcp", PoseTracker: "tag-tracker", Marker: float64(6), ViewSwitch: "view"}
	deps, _, err := cfg.Validate("services.0")
	if err != nil {
		t.Fatal(err)
	}
	want := []string{"arm", "gripper-tcp", "builtin", "tag-tracker", "view"}
	if strings.Join(deps, ",") != strings.Join(want, ",") {
		t.Fatalf("deps = %v, want %v", deps, want)
	}
	if _, _, err := (Config{Arm: "arm", TCP: "t", PoseTracker: "p"}).Validate("x"); err == nil {
		t.Fatal("expected missing marker error")
	}
	if _, _, err := (Config{Arm: "arm", TCP: "t", PoseTracker: "p", Marker: "6", SampleHz: 5000}).Validate("x"); err == nil {
		t.Fatal("expected sample_hz bound error")
	}
	if got := markerString(float64(6)); got != "6" {
		t.Fatalf("markerString = %q", got)
	}
}
