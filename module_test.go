package restockbot

import (
	"context"
	"math"
	"strings"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"
)

func TestTagID(t *testing.T) {
	if got := tagIDString(float64(6)); got != "6" {
		t.Fatalf("tagIDString(6.0) = %q", got)
	}
	if !tagMatches("tag36h11-6", "6") || !tagMatches("6", "6") || tagMatches("16", "6") {
		t.Fatal("tagMatches mismatch")
	}
}

func TestTiltKeepsPositionAndPitches(t *testing.T) {
	cur := spatialmath.NewPose(r3.Vector{X: 100, Y: 20, Z: 300}, &spatialmath.OrientationVectorDegrees{OX: 1, Theta: 0})
	got := tiltPose(cur, 30)
	if !spatialmath.R3VectorAlmostEqual(got.Point(), cur.Point(), 1e-6) {
		t.Fatalf("position moved: %v", got.Point())
	}
	// Angle between the old and new tool +Z axes should be the tilt.
	a := cur.Orientation().OrientationVectorRadians().Vector()
	b := got.Orientation().OrientationVectorRadians().Vector()
	if deg := math.Acos(a.Normalize().Dot(b.Normalize())) * 180 / math.Pi; math.Abs(deg-30) > 1e-6 {
		t.Fatalf("tilt = %v deg, want 30", deg)
	}
}

func TestConstructorAcceptsRegisteredConfigType(t *testing.T) {
	// viam-server hands the constructor the type registered in init(); empty deps should
	// fail on dependency lookup, not on config type.
	conf := resource.Config{Name: "d", API: generic.API, Model: DrawOpener, ConvertedAttributes: Config{}}
	_, err := newRestockBotDrawOpener(context.Background(), resource.Dependencies{}, conf, logging.NewTestLogger(t))
	if err == nil || strings.Contains(err.Error(), "incorrect config type") {
		t.Fatalf("unexpected error: %v", err)
	}
}
