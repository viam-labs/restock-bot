package restockbot

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

func TestMarkerRoundTrip(t *testing.T) {
	marker := spatialmath.NewPose(r3.Vector{X: 400, Y: -20, Z: 100}, &spatialmath.OrientationVectorDegrees{OX: 1, Theta: 90})
	tcp := spatialmath.NewPose(r3.Vector{X: 300, Y: 10, Z: 150}, &spatialmath.OrientationVectorDegrees{OX: 0.5, OZ: 1, Theta: 20})
	if got := toWorld(marker, toMarker(marker, tcp)); !spatialmath.PoseAlmostEqual(got, tcp) {
		t.Fatalf("round trip: got %v want %v", got, tcp)
	}
}

func TestToMarkerKnownValue(t *testing.T) {
	// Marker at x=100, rotated 90 deg about Z. World point (100, 10) is 10 mm along the marker's +X.
	m := spatialmath.NewPose(r3.Vector{X: 100}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 90})
	got := toMarker(m, spatialmath.NewPoseFromPoint(r3.Vector{X: 100, Y: 10})).Point()
	if !spatialmath.R3VectorAlmostEqual(got, r3.Vector{X: 10}, 1e-6) {
		t.Fatalf("toMarker = %v, want (10, 0, 0)", got)
	}
}

func TestAveragePoses(t *testing.T) {
	p := spatialmath.NewPose(r3.Vector{X: 1, Y: 2, Z: 3}, &spatialmath.OrientationVectorDegrees{OX: 1, Theta: 30})
	if got := averagePoses([]spatialmath.Pose{p, p, p}); !spatialmath.PoseAlmostEqual(got, p) {
		t.Fatalf("identical: got %v want %v", got, p)
	}
	// -q is the same rotation as q; averaging must not cancel them out.
	neg := spatialmath.Quaternion(quat.Scale(-1, p.Orientation().Quaternion()))
	flipped := spatialmath.NewPose(p.Point(), &neg)
	if got := averagePoses([]spatialmath.Pose{p, flipped}); !spatialmath.PoseAlmostEqual(got, p) {
		t.Fatalf("opposite hemispheres: got %v want %v", got, p)
	}
	a := spatialmath.NewPoseFromPoint(r3.Vector{X: 0})
	b := spatialmath.NewPoseFromPoint(r3.Vector{X: 10})
	if got := averagePoses([]spatialmath.Pose{a, b}).Point(); !spatialmath.R3VectorAlmostEqual(got, r3.Vector{X: 5}, 1e-9) {
		t.Fatalf("position mean: got %v", got)
	}
}

func TestFindMarker(t *testing.T) {
	readings := referenceframe.FrameSystemPoses{"tag36h11-16": nil, "tag36h11-6": nil}
	if key, ok := findMarker(readings, "6"); !ok || key != "tag36h11-6" {
		t.Fatalf("got %q %v", key, ok)
	}
	if _, ok := findMarker(readings, "7"); ok {
		t.Fatal("matched missing marker")
	}
}
