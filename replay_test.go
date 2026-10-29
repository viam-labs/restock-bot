package restockbot

import (
	"testing"

	"go.viam.com/rdk/spatialmath"
)

func TestWaypoints(t *testing.T) {
	p := spatialmath.NewZeroPose()
	samples := []Sample{{T: 0, Pose: p}, {T: 1, Pose: p, Mark: "hooked"}, {T: 2, Pose: p}, {T: 3, Pose: p, Mark: "pulled", Linear: true}}
	got := waypoints(samples)
	if len(got) != 2 || got[0].Mark != "hooked" || got[1].Mark != "pulled" || !got[1].Linear {
		t.Fatalf("got %+v", got)
	}
}
