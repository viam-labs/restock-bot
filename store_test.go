package restockbot

import (
	"path/filepath"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/app"
	"go.viam.com/rdk/spatialmath"
)

func TestSampleRoundTrip(t *testing.T) {
	in := Sample{
		T:      1.5,
		Pose:   spatialmath.NewPose(r3.Vector{X: 12, Y: -3, Z: 85}, &spatialmath.OrientationVectorDegrees{OX: 0.2, OZ: 1, Theta: 15}),
		Mark:   "pull",
		Linear: true,
	}
	for name, row := range map[string]map[string]any{
		"raw":       encodeSample(in),
		"wrapped":   {"readings": encodeSample(in)},
		"docommand": {"docommand_output": encodeSample(in)},
	} {
		out, err := decodeSample(row)
		if err != nil {
			t.Fatalf("%s: %v", name, err)
		}
		if out.T != in.T || out.Mark != in.Mark || out.Linear != in.Linear || !spatialmath.PoseAlmostEqual(out.Pose, in.Pose) {
			t.Fatalf("%s: got %+v want %+v", name, out, in)
		}
	}
	unmarked := encodeSample(Sample{Pose: spatialmath.NewZeroPose()})
	if _, ok := unmarked["mark"]; ok {
		t.Fatal("unmarked sample encoded a mark")
	}
	if _, err := decodeSample(map[string]any{"t": 1.0}); err == nil {
		t.Fatal("expected error for incomplete sample")
	}
}

func TestNewestSequence(t *testing.T) {
	now := time.Now()
	seqs := []*app.Sequence{
		{ID: "old", SequenceTags: []string{"fixture:drawer", "motion:open"}, CreatedAt: now.Add(-time.Hour)},
		{ID: "new", SequenceTags: []string{"fixture:drawer", "motion:open", "taught:jog"}, CreatedAt: now},
		{ID: "other-motion", SequenceTags: []string{"fixture:drawer", "motion:close"}, CreatedAt: now.Add(time.Hour)},
		{ID: "fixture-only", SequenceTags: []string{"fixture:drawer"}, CreatedAt: now.Add(time.Hour)},
	}
	if got := newestSequence(seqs, "drawer", "open"); got == nil || got.ID != "new" {
		t.Fatalf("got %v", got)
	}
	if got := newestSequence(seqs, "bin", "open"); got != nil {
		t.Fatalf("expected nil, got %v", got.ID)
	}
}

func TestCheckCacheName(t *testing.T) {
	for _, bad := range []string{"", ".", "..", "../x", "a/b"} {
		if checkCacheName(bad) == nil {
			t.Fatalf("%q accepted", bad)
		}
	}
	if err := checkCacheName("open"); err != nil {
		t.Fatal(err)
	}
}

func TestCacheRoundTrip(t *testing.T) {
	path := filepath.Join(t.TempDir(), "drawer", "open.json")
	in := Sample{T: 2, Pose: spatialmath.NewPose(r3.Vector{X: 5, Y: 6, Z: 7}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 30}), Mark: "hooked"}
	if err := writeCache(path, &cacheFile{SequenceID: "s1", Samples: []map[string]any{encodeSample(in)}}); err != nil {
		t.Fatal(err)
	}
	cf, err := readCache(path)
	if err != nil {
		t.Fatal(err)
	}
	out, err := decodeSample(cf.Samples[0])
	if err != nil {
		t.Fatal(err)
	}
	if cf.SequenceID != "s1" || out.Mark != "hooked" || !spatialmath.PoseAlmostEqual(out.Pose, in.Pose) {
		t.Fatalf("got %s %+v", cf.SequenceID, out)
	}
}
