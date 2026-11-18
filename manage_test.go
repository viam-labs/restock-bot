package restockbot

import (
	"testing"
	"time"

	"go.viam.com/rdk/app"
)

func TestOwnedMotion(t *testing.T) {
	if _, err := ownedMotion(&app.Sequence{ID: "s", SequenceTags: []string{"fixture:bin", "motion:open"}}, "drawer"); err == nil {
		t.Fatal("other fixture's sequence accepted")
	}
	if m, err := ownedMotion(&app.Sequence{SequenceTags: []string{"fixture:drawer"}}, "drawer"); err != nil || m != "" {
		t.Fatalf("got %q, %v", m, err)
	}
	if m, err := ownedMotion(&app.Sequence{SequenceTags: []string{"fixture:drawer", "motion:open"}}, "drawer"); err != nil || m != "open" {
		t.Fatalf("got %q, %v", m, err)
	}
}

func TestGroupMotions(t *testing.T) {
	now := time.Date(2026, 10, 29, 12, 0, 0, 0, time.UTC)
	seq := func(id, fixture, motion string, created time.Time) *app.Sequence {
		tags := []string{"fixture:" + fixture, "taught:freedrive"}
		if motion != "" {
			tags = append(tags, "motion:"+motion)
		}
		return &app.Sequence{ID: id, SequenceTags: tags, CreatedAt: created, StartTime: now, EndTime: now.Add(12 * time.Second)}
	}
	got := groupMotions([]*app.Sequence{
		seq("old", "drawer", "open", now.Add(-time.Hour)),
		seq("new", "drawer", "open", now),
		seq("close", "drawer", "close", now),
		seq("bin", "bin", "open", now),
		seq("untagged", "drawer", "", now),
	}, "drawer")
	if len(got) != 2 {
		t.Fatalf("got %v", got)
	}
	open := got["open"].([]any)
	first := open[0].(map[string]any)
	if len(open) != 2 || first["sequence_id"] != "new" || first["taught"] != "freedrive" ||
		first["duration_s"] != 12.0 || first["created"] != "2026-10-29T12:00:00Z" {
		t.Fatalf("got %v", open)
	}
}

func TestOptString(t *testing.T) {
	if v, err := optString(map[string]any{}, "sequence_id"); v != "" || err != nil {
		t.Fatalf("absent: %q, %v", v, err)
	}
	for _, bad := range []any{"", 3.0, nil} {
		if _, err := optString(map[string]any{"sequence_id": bad}, "sequence_id"); err == nil {
			t.Fatalf("%v accepted", bad)
		}
	}
}
