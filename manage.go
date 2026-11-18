package restockbot

import (
	"context"
	"errors"
	"fmt"
	"slices"
	"sort"
	"strings"
	"time"

	"go.viam.com/rdk/app"
)

// tagValue returns the value of seq's first tag with prefix, or "".
func tagValue(seq *app.Sequence, prefix string) string {
	for _, tag := range seq.SequenceTags {
		if v, ok := strings.CutPrefix(tag, prefix); ok {
			return v
		}
	}
	return ""
}

// ownedMotion returns seq's motion tag ("" if none), or an error if seq isn't this fixture's.
func ownedMotion(seq *app.Sequence, fixture string) (string, error) {
	if !slices.Contains(seq.SequenceTags, "fixture:"+fixture) {
		return "", fmt.Errorf("sequence %s does not belong to fixture %q", seq.ID, fixture)
	}
	return tagValue(seq, "motion:"), nil
}

// groupMotions lists the fixture's sequences by motion, newest first.
func groupMotions(seqs []*app.Sequence, fixture string) map[string]any {
	groups := map[string][]*app.Sequence{}
	for _, seq := range seqs {
		if m, err := ownedMotion(seq, fixture); err == nil && m != "" {
			groups[m] = append(groups[m], seq)
		}
	}
	out := make(map[string]any, len(groups))
	for m, group := range groups {
		sort.Slice(group, func(i, j int) bool { return group[i].CreatedAt.After(group[j].CreatedAt) })
		entries := make([]any, 0, len(group))
		for _, seq := range group {
			entries = append(entries, map[string]any{
				"sequence_id": seq.ID,
				"created":     seq.CreatedAt.UTC().Format(time.RFC3339),
				"taught":      tagValue(seq, "taught:"),
				"duration_s":  seq.EndTime.Sub(seq.StartTime).Seconds(),
			})
		}
		out[m] = entries
	}
	return out
}

func (s *fixtureTeacher) listMotions(ctx context.Context) map[string]any {
	seqs, err := s.store.listSequences(ctx)
	if err != nil {
		return fail(err)
	}
	return map[string]any{"ok": true, "motions": groupMotions(seqs, s.fixture)}
}

// deleteMotion deletes one of this fixture's sequences; its uploaded samples remain in Viam Data.
func (s *fixtureTeacher) deleteMotion(ctx context.Context, id string) map[string]any {
	if id == "" {
		return fail(errors.New("delete requires a 'sequence_id'"))
	}
	if err := s.claim("delete"); err != nil {
		return fail(err)
	}
	defer s.release()
	seq, err := s.store.getSequence(ctx, id)
	if err != nil {
		return fail(err)
	}
	motionName, err := ownedMotion(seq, s.fixture)
	if err != nil {
		return fail(err)
	}
	if err := s.store.deleteSequence(ctx, id); err != nil {
		return fail(err)
	}
	s.store.dropCache(s.fixture, motionName, id)
	s.logger.Infof("deleted sequence %s (%s/%s)", id, s.fixture, motionName)
	return map[string]any{"ok": true, "sequence_id": id, "motion": motionName}
}

// resolvePinned returns sequence id's samples and motion tag; it must be this fixture's, and motionName's if given.
func (s *fixtureTeacher) resolvePinned(ctx context.Context, id, motionName string) ([]Sample, string, error) {
	seq, err := s.store.getSequence(ctx, id)
	if err != nil {
		return nil, "", err
	}
	tagged, err := ownedMotion(seq, s.fixture)
	if err != nil {
		return nil, "", err
	}
	if motionName != "" && motionName != tagged {
		return nil, "", fmt.Errorf("sequence %s is motion %q, not %q", id, tagged, motionName)
	}
	samples, err := s.store.resolvePinned(ctx, seq)
	return samples, tagged, err
}
