package restockbot

import (
	"context"
	"errors"
	"strings"
	"testing"

	"go.viam.com/rdk/testutils/inject"
)

func TestMarkRejectsPending(t *testing.T) {
	sess := &session{}
	s := &fixtureTeacher{session: sess}
	if res := s.mark("a", false); res["ok"] != true {
		t.Fatal(res)
	}
	if res := s.mark("b", false); res["ok"] != false {
		t.Fatalf("second pending mark accepted: %v", res)
	}
	if sess.markLabel != "a" {
		t.Fatalf("pending mark overwritten: %q", sess.markLabel)
	}
}

func TestGripperAction(t *testing.T) {
	ctx := context.Background()
	if res := (&fixtureTeacher{session: &session{}}).gripperAction(ctx, "grab", ""); res["ok"] != false {
		t.Fatalf("accepted without a gripper: %v", res)
	}
	g := inject.NewGripper("gripper")
	g.GrabFunc = func(context.Context, map[string]interface{}) (bool, error) { return true, nil }
	sess := &session{}
	s := &fixtureTeacher{session: sess, gripper: g}
	if res := s.gripperAction(ctx, "close", ""); res["ok"] != false {
		t.Fatalf("accepted bad action: %v", res)
	}
	res := s.gripperAction(ctx, "grab", "")
	if res["ok"] != true || res["holding"] != true || sess.markLabel != "grab" || sess.markGripper != gripperGrab {
		t.Fatalf("res %v, pending %q/%q", res, sess.markLabel, sess.markGripper)
	}
	// The first mark is still pending, so this one is refused before the gripper moves.
	grabs := 0
	g.GrabFunc = func(context.Context, map[string]interface{}) (bool, error) { grabs++; return true, nil }
	res = s.gripperAction(ctx, "grab", "again")
	if res["ok"] != false || grabs != 0 || !strings.Contains(res["error"].(string), "previous mark not yet captured") {
		t.Fatalf("got %v", res)
	}
	g.OpenFunc = func(context.Context, map[string]interface{}) error { return errors.New("jammed") }
	sess.hasMark = false
	if res := s.gripperAction(ctx, "open", ""); res["ok"] != false || sess.hasMark {
		t.Fatalf("failed actuation recorded: %v", res)
	}
}

func TestGripperActionSessionEnded(t *testing.T) {
	s := &fixtureTeacher{session: &session{}}
	g := inject.NewGripper("gripper")
	g.GrabFunc = func(context.Context, map[string]interface{}) (bool, error) {
		s.mu.Lock()
		s.session = nil
		s.mu.Unlock()
		return true, nil
	}
	s.gripper = g
	res := s.gripperAction(context.Background(), "grab", "")
	if res["ok"] != false || !strings.Contains(res["error"].(string), "ran but was not recorded") {
		t.Fatalf("got %v", res)
	}
}
