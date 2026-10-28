package restockbot

import (
	"context"
	"fmt"
	"math"
	"sort"
	"strings"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/posetracker"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

var DrawOpener = resource.NewModel("viam-labs", "restock-bot", "draw-opener")

func init() {
	resource.RegisterService(generic.API, DrawOpener,
		resource.Registration[resource.Resource, Config]{
			Constructor: newRestockBotDrawOpener,
		},
	)
}

const (
	// Tilt the TCP nose-down by this many degrees, in place.
	tiltDeg = 30.0
	// Retract this far along world -X, with the linear constraint applied.
	retractMM                = 250.0
	lineToleranceMM          = 5.0
	orientationToleranceDegs = 5.0

	// Countertop obstacle, built in the world frame relative to the tag pose.
	countertopAboveMM     = 150.0
	countertopThicknessMM = 52.0
	countertopDepthMM     = 1500.0
	countertopLengthMM    = 3000.0

	tagFrame = "tag-6"

	findDrawerTimeout = 60 * time.Second
	findDrawerPoll    = 500 * time.Millisecond
)

type Config struct {
	Arm         string `json:"arm,omitempty"`
	Motion      string `json:"motion,omitempty"`
	PoseTracker string `json:"pose_tracker,omitempty"`
	Switch      string `json:"switch,omitempty"`
	TCP         string `json:"tcp,omitempty"`
	// TagID may be configured as a number or a string.
	TagID any `json:"tag_id,omitempty"`

	// Tuning knobs for the TCP target relative to the tag frame.
	AboveMM    float64  `json:"above_mm,omitempty"`
	BehindMM   float64  `json:"behind_mm,omitempty"`
	ApproachMM *float64 `json:"approach_mm,omitempty"`
}

func orDefault(s, def string) string {
	if s == "" {
		return def
	}
	return s
}

func (cfg Config) withDefaults() Config {
	c := cfg
	c.Arm = orDefault(c.Arm, "arm")
	c.Motion = orDefault(c.Motion, "builtin")
	c.PoseTracker = orDefault(c.PoseTracker, "tag-tracker")
	c.Switch = orDefault(c.Switch, "find-drawer-position")
	c.TCP = orDefault(c.TCP, "gripper-tcp")
	if c.TagID == nil {
		c.TagID = "1"
	}
	if c.ApproachMM == nil {
		v := 100.0
		c.ApproachMM = &v
	}
	return c
}

// Validate declares dependencies. tcp is only used as a frame name, but declaring it
// ensures the frame exists before we plan against it.
func (cfg Config) Validate(path string) ([]string, []string, error) {
	c := cfg.withDefaults()
	return []string{c.Arm, c.Motion, c.PoseTracker, c.Switch, c.TCP}, nil, nil
}

type restockBotDrawOpener struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name   resource.Name
	logger logging.Logger
	cfg    Config
	tagID  string

	motion      motion.Service
	poseTracker posetracker.PoseTracker
	sw          toggleswitch.Switch

	mu sync.Mutex
	// Cached world-frame pose of the tag, resolved by locate_tag.
	tagWorldPose spatialmath.Pose
}

func newRestockBotDrawOpener(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[Config](rawConf)
	if err != nil {
		return nil, err
	}
	return NewDrawOpener(ctx, deps, rawConf.ResourceName(), conf.withDefaults(), logger)
}

func NewDrawOpener(ctx context.Context, deps resource.Dependencies, name resource.Name, conf Config, logger logging.Logger) (resource.Resource, error) {
	conf = conf.withDefaults()
	not, err := motion.FromDependencies(deps, conf.Motion)
	if err != nil {
		return nil, err
	}
	pt, err := posetracker.FromDependencies(deps, conf.PoseTracker)
	if err != nil {
		return nil, err
	}
	sw, err := toggleswitch.FromDependencies(deps, conf.Switch)
	if err != nil {
		return nil, err
	}
	s := &restockBotDrawOpener{
		name:        name,
		logger:      logger,
		cfg:         conf,
		tagID:       tagIDString(conf.TagID),
		motion:      not,
		poseTracker: pt,
		sw:          sw,
	}
	logger.Infof("drawer-opener configured: motion=%s pose_tracker=%s switch=%s tcp=%s tag_id=%s",
		conf.Motion, conf.PoseTracker, conf.Switch, conf.TCP, s.tagID)
	return s, nil
}

func (s *restockBotDrawOpener) Name() resource.Name {
	return s.name
}

// tagIDString normalizes a JSON number (decoded as float64) or string tag id.
func tagIDString(v any) string {
	if f, ok := v.(float64); ok {
		return fmt.Sprint(int(f))
	}
	return fmt.Sprint(v)
}

// tagMatches accepts pose tracker keys of the form "6" or "<prefix>-6".
func tagMatches(key, tagID string) bool {
	return key == tagID || strings.HasSuffix(key, "-"+tagID)
}

func poseToMap(p spatialmath.Pose) map[string]any {
	pt := p.Point()
	o := p.Orientation().OrientationVectorDegrees()
	return map[string]any{"x": pt.X, "y": pt.Y, "z": pt.Z, "o_x": o.OX, "o_y": o.OY, "o_z": o.OZ, "theta": o.Theta}
}

func explainMoveError(err error) string {
	msg := err.Error()
	low := strings.ToLower(msg)
	if strings.Contains(low, "cbirrt") || strings.Contains(low, "no direct solutions") {
		return "The straight-line (linear constraint) path specifically is infeasible: the planner could not " +
			"find a direct solution along the line. This does NOT mean the goal pose is unreachable -- it may be " +
			"reachable without the linear constraint, with looser tolerances, a shorter retract, or from a " +
			"different start pose. Underlying error: " + msg
	}
	return msg
}

func fail(err error) map[string]any {
	return map[string]any{"ok": false, "error": err.Error()}
}

// tiltPose pitches the pose nose-down about its local +Y axis, keeping position fixed.
func tiltPose(cur spatialmath.Pose, deg float64) spatialmath.Pose {
	return spatialmath.Compose(cur, spatialmath.NewPoseFromOrientation(&spatialmath.R4AA{Theta: deg * math.Pi / 180, RY: 1}))
}

// worldState carries the tag frame (so tag-relative goals resolve) and the countertop obstacle.
func (s *restockBotDrawOpener) worldState() (*referenceframe.WorldState, error) {
	s.mu.Lock()
	tag := s.tagWorldPose
	s.mu.Unlock()
	if tag == nil {
		s.logger.Warn("tag world pose unknown -- this move is planned WITHOUT the countertop obstacle")
		return referenceframe.NewWorldState(nil, nil)
	}
	tp := tag.Point()
	countertop, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{X: tp.X - countertopDepthMM/2, Y: tp.Y, Z: tp.Z + countertopAboveMM}),
		r3.Vector{X: countertopDepthMM, Y: countertopLengthMM, Z: countertopThicknessMM},
		"countertop",
	)
	if err != nil {
		return nil, err
	}
	return referenceframe.NewWorldState(
		[]*referenceframe.GeometriesInFrame{referenceframe.NewGeometriesInFrame(referenceframe.World, []spatialmath.Geometry{countertop})},
		[]*referenceframe.LinkInFrame{referenceframe.NewLinkInFrame(referenceframe.World, tag, tagFrame, nil)},
	)
}

func (s *restockBotDrawOpener) move(ctx context.Context, dest *referenceframe.PoseInFrame, constraints *motionplan.Constraints) map[string]any {
	ws, err := s.worldState()
	if err != nil {
		return fail(err)
	}
	ok, err := s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.cfg.TCP,
		Destination:   dest,
		WorldState:    ws,
		Constraints:   constraints,
	})
	if err != nil {
		msg := explainMoveError(err)
		s.logger.Errorf("move failed: %s", msg)
		return map[string]any{"ok": false, "error": msg}
	}
	return map[string]any{"ok": ok}
}

func (s *restockBotDrawOpener) currentTCPWorldPose(ctx context.Context) (spatialmath.Pose, error) {
	pif, err := s.motion.GetPose(ctx, s.cfg.TCP, referenceframe.World, nil, nil)
	if err != nil {
		return nil, err
	}
	return pif.Pose(), nil
}

// ensureTag locates the tag if no world pose is cached yet.
func (s *restockBotDrawOpener) ensureTag(ctx context.Context) map[string]any {
	s.mu.Lock()
	known := s.tagWorldPose != nil
	s.mu.Unlock()
	if known {
		return map[string]any{"ok": true}
	}
	s.logger.Info("no cached tag world pose -- running locate_tag first")
	return s.locateTag(ctx)
}

// findDrawer triggers the arm-position-saver switch and waits for it to return to 0.
func (s *restockBotDrawOpener) findDrawer(ctx context.Context) map[string]any {
	if err := s.sw.SetPosition(ctx, 2, nil); err != nil {
		return fail(err)
	}
	ctx, cancel := context.WithTimeout(ctx, findDrawerTimeout)
	defer cancel()
	ticker := time.NewTicker(findDrawerPoll)
	defer ticker.Stop()
	for {
		pos, err := s.sw.GetPosition(ctx, nil)
		if err != nil {
			return fail(err)
		}
		if pos == 0 {
			return map[string]any{"ok": true}
		}
		select {
		case <-ctx.Done():
			return fail(fmt.Errorf("waiting for switch to return to 0: %w", ctx.Err()))
		case <-ticker.C:
		}
	}
}

// locateTag reads the tag from the pose tracker and resolves it into the world frame.
func (s *restockBotDrawOpener) locateTag(ctx context.Context) map[string]any {
	poses, err := s.poseTracker.Poses(ctx, nil, nil)
	if err != nil {
		return fail(err)
	}
	keys := make([]string, 0, len(poses))
	for k := range poses {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	var match string
	for _, k := range keys {
		if tagMatches(k, s.tagID) {
			match = k
			break
		}
	}
	if match == "" {
		return fail(fmt.Errorf("tag id %q not found; pose tracker reported keys: %v", s.tagID, keys))
	}

	pif := poses[match]
	cameraFrame := orDefault(pif.Parent(), "cam")
	link := referenceframe.NewLinkInFrame(cameraFrame, pif.Pose(), tagFrame, nil)
	world, err := s.motion.GetPose(ctx, tagFrame, referenceframe.World, []*referenceframe.LinkInFrame{link}, nil)
	if err != nil {
		res := fail(err)
		res["camera"] = poseToMap(pif.Pose())
		return res
	}

	s.mu.Lock()
	s.tagWorldPose = world.Pose()
	s.mu.Unlock()
	s.logger.Infof("locate_tag: %s in %s -> world %v", match, cameraFrame, poseToMap(world.Pose()))
	return map[string]any{
		"ok":           true,
		"tag":          match,
		"camera_frame": cameraFrame,
		"camera":       poseToMap(pif.Pose()),
		"world":        poseToMap(world.Pose()),
	}
}

// moveInTagFrame moves the TCP to a point behind/above the tag, matching the tag's orientation.
func (s *restockBotDrawOpener) moveInTagFrame(ctx context.Context, step string, behind float64) map[string]any {
	if res := s.ensureTag(ctx); res["ok"] != true {
		return res
	}
	target := spatialmath.NewPoseFromPoint(r3.Vector{X: -behind, Z: s.cfg.AboveMM})
	s.logger.Infof("%s: moving %s to %v in frame %s", step, s.cfg.TCP, poseToMap(target), tagFrame)
	res := s.move(ctx, referenceframe.NewPoseInFrame(tagFrame, target), nil)
	res["target"] = poseToMap(target)
	return res
}

func (s *restockBotDrawOpener) approach(ctx context.Context) map[string]any {
	return s.moveInTagFrame(ctx, "approach", s.cfg.BehindMM+*s.cfg.ApproachMM)
}

func (s *restockBotDrawOpener) goal(ctx context.Context) map[string]any {
	return s.moveInTagFrame(ctx, "goal", s.cfg.BehindMM)
}

func (s *restockBotDrawOpener) tilt(ctx context.Context) map[string]any {
	if res := s.ensureTag(ctx); res["ok"] != true {
		return res
	}
	cur, err := s.currentTCPWorldPose(ctx)
	if err != nil {
		return fail(err)
	}
	target := tiltPose(cur, tiltDeg)
	s.logger.Infof("tilt: %v deg nose-down to %v", tiltDeg, poseToMap(target))
	res := s.move(ctx, referenceframe.NewPoseInFrame(referenceframe.World, target), nil)
	res["from"] = poseToMap(cur)
	res["target"] = poseToMap(target)
	return res
}

func (s *restockBotDrawOpener) retract(ctx context.Context) map[string]any {
	if res := s.ensureTag(ctx); res["ok"] != true {
		return res
	}
	cur, err := s.currentTCPWorldPose(ctx)
	if err != nil {
		return fail(err)
	}
	target := spatialmath.NewPose(cur.Point().Sub(r3.Vector{X: retractMM}), cur.Orientation())
	constraints := &motionplan.Constraints{LinearConstraint: []motionplan.LinearConstraint{
		{LineToleranceMm: lineToleranceMM, OrientationToleranceDegs: orientationToleranceDegs},
	}}
	s.logger.Infof("retract: %vmm along world -X to %v (linear constraint)", retractMM, poseToMap(target))
	res := s.move(ctx, referenceframe.NewPoseInFrame(referenceframe.World, target), constraints)
	res["from"] = poseToMap(cur)
	res["target"] = poseToMap(target)
	return res
}

func (s *restockBotDrawOpener) handlers() map[string]func(context.Context) map[string]any {
	return map[string]func(context.Context) map[string]any{
		"find_drawer":  s.findDrawer,
		"locate_tag":   s.locateTag,
		"approach":     s.approach,
		"goal":         s.goal,
		"tilt":         s.tilt,
		"retract":      s.retract,
		"run_sequence": s.runSequence,
	}
}

// runSequence chains every step in order, stopping at the first failure.
func (s *restockBotDrawOpener) runSequence(ctx context.Context) map[string]any {
	h := s.handlers()
	results := []any{}
	for _, step := range []string{"find_drawer", "locate_tag", "approach", "goal", "tilt", "retract"} {
		s.logger.Infof("run_sequence: step %s", step)
		res := h[step](ctx)
		res["step"] = step
		results = append(results, res)
		if res["ok"] != true {
			s.logger.Errorf("run_sequence: stopping at failed step %s", step)
			return map[string]any{"ok": false, "failed_step": step, "steps": results}
		}
	}
	return map[string]any{"ok": true, "steps": results}
}

// DoCommand dispatches {"cmd": "<name>"}; failures are reported in the result, never returned as errors.
func (s *restockBotDrawOpener) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	h := s.handlers()
	names := make([]string, 0, len(h))
	for k := range h {
		names = append(names, k)
	}
	sort.Strings(names)
	available := strings.Join(names, ", ")

	name, ok := cmd["cmd"].(string)
	if !ok {
		return map[string]any{"ok": false, "error": "command must include a string 'cmd' field", "available": available}, nil
	}
	fn, ok := h[name]
	if !ok {
		return map[string]any{"ok": false, "error": fmt.Sprintf("unknown cmd %q", name), "available": available}, nil
	}
	return fn(ctx), nil
}

func (s *restockBotDrawOpener) Status(ctx context.Context) (map[string]interface{}, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return map[string]any{
		"tag_id":               s.tagID,
		"tcp":                  s.cfg.TCP,
		"tag_world_pose_known": s.tagWorldPose != nil,
	}, nil
}
