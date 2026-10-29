package restockbot

import (
	"context"
	"errors"
	"fmt"
	"strings"
	"sync"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/posetracker"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

var FixtureTeacher = resource.NewModel("viam-labs", "restock-bot", "fixture-teacher")

func init() {
	resource.RegisterService(generic.API, FixtureTeacher,
		resource.Registration[resource.Resource, Config]{
			Constructor: newFixtureTeacher,
		},
	)
}

// Vec3 is a box size in mm.
type Vec3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// PoseConfig is a pose in the marker frame; orientation vector with theta in degrees.
type PoseConfig struct {
	X     float64  `json:"x"`
	Y     float64  `json:"y"`
	Z     float64  `json:"z"`
	OX    float64  `json:"o_x"`
	OY    float64  `json:"o_y"`
	OZ    *float64 `json:"o_z,omitempty"`
	Theta float64  `json:"theta"`
}

func (p PoseConfig) pose() spatialmath.Pose {
	oz := 1.0
	if p.OZ != nil {
		oz = *p.OZ
	}
	return spatialmath.NewPose(
		r3.Vector{X: p.X, Y: p.Y, Z: p.Z},
		&spatialmath.OrientationVectorDegrees{OX: p.OX, OY: p.OY, OZ: oz, Theta: p.Theta},
	)
}

type Obstacle struct {
	Label  string     `json:"label"`
	Pose   PoseConfig `json:"pose"`
	DimsMM Vec3       `json:"dims_mm"`
}

type Config struct {
	Fixture     string `json:"fixture,omitempty"`
	Arm         string `json:"arm"`
	TCP         string `json:"tcp"`
	Motion      string `json:"motion,omitempty"`
	PoseTracker string `json:"pose_tracker"`
	// Marker may be configured as a number or a string.
	Marker     any        `json:"marker"`
	ViewSwitch string     `json:"view_switch,omitempty"`
	SampleHz   float64    `json:"sample_hz,omitempty"`
	Obstacles  []Obstacle `json:"obstacles,omitempty"`
}

func orDefault(s, def string) string {
	if s == "" {
		return def
	}
	return s
}

func (cfg Config) Validate(path string) ([]string, []string, error) {
	for _, f := range []struct{ name, val string }{{"arm", cfg.Arm}, {"tcp", cfg.TCP}, {"pose_tracker", cfg.PoseTracker}} {
		if f.val == "" {
			return nil, nil, resource.NewConfigValidationFieldRequiredError(path, f.name)
		}
	}
	if cfg.Marker == nil {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "marker")
	}
	if cfg.SampleHz < 0 || cfg.SampleHz > maxSampleHz {
		return nil, nil, fmt.Errorf("%s: sample_hz must be between 0 and %v", path, maxSampleHz)
	}
	deps := []string{cfg.Arm, cfg.TCP, orDefault(cfg.Motion, "builtin"), cfg.PoseTracker}
	if cfg.ViewSwitch != "" {
		deps = append(deps, cfg.ViewSwitch)
	}
	return deps, nil, nil
}

type fixtureTeacher struct {
	resource.AlwaysRebuild

	name    resource.Name
	logger  logging.Logger
	cfg     Config
	fixture string
	marker  string

	arm        arm.Arm
	motion     motion.Service
	tracker    posetracker.PoseTracker
	viewSwitch toggleswitch.Switch // nil when view_switch is unset
	obstacles  []spatialmath.Geometry
	store      *store

	mu          sync.Mutex
	closed      bool
	busy        string // "", "teach" or "run"
	markerWorld spatialmath.Pose
	session     *session
	cancelRun   context.CancelFunc
	runDone     chan struct{}
}

func newFixtureTeacher(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[Config](rawConf)
	if err != nil {
		return nil, err
	}
	return NewFixtureTeacher(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewFixtureTeacher(ctx context.Context, deps resource.Dependencies, name resource.Name, conf Config, logger logging.Logger) (resource.Resource, error) {
	conf.Motion = orDefault(conf.Motion, "builtin")
	if conf.SampleHz <= 0 {
		conf.SampleHz = 30
	}
	a, err := arm.FromDependencies(deps, conf.Arm)
	if err != nil {
		return nil, err
	}
	mot, err := motion.FromDependencies(deps, conf.Motion)
	if err != nil {
		return nil, err
	}
	pt, err := posetracker.FromDependencies(deps, conf.PoseTracker)
	if err != nil {
		return nil, err
	}
	var sw toggleswitch.Switch
	if conf.ViewSwitch != "" {
		if sw, err = toggleswitch.FromDependencies(deps, conf.ViewSwitch); err != nil {
			return nil, err
		}
	}
	obstacles := make([]spatialmath.Geometry, 0, len(conf.Obstacles))
	for _, o := range conf.Obstacles {
		box, err := spatialmath.NewBox(o.Pose.pose(), r3.Vector{X: o.DimsMM.X, Y: o.DimsMM.Y, Z: o.DimsMM.Z}, o.Label)
		if err != nil {
			return nil, fmt.Errorf("obstacle %q: %w", o.Label, err)
		}
		obstacles = append(obstacles, box)
	}
	return &fixtureTeacher{
		name:       name,
		logger:     logger,
		cfg:        conf,
		fixture:    orDefault(conf.Fixture, name.ShortName()),
		marker:     markerString(conf.Marker),
		arm:        a,
		motion:     mot,
		tracker:    pt,
		viewSwitch: sw,
		obstacles:  obstacles,
		store:      newStore(name.ShortName(), logger),
	}, nil
}

func (s *fixtureTeacher) Name() resource.Name {
	return s.name
}

// markerString normalizes a JSON number (decoded as float64) or string marker id.
func markerString(v any) string {
	if f, ok := v.(float64); ok {
		return fmt.Sprint(int(f))
	}
	return fmt.Sprint(v)
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
			"reachable without the linear constraint or with a different waypoint. Underlying error: " + msg
	}
	return msg
}

func fail(err error) map[string]any {
	return map[string]any{"ok": false, "error": err.Error()}
}

// claimLocked marks the service busy with a teach session or run; the caller holds s.mu.
func (s *fixtureTeacher) claimLocked(kind string) error {
	if s.closed {
		return errors.New("service is closing")
	}
	if s.busy != "" {
		return fmt.Errorf("busy: %s in progress", s.busy)
	}
	s.busy = kind
	return nil
}

// claim marks the service busy with a teach session or run; only one may be active.
func (s *fixtureTeacher) claim(kind string) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.claimLocked(kind)
}

func (s *fixtureTeacher) release() {
	s.mu.Lock()
	s.busy = ""
	s.mu.Unlock()
}

const availableCmds = "start_teach, mark, end_teach, run, stop"

// DoCommand dispatches {"cmd": ...}; failures are reported in the result, never returned as errors.
func (s *fixtureTeacher) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	name, _ := cmd["cmd"].(string)
	motionName, _ := cmd["motion"].(string)
	switch name {
	case "start_teach":
		return s.startTeach(ctx, motionName), nil
	case "mark":
		label, _ := cmd["label"].(string)
		linear, _ := cmd["linear"].(bool)
		return s.mark(label, linear), nil
	case "end_teach":
		return s.endTeach(ctx), nil
	case "run":
		dryRun, ok := cmd["dry_run"].(bool)
		if _, present := cmd["dry_run"]; present && !ok {
			return fail(errors.New("dry_run must be a boolean")), nil
		}
		return s.run(ctx, motionName, dryRun), nil
	case "stop":
		return s.stop(ctx), nil
	default:
		return map[string]any{"ok": false, "error": fmt.Sprintf("unknown cmd %q", name), "available": availableCmds}, nil
	}
}

func (s *fixtureTeacher) Status(ctx context.Context) (map[string]interface{}, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	teaching := ""
	if s.session != nil {
		teaching = s.session.motion
	}
	return map[string]any{
		"fixture":      s.fixture,
		"marker_known": s.markerWorld != nil,
		"teaching":     teaching,
	}, nil
}

func (s *fixtureTeacher) Close(ctx context.Context) error {
	s.mu.Lock()
	s.closed = true
	sess, cancel, runDone := s.session, s.cancelRun, s.runDone
	s.session = nil
	s.mu.Unlock()
	if cancel != nil {
		// Cancellation alone may not halt a trajectory, so stop the arm too, then let run unwind.
		cancel()
		if err := s.arm.Stop(ctx, nil); err != nil {
			s.logger.Errorf("stopping arm on close: %v", err)
		}
		select {
		case <-runDone:
		case <-ctx.Done():
		}
	}
	if sess != nil {
		sess.cancel()
		<-sess.done
		s.endSession(sess)
	}
	return s.store.close()
}

var errNoMotion = errors.New("command requires a 'motion' name")
