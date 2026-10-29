package restockbot

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"slices"
	"sort"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/app"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

const (
	// Viam Data only ingests tabular data for known component/method pairs; DoCommand is accepted for any resource.
	teachMethod = "DoCommand"
	// DoCommand captures wrap their output under this key.
	doCommandOutputKey = "docommand_output"
	// Sequence windows are padded so boundary samples are always inside; sessions are seconds apart.
	sequencePad = time.Second
)

// Sample is one TCP pose in the marker frame.
type Sample struct {
	At     time.Time // capture time; not stored in the reading
	T      float64   // seconds since start_teach
	Pose   spatialmath.Pose
	Mark   string
	Linear bool
}

func encodeSample(smp Sample) map[string]any {
	pt := smp.Pose.Point()
	o := smp.Pose.Orientation().OrientationVectorDegrees()
	m := map[string]any{"t": smp.T, "x": pt.X, "y": pt.Y, "z": pt.Z, "o_x": o.OX, "o_y": o.OY, "o_z": o.OZ, "theta": o.Theta}
	if smp.Mark != "" {
		m["mark"] = smp.Mark
	}
	if smp.Linear {
		m["linear"] = true
	}
	return m
}

// decodeSample accepts a raw reading or one wrapped under "docommand_output" or "readings".
func decodeSample(row map[string]any) (Sample, error) {
	for _, key := range []string{doCommandOutputKey, "readings"} {
		if inner, ok := row[key].(map[string]any); ok {
			row = inner
			break
		}
	}
	var v [8]float64
	for i, k := range []string{"t", "x", "y", "z", "o_x", "o_y", "o_z", "theta"} {
		f, ok := row[k].(float64)
		if !ok {
			return Sample{}, fmt.Errorf("sample missing numeric %q: %v", k, row)
		}
		v[i] = f
	}
	mark, _ := row["mark"].(string)
	linear, _ := row["linear"].(bool)
	return Sample{
		T:      v[0],
		Pose:   spatialmath.NewPose(r3.Vector{X: v[1], Y: v[2], Z: v[3]}, &spatialmath.OrientationVectorDegrees{OX: v[4], OY: v[5], OZ: v[6], Theta: v[7]}),
		Mark:   mark,
		Linear: linear,
	}, nil
}

// newestSequence returns the most recently created sequence carrying both tags, or nil.
func newestSequence(seqs []*app.Sequence, fixture, motionName string) *app.Sequence {
	var best *app.Sequence
	for _, seq := range seqs {
		if !slices.Contains(seq.SequenceTags, "fixture:"+fixture) || !slices.Contains(seq.SequenceTags, "motion:"+motionName) {
			continue
		}
		if best == nil || seq.CreatedAt.After(best.CreatedAt) {
			best = seq
		}
	}
	return best
}

type cacheFile struct {
	SequenceID string           `json:"sequence_id"`
	Samples    []map[string]any `json:"samples"`
}

// store talks to Viam Data and keeps a local cache of fetched motions.
type store struct {
	logger       logging.Logger
	resourceName string
	partID       string
	orgID        string
	cacheDir     string

	mu     sync.Mutex
	client *app.ViamClient
}

func newStore(resourceName string, logger logging.Logger) *store {
	return &store{
		logger:       logger,
		resourceName: resourceName,
		partID:       os.Getenv(utils.MachinePartIDEnvVar),
		orgID:        os.Getenv(utils.PrimaryOrgIDEnvVar),
		cacheDir:     orDefault(os.Getenv("VIAM_MODULE_DATA"), os.TempDir()),
	}
}

// data lazily connects so the service still constructs while offline.
func (st *store) data(ctx context.Context) (*app.DataClient, error) {
	st.mu.Lock()
	defer st.mu.Unlock()
	if st.client == nil {
		c, err := app.CreateViamClientFromEnvVars(ctx, nil, st.logger)
		if err != nil {
			return nil, err
		}
		st.client = c
	}
	return st.client.DataClient(), nil
}

func (st *store) close() error {
	st.mu.Lock()
	defer st.mu.Unlock()
	if st.client == nil {
		return nil
	}
	err := st.client.Close()
	st.client = nil
	return err
}

func (st *store) upload(ctx context.Context, batch []Sample) error {
	dc, err := st.data(ctx)
	if err != nil {
		return err
	}
	rows := make([]map[string]any, len(batch))
	times := make([][2]time.Time, len(batch))
	for i, smp := range batch {
		rows[i] = map[string]any{doCommandOutputKey: encodeSample(smp)}
		times[i] = [2]time.Time{smp.At, smp.At}
	}
	_, err = dc.TabularDataCaptureUpload(ctx, rows, st.partID, generic.API.String(), st.resourceName, teachMethod, times, nil)
	return err
}

func (st *store) createSequence(ctx context.Context, tags []string, first, last time.Time) (string, error) {
	dc, err := st.data(ctx)
	if err != nil {
		return "", err
	}
	return dc.CreateSequence(ctx, st.partID,
		[]app.SequenceResourceFilter{{ResourceName: st.resourceName, MethodName: teachMethod}},
		tags, first.Add(-sequencePad), last.Add(sequencePad))
}

// checkCacheName rejects names that would escape the cache directory; motion names come from DoCommand input.
func checkCacheName(name string) error {
	if name == "" || name == "." || name == ".." || name != filepath.Base(name) {
		return fmt.Errorf("invalid name %q: must be a single path element", name)
	}
	return nil
}

// resolve returns the newest motion's samples, falling back to the local cache when the cloud is unreachable.
func (st *store) resolve(ctx context.Context, fixture, motionName string) (string, []Sample, error) {
	for _, name := range []string{fixture, motionName} {
		if err := checkCacheName(name); err != nil {
			return "", nil, err
		}
	}
	path := filepath.Join(st.cacheDir, fixture, motionName+".json")
	cached, cacheErr := readCache(path)
	cf, err := st.fetchNewest(ctx, fixture, motionName, cached)
	if err != nil {
		if cacheErr != nil {
			return "", nil, err
		}
		st.logger.Warnf("cloud lookup for %s/%s failed, using cached sequence %s: %v", fixture, motionName, cached.SequenceID, err)
		cf = cached
	}
	samples := make([]Sample, 0, len(cf.Samples))
	for _, row := range cf.Samples {
		smp, err := decodeSample(row)
		if err != nil {
			return "", nil, err
		}
		samples = append(samples, smp)
	}
	sort.SliceStable(samples, func(i, j int) bool { return samples[i].T < samples[j].T })
	// Only cache rows that decoded, so a bad fetch can't poison later offline runs.
	if cf != cached {
		if err := writeCache(path, cf); err != nil {
			st.logger.Warnf("writing cache %s: %v", path, err)
		}
	}
	return cf.SequenceID, samples, nil
}

func (st *store) fetchNewest(ctx context.Context, fixture, motionName string, cached *cacheFile) (*cacheFile, error) {
	dc, err := st.data(ctx)
	if err != nil {
		return nil, err
	}
	var all []*app.Sequence
	page, err := dc.ListSequences(ctx, st.orgID, 100)
	for ; err == nil && len(page.Sequences) > 0; page, err = page.NextPage(ctx) {
		all = append(all, page.Sequences...)
	}
	if err != nil {
		return nil, err
	}
	seq := newestSequence(all, fixture, motionName)
	if seq == nil {
		return nil, fmt.Errorf("no sequence tagged fixture:%s and motion:%s", fixture, motionName)
	}
	if cached != nil && cached.SequenceID == seq.ID {
		return cached, nil
	}
	if len(seq.Resources) == 0 {
		return nil, fmt.Errorf("sequence %s has no resources", seq.ID)
	}
	// The teaching machine's resource name, which may differ from this service's name.
	filter := &app.Filter{
		PartID:        seq.PartID,
		ComponentName: seq.Resources[0].ResourceName,
		Method:        teachMethod,
		Interval:      app.CaptureInterval{Start: seq.StartTime, End: seq.EndTime},
	}
	cf := &cacheFile{SequenceID: seq.ID}
	last := ""
	for {
		resp, err := dc.TabularDataByFilter(ctx, &app.DataByFilterOptions{Filter: filter, Limit: 100, Last: last})
		if err != nil {
			return nil, err
		}
		for _, row := range resp.TabularData {
			cf.Samples = append(cf.Samples, row.Data)
		}
		if len(resp.TabularData) == 0 || resp.Last == "" || resp.Last == last {
			break
		}
		last = resp.Last
	}
	if len(cf.Samples) == 0 {
		return nil, fmt.Errorf("sequence %s returned no samples", seq.ID)
	}
	return cf, nil
}

func readCache(path string) (*cacheFile, error) {
	b, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	var cf cacheFile
	if err := json.Unmarshal(b, &cf); err != nil {
		return nil, err
	}
	return &cf, nil
}

func writeCache(path string, cf *cacheFile) error {
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return err
	}
	b, err := json.Marshal(cf)
	if err != nil {
		return err
	}
	tmp := path + ".tmp"
	if err := os.WriteFile(tmp, b, 0o644); err != nil {
		return err
	}
	return os.Rename(tmp, path)
}
