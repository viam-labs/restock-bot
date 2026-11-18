package restockbot

import (
	"encoding/json"
	"errors"
	"fmt"
	"math"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/utils"
)

// speedKeys are run's optional speed arguments, echoed in the run result.
var speedKeys = []string{"speed_degs_per_sec", "accel_degs_per_sec2"}

// parseSpeed reads run's optional speed limits; nil means no limit (use motion.Move).
func parseSpeed(cmd map[string]any) (*arm.MoveOptions, error) {
	speed, hasSpeed, err := optPositive(cmd, "speed_degs_per_sec")
	if err != nil {
		return nil, err
	}
	accel, hasAccel, err := optPositive(cmd, "accel_degs_per_sec2")
	if err != nil {
		return nil, err
	}
	if hasAccel && !hasSpeed {
		return nil, errors.New("accel_degs_per_sec2 requires speed_degs_per_sec")
	}
	if !hasSpeed {
		return nil, nil
	}
	return &arm.MoveOptions{MaxVelRads: utils.DegToRad(speed), MaxAccRads: utils.DegToRad(accel)}, nil
}

func optPositive(cmd map[string]any, key string) (float64, bool, error) {
	v, present := cmd[key]
	if !present {
		return 0, false, nil
	}
	f, ok := v.(float64)
	if !ok || !(f > 0) || math.IsInf(f, 1) { // !(f > 0) also rejects NaN
		return 0, false, fmt.Errorf("%s must be a positive number", key)
	}
	return f, true, nil
}

// jointPath extracts the arm's joint positions from a motion service "plan" DoCommand response.
func jointPath(resp map[string]any, armName string) ([][]referenceframe.Input, error) {
	if _, partial := resp["plan_partialwp"]; partial {
		return nil, errors.New("motion service returned a partial plan")
	}
	raw, err := json.Marshal(resp["plan"])
	if err != nil {
		return nil, fmt.Errorf("encoding plan: %w", err)
	}
	var traj motionplan.Trajectory
	if err := json.Unmarshal(raw, &traj); err != nil {
		return nil, fmt.Errorf("decoding plan: %w", err)
	}
	noJoints := fmt.Errorf("plan has no joint positions for arm %s", armName)
	if len(traj) == 0 {
		return nil, noJoints
	}
	path := make([][]referenceframe.Input, 0, len(traj))
	for _, step := range traj {
		inputs := step[armName]
		if len(inputs) == 0 {
			return nil, noJoints
		}
		path = append(path, inputs)
	}
	return path, nil
}
