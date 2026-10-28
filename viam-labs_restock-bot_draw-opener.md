# Model viam-labs:restock-bot:draw-opener

Opens a drawer marked with an AprilTag: drives an arm-position-saver switch to a viewing pose, locates the tag via a pose tracker, approaches and hooks the drawer from the tag frame, tilts, then retracts along world -X with a linear constraint. A countertop obstacle is placed relative to the tag.

## Configuration

```json
{
  "arm": "arm",
  "motion": "builtin",
  "pose_tracker": "tag-tracker",
  "switch": "find-drawer-position",
  "tcp": "gripper-tcp",
  "tag_id": 6,
  "above_mm": 0,
  "behind_mm": 0,
  "approach_mm": 100
}
```

| Name           | Type          | Default                | Description |
|----------------|---------------|------------------------|-------------|
| `arm`          | string        | `arm`                  | Arm (declared as a dependency only) |
| `motion`       | string        | `builtin`              | Motion service |
| `pose_tracker` | string        | `tag-tracker`          | Pose tracker reporting the tag |
| `switch`       | string        | `find-drawer-position` | arm-position-saver switch for the viewing pose |
| `tcp`          | string        | `gripper-tcp`          | Frame moved by the motion service |
| `tag_id`       | int or string | `1`                    | Tag id; matches keys `6` or `<prefix>-6` |
| `above_mm`     | float         | `0`                    | TCP goal offset above the tag (tag frame) |
| `behind_mm`    | float         | `0`                    | TCP goal offset behind the tag (tag frame) |
| `approach_mm`  | float         | `100`                  | Extra standoff for the approach pose |

## DoCommand

`{"cmd": "<name>"}` where name is one of `find_drawer`, `locate_tag`, `approach`, `goal`, `tilt`, `retract`, `run_sequence`. Results always contain `ok`; failures include `error` rather than returning an RPC error.
