# Model viam-labs:restock-bot:fixture-teacher

Teach arm motions relative to a fiducial marker on a fixture, store them as Viam Data sequences, and replay them on any machine with the same fixture. Configure one instance per fixture.

Every recorded pose is stored in the marker's frame. Before teaching or replaying, the service finds the marker and works out where it is relative to the arm, so a replay still lands on the fixture after the arm or the fixture has moved.

## Requirements

- An arm, and a frame to record and move (usually the gripper tip). Freedrive teaching needs an arm that supports manual mode; otherwise jog the arm by other means while recording.
- A camera in the frame system and a pose tracker that reports the marker, such as `viam:apriltag:pose_tracker` or a ChArUco tracker. A ChArUco board is one marker.
- Optionally, a gripper, to teach and replay `open` and `grab` actions.
- Optionally, an [arm-position-saver](https://app.viam.com/module/erh/vmodutils) switch that moves the camera to a pose where it sees the marker. The service calls position `2` (go to the saved pose). It never calls `1`, which would overwrite the saved pose.
- A cloud connection to teach. Replay falls back to a locally cached copy of the motion when the cloud is unreachable.

## Configuration

```json
{
  "fixture": "drawer",
  "arm": "arm",
  "tcp": "gripper-tcp",
  "gripper": "gripper",
  "motion": "builtin",
  "pose_tracker": "tag-tracker",
  "marker": 6,
  "view_switch": "find-drawer-position",
  "sample_hz": 30
}
```

| Name           | Type             | Required | Default      | Description |
|----------------|------------------|----------|--------------|-------------|
| `fixture`      | string           | no       | service name | Fixture identity. Machines with the same fixture name share motions. |
| `arm`          | string           | yes      |              | Arm (freedrive and stop) |
| `tcp`          | string           | yes      |              | Frame that is recorded and moved |
| `gripper`      | string           | no       |              | Gripper; needed only for motions with gripper actions |
| `motion`       | string           | no       | `builtin`    | Motion service |
| `pose_tracker` | string           | yes      |              | Pose tracker reporting the marker |
| `marker`       | string or number | yes      |              | Marker body; matches `<marker>` or `<prefix>-<marker>` (for example `6` matches `tag36h11-6`) |
| `view_switch`  | string           | no       |              | arm-position-saver switch that moves the camera to see the marker. If unset, the marker must already be visible. |
| `sample_hz`    | number           | no       | `30`         | Teaching sample rate, up to 1000 |
| `obstacles`    | list             | no       | `[]`         | Boxes in the **marker** frame (see below) |

Cloud access uses the API key viam-server gives modules, so no credentials are configured.

### Obstacles

```json
"obstacles": [
  {"label": "counter", "pose": {"x": 0, "y": -200, "z": -50}, "dims_mm": {"x": 800, "y": 400, "z": 40}}
]
```

`pose` uses `x, y, z, o_x, o_y, o_z, theta` (degrees). Omitted fields default to zero, and `o_z` defaults to 1. Positions and box axes follow the **marker's axes** as the pose tracker reports them, not the world's. Use the `obstacles` output of a `dry_run` to check where each box lands in world coordinates before a real run.

## Teaching a motion

1. `{"cmd": "start_teach", "motion": "open"}`. The arm moves to the view pose, locates the marker, enables freedrive (for up to 5 minutes), and starts recording.
2. Guide the arm through the motion. At each point the replay should pass through, send `{"cmd": "mark", "label": "<name>"}`.
   - Mark an approach point before contact. The planner knows nothing about the fixture except the configured obstacles.
   - Add `"linear": true` to any mark the arm must reach in a straight line, such as a pull while hooked on a handle. Unmarked moves can take any collision-free path.
   - To open or close the gripper during the demonstration, send `{"cmd": "gripper", "action": "grab"}` (or `"open"`), optionally with a `label` (default: the action). The gripper really moves, and the next sample becomes a waypoint carrying the action. A mark or gripper action sent before the previous one is captured is rejected without moving anything; retry it. In the rare case a mark lands while the gripper is moving, the action runs but is not recorded and the error says so; put the gripper back and retry.
3. `{"cmd": "end_teach"}`. This uploads the recording and creates a sequence tagged `fixture:<fixture>`, `motion:<motion>`, and `taught:freedrive` (or `taught:jog`).

Teaching the same motion again creates a newer sequence, and replay always uses the newest. Use `list_motions` to see every version and `delete` to remove one; `run` with `sequence_id` replays a specific version.

## Replaying a motion

1. `{"cmd": "run", "motion": "open", "dry_run": true}`. This locates the marker and returns each waypoint's world pose and each obstacle's world pose without following the motion. The arm still moves to the view pose.
2. `{"cmd": "run", "motion": "open"}`. This locates the marker and moves through the waypoints in order, stopping at the first failure.
   - Waypoints with a gripper action open or grab after the arm reaches them. A grab that holds nothing is reported as `holding: false` but does not stop the run. A motion with gripper actions fails before moving if no `gripper` is configured.
   - Add `"speed_degs_per_sec": 20` to limit joint speed (deg/s, like the xArm's `speed_degs_per_sec` attribute) for this run only, and optionally `"accel_degs_per_sec2"` (only with a speed). Without them the arm moves at its configured speed. With them the service plans each move with the motion service's `plan` command (the builtin motion service supports it) and sends the joint path to the arm with those limits. The arm driver may clamp or ignore them: the xArm clamps speed to 3–180 deg/s and ignores both when `traj_gen` is configured. A speed-limited run moves only the arm, and `arm` must be its frame-system name (with the `remote:` prefix for a remote arm).
3. `{"cmd": "stop"}` cancels a run and stops the arm and gripper.

Only marked samples are replayed. The full recording stays in Viam Data for later use, such as fitting dynamic motion primitives. Each sample holds `t` (seconds since `start_teach`), the pose (`x, y, z, o_x, o_y, o_z, theta`, in the marker frame), and, on marked samples, `mark`, `linear`, and `gripper`.

## DoCommand reference

| Command | Payload | Result |
|---------|---------|--------|
| `start_teach` | `{"cmd": "start_teach", "motion": "open"}` | `motion`, `taught` (`freedrive` or `jog`) |
| `mark` | `{"cmd": "mark", "label": "pulled", "linear": true}` | `ok` |
| `gripper` | `{"cmd": "gripper", "action": "grab", "label": "pick"}` | `ok`, `holding` (grab) |
| `end_teach` | `{"cmd": "end_teach"}` | `sequence_id`, `samples`, `marks` |
| `run` | `{"cmd": "run", "motion": "open", "sequence_id": "<optional>", "dry_run": true, "speed_degs_per_sec": 20}` | `sequence_id`, then `waypoints` and `obstacles` (dry run) or `steps` and `failed_step`. `sequence_id` pins a version (fetched from the cloud, not cached); `motion` is then optional, and must match the sequence if given. With a speed, the result also echoes `speed_degs_per_sec` (and `accel_degs_per_sec2`); `dry_run` ignores them. |
| `list_motions` | `{"cmd": "list_motions"}` | `motions`: per motion, newest first, `sequence_id`, `created`, `taught`, `duration_s` |
| `delete` | `{"cmd": "delete", "sequence_id": "..."}` | `sequence_id`, `motion`; only this fixture's sequences; samples stay in Viam Data |
| `stop` | `{"cmd": "stop"}` | `ok` |

Only one teaching session or run is active at a time. Results always contain `ok` and never return an RPC error; failures carry `error`. `GetStatus` reports `fixture`, `marker_known`, and `teaching`.
