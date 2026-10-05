# al-robot-driver-rs

Pepper robot driver: binds the NAOqi QI surface (AL* services, ALMemory keys and
events) to domain capabilities that flow over a pluggable transport. The driver
does not implement planners, localizers or depth math — it drives what Pepper
exposes and bridges the rest to external modules.

All QI functionality comes from the [`qi`](../../libqi-rs) crate; this crate
never touches the QI wire format.

## Layout

| Module | Role |
| --- | --- |
| `domain` | Robot-agnostic data types exchanged with the transport adapter |
| `transport` | The adapter seam: publish / subscribe / RPC / has_consumers / timer / clock |
| `qi` | Typed proxies over the AL* services and the object served back to the robot |
| `capabilities` | One module per capability (`tf`, `odom`, `laser`, ...) |
| `control` | The control RPCs (`navigation_tools`, `vision_tools`, ...) |
| `driver` | Lifecycle, capability registry, scheduler and word spotting |
| `kinematics` | Minimal URDF joint tree and forward kinematics for `tf` |
| `assets` | `share/` loading: pepper URDF and camera calibration tables |
| `shm` | One-byte POSIX shared-memory flags of the SinfonIA stack |

## Build and test

```sh
cargo build
cargo test
```

The test suite runs the whole driver against fakes: `capabilities::support`
provides a fake robot (recording QI calls) and `MemoryTransport` records the
published messages. No robot and no QI wire are needed.

## Run

```sh
cargo run -- --qi-address tcp://pepper.local:9559 --instance-prefix my-robot
```

| Flag | Meaning |
| --- | --- |
| `--qi-address` | Address of the robot's QI space |
| `--instance-prefix` | Non-empty prefix reported by `_whoWillWin` |
| `--publish-odom` | Also stream `odom -> base_link` on `tf` |
| `--assets` | Base directory holding `share/` (defaults to the binary's directory) |

At boot the driver connects to the robot, registers the `robot_toolkit` object
plus one callback object per ALMemory event, resolves the AL* services, starts
the scheduler and enables only `special_settings`. The `robot_toolkit` object
exposes `_whoWillWin`, `attach-transport` and `startPublishing`; ALMemory events
arrive as the callback methods (`touchCallback`, `wordRecognizedCallback`, ...).

## Capabilities

Capabilities are off at boot (except `special_settings`). Periodic QI work runs
only while the capability is enabled, publishing is on and the bus reports
consumers. Topic names equal capability ids.

| Id (topic) | Kind | Rate | QI surface |
| --- | --- | --- | --- |
| `tf` | periodic | 50 Hz | `ALMotion.getPosition/getAngles`, URDF chains |
| `odom` | periodic | 10 Hz | `ALMotion.getPosition/getRobotVelocity` |
| `laser` | periodic | 10 Hz | 90 laser keys in `ALMemory` |
| `depth_to_laser` | periodic | 10 Hz | `NAOqiDepth2Laser/*` keys (external module) |
| `merged_laser` | periodic | 10 Hz | physical + depth-to-laser merger |
| `front_camera`, `bottom_camera`, `depth_camera` | periodic | 10 Hz | `ALVideoDevice` |
| `front_camera_face_detector`, `bottom_camera_face_detector` | event | — | `FaceDetected` + `ALVideoDevice` |
| `mic` | event | — | `ALAudioDevice` (`processRemote`) |
| `miclocalization` | event | — | `ALSoundLocalization/SoundLocated` |
| `speech` | input | — | `ALTextToSpeech`, `ALAnimatedSpeech` |
| `cmd_vel` | input | — | `ALMotion.move` + watchdog |
| `moveto`, `free_zone` | input | — | `ALMotion.moveTo`, `ALNavigation.getFreeZone` |
| `navigation_goal`, `pose-set` | input | — | `NAOqiPlanner/Goal`, `NAOqiLocalizer/SetPose` |
| `navigation_path`, `pose-pub` | periodic | 10 Hz | `NAOqiPlanner/Path`, `NAOqiLocalizer/RobotPose` |
| `navigation_result` | event | — | `NAOqiPlanner/Result` |
| `animation`, `set_angles`, `leds` | input | — | `ALBehaviorManager`, `ALMotion.setAngles`, `ALLeds` |
| `sonar` | periodic | 50 Hz | `ALSonar` + sonar keys |
| `touch` | event | — | 12 touch events |
| `special_settings` | input | — | `ALMotion` rest/wake/safety, `ALBasicAwareness` |

## Control RPCs

`navigation_tools`, `vision_tools`, `audio_tools`, `motion_tools`, `misc_tools`
and `speech_recognition` are served on the transport RPC channel. Commands are
exclusive and always succeed on the channel; failures come back in the result
string. See `technical-requirements.md` section 6 for the command sets.

## Writing a transport adapter

Implement `transport::Transport` and map the `domain::Message` enum to your bus.
`transport::MemoryTransport` is the reference implementation used by the tests.
Adapter-facing rules the driver relies on:

- `has_consumers` gates sensor and TF work; return `false` to skip cycles.
- Message handlers may be called from any thread but must run inside a Tokio
  runtime context (input capabilities spawn their QI calls).
- `now()` stamps every message; `timer(hz)` drives the scheduler.

## Known gap

libqi-rs registers services only while starting a node, so the QI callback
objects cannot be unregistered and re-registered at runtime. `disable` paths
unsubscribe the ALMemory events (which stops all traffic), and
`Connected::unregister_callback_objects` is marked `unimplemented!` to keep the
gap explicit.
