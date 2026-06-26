# Team Faabs RoboCup SSL Robot Code

This repository contains the onboard code for Team Faabs RoboCup Small Size
League robots. The active robot software currently lives in
[`tf_jetsoncode`](tf_jetsoncode/): a Rust application that runs on each
robot's NVIDIA Jetson and connects the robot to CrashPilot, onboard vision, and
the low-level motor/kicker controller.

The [`firmware`](firmware/) directory is intentionally still empty. It is kept
as the future home for next year's microcontroller firmware and will replace
the old Teensy-specific code once the new hardware platform is ready.

## What This Code Does

Each robot runs one instance of `tf_jetsoncode`. The program acts as the
real-time bridge between high-level match strategy and the robot hardware:

- receives robot commands and world state from CrashPilot over UDP
- receives onboard vision data over a Unix domain socket
- communicates with the current Teensy controller over USB HID
- runs local robot behavior such as ball handling, defense, goalie movement,
  and ORCA-based collision avoidance
- sends compact motion, dribbler, kick, and chip commands to the controller
- reports battery, current, ball sensor, kicker readiness, errors, and packet
  status back to CrashPilot

The main control loop runs at 500 Hz. Each tick reads the newest available
inputs, updates the robot command, writes the newest controller packet, and
sends a status packet back to CrashPilot.

## Repository Layout

```text
.
|-- README.md                  Project overview and setup guide
|-- LICENSE                    MIT license
|-- todo.md                    Short development TODO list
|-- firmware/                  Future firmware for next year's microchips
`-- tf_jetsoncode/             Active Jetson robot code
    |-- Cargo.toml             Rust package manifest
    |-- Cargo.lock             Locked Rust dependencies
    |-- config.toml            Runtime configuration example/current config
    |-- Cross.toml             cross-rs configuration for Jetson builds
    |-- Makefile               Build, upload, and run helper commands
    |-- deployment.md          Jetson realtime/systemd deployment notes
    `-- src/
        |-- main.rs            Application entry point
        |-- lib.rs             Robot loop and packet flow
        |-- config.rs          TOML configuration loading/defaults
        |-- communication.rs   Shared communication types and task startup
        |-- communication/     CrashPilot, vision, and HID communication
        |-- robot_logic.rs     Task dispatch from CrashPilot commands
        `-- robot_logic/       Movement, ORCA, goalie, ball, and defense logic
```

## Runtime Architecture

`tf_jetsoncode` starts three communication tasks and then enters the main robot
loop.

1. CrashPilot input is received as protobuf `CpRobot` packets over UDP.
2. Onboard vision is read from the configured Unix socket.
3. Teensy/controller data is read and written over USB HID.
4. The newest data from each source is stored in a shared event buffer.
5. The 500 Hz robot loop consumes the newest available event snapshot.
6. Robot logic converts CrashPilot state and task commands into controller
   output.
7. The controller output is published to the HID task.
8. Robot status is encoded as protobuf `RobotCp` and sent back to CrashPilot.

The controller output packet currently contains:

- command flags: error, kick, chip, dribbler, and reserved LED bits
- game/controller state
- kick and dribbler power
- target movement direction, speed, and orientation
- current robot orientation
- current robot velocity vector

The controller input packet currently contains:

- error flag
- ball sensor flag
- kick-ready and chip-ready flags
- button and DIP switch bits
- battery level
- current draw

## Hardware and Network Interfaces

The current active hardware interface is a Teensy-compatible USB HID device.
The default VID/PID are:

```toml
[teensy]
vid = 0x16C0
pid = 0x0486
```

The program also expects:

- UDP input from CrashPilot on `cp_config.port`
- UDP status output to `cp_config.host:cp_config.port_outgoing`
- a Unix domain socket for onboard vision at `onboard_vision_socket_path`

Example configuration:

```toml
robot_id = 2
onboard_vision_socket_path = "/tmp/ov_socket"

[cp_config]
host = "0.0.0.0"
port = 1024
port_outgoing = 8129

[teensy]
vid = 0x16C0
pid = 0x0486
```

If `config.toml` does not exist in the process working directory, the program
creates one with default values on startup.

## Prerequisites

For local development:

- Rust toolchain with the 2024 edition supported
- Cargo
- `pkg-config`
- Linux HID/libusb development libraries if building natively with HID support

For Jetson cross-compilation:

- Docker or another container runtime supported by `cross`
- [`cross`](https://github.com/cross-rs/cross)
- SSH access to the robots for upload/run targets

The project uses `cross` for `aarch64-unknown-linux-gnu` builds. The required
ARM packages for HID/libusb are installed by the image setup in
[`tf_jetsoncode/Cross.toml`](tf_jetsoncode/Cross.toml).

## Building

From the Jetson code directory:

```bash
cd tf_jetsoncode
```

Build for the local machine:

```bash
cargo build
```

Run the unit tests:

```bash
cargo test
```

Build for the Jetson target:

```bash
make cross-build-debug
```

Build an optimized Jetson release binary:

```bash
make cross-build-release
```

The Jetson binaries are produced under:

```text
tf_jetsoncode/target/aarch64-unknown-linux-gnu/debug/tf_jetsoncode
tf_jetsoncode/target/aarch64-unknown-linux-gnu/release/tf_jetsoncode
```

## Running Locally

For development runs on the current machine:

```bash
cd tf_jetsoncode
cargo run
```

The process will try to:

- bind the configured CrashPilot UDP input port
- connect to the configured onboard vision socket
- open the configured HID controller
- bind a UDP socket for CrashPilot status output

If any external component is missing, the corresponding task logs an error.
The robot loop itself still starts after the communication handles have been
created, but useful behavior requires real or simulated inputs.

## Robot Upload and Run Helpers

The [`tf_jetsoncode/Makefile`](tf_jetsoncode/Makefile) contains the standard
robot workflow.

List configured robot IDs and hosts:

```bash
cd tf_jetsoncode
make list-robots
```

Upload a debug build to one robot:

```bash
make upload-debug ROBOT=2
```

Upload a release build to all configured robots:

```bash
make upload-release ROBOT=all
```

Upload and run a debug build on one robot:

```bash
make run-debug ROBOT=2
```

Upload and run a release build on one robot:

```bash
make run-release ROBOT=2
```

Deploy a release build through the systemd service flow:

```bash
make run-deploy-release ROBOT=all
```

The default robot hosts are defined in the Makefile:

```text
0 -> 10.0.64.100
1 -> 10.0.64.101
2 -> 10.0.64.102
4 -> 10.0.64.104
5 -> 10.0.64.105
```

Override them on the command line when needed:

```bash
make upload-release ROBOT=2 ROBOT_2_HOST=10.0.64.202
```

## Jetson Deployment

Production robots should run the binary as a systemd service with realtime
scheduling enabled. The full setup is documented in
[`tf_jetsoncode/deployment.md`](tf_jetsoncode/deployment.md).

The deployment flow covers:

- creating a `realtime` group
- allowing realtime priority and unlimited memlock
- enabling realtime scheduling in the kernel
- installing `tf_jetsoncode.service`
- starting, stopping, and inspecting service logs with `systemctl` and
  `journalctl`

After changing realtime permissions, reboot the Jetson before relying on the
service configuration.

## Configuration Reference

`Config` is loaded from `config.toml` in the process working directory.

| Key | Meaning |
| --- | --- |
| `robot_id` | Robot ID expected in CrashPilot packets and reported back in status packets. |
| `onboard_vision_socket_path` | Unix socket path used to receive onboard vision data. |
| `cp_config.host` | CrashPilot host/IP used for outgoing robot status packets. |
| `cp_config.port` | Local UDP port used to receive CrashPilot robot packets. |
| `cp_config.port_outgoing` | Remote UDP port used for outgoing `RobotCp` status packets. |
| `teensy.vid` | USB HID vendor ID for the current controller. |
| `teensy.pid` | USB HID product ID for the current controller. |

Keep each robot's `robot_id` and network settings aligned with the robot's
CrashPilot configuration. The code asserts that incoming CrashPilot packets
match the configured robot ID once command data is present.

## Robot Logic

CrashPilot sends the current game state and a task. The Jetson code maps that
task to local behavior:

- `TaskPos`: move to a target position with an optional speed limit
- `TaskKick`: rotate to the kick orientation and kick
- `TaskChip`: rotate to the chip orientation and chip
- `TaskRecKick`: receive a moving ball and prepare a kick
- `TaskSteal`: approach and capture the ball
- `TaskDribble`: capture the ball, enable the dribbler, and move to target
- `TaskBlock`: defend a robot or defend the goal area
- `TaskPosBall`: move the ball toward a target position

Movement commands are passed through the ORCA navigation layer where applicable
so the robot can avoid obstacles and respect field constraints.

## Development Notes

- Keep generated build output out of commits. The Rust `target/` directory can
  become very large and should be treated as local build state.
- Prefer adding focused unit tests around pure robot logic. Existing tests live
  in modules such as `robot_logic/orca.rs`, `robot_logic/helpers.rs`, and
  `robot_logic/vec.rs`.
- Run `cargo fmt` before committing Rust changes.
- Run `cargo test` before deploying behavior changes to robots.
- Update this README when communication packet formats, robot hosts, deployment
  paths, or hardware assumptions change.

## Firmware Directory

`firmware/` is reserved for the next microcontroller generation. At the moment,
the active low-level controller interface is still the Teensy HID protocol used
by `tf_jetsoncode`.

When the new firmware is added, include at least:

- supported microcontroller/board names
- toolchain and flashing instructions
- wiring/interface assumptions
- packet/protocol documentation
- calibration steps
- safety notes for motors, dribbler, kicker, and chipper

## License

This project is licensed under the MIT License. See [`LICENSE`](LICENSE).
