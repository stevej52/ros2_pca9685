# Other ROS drivers for the PCA9685, compared

A survey of every ROS / ROS 2 driver or node for the PCA9685 that could be
found in September 2026, read side by side with this package. Each project
that carries a license is copied into this directory (`ros2/` and `ros1/`)
at the commit noted in its `UPSTREAM.md`, so the code can be read without
leaving this repository. Nothing in here is built or linted:
`COLCON_IGNORE` and `AMENT_IGNORE` keep colcon and the ament linters out.

Two projects are described but not copied because they carry no license at
all: [KevWal/ros2_waveshare_motor_driver](https://github.com/KevWal/ros2_waveshare_motor_driver)
and [dennn66/ros_pca9685](https://github.com/dennn66/ros_pca9685).

## Scorecard

"Config" is how a robot is described to the driver. "Units" is what a
command means. "Safety" covers command timeouts, start-up and shutdown
behaviour. Dates are the last upstream commit.

| Project | ROS | Language | License | Last commit | All 16 outputs | Config | Units | cmd_vel | Safety | Tests | Verdict |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **this package** (`ros2_pca9685` 1.0) | 2 (Humble+) | Python | Apache-2.0 | 2026-09 | yes, named | YAML, validated, live-tunable | degrees / throttle / duty, µs raw | any axis mix per channel | timeout, home, shutdown modes, chip-reset recovery | 91 (unit + node) | the general topic-based driver |
| [vertueux/i2c_pwm_board](https://github.com/vertueux/i2c_pwm_board) | 2 | C++ | GPL-3.0 | 2025-10 | yes, up to 62 boards | services at runtime | 12-bit ticks; ±1 proportional | ackerman / differential / mecanum drive modes | none | none | most complete alternative, but GPL, ticks, service-configured |
| [telemething/i2cpwm_board](https://github.com/telemething/i2cpwm_board) | 2 | C++ | GPL-3.0 | 2023-12 | yes | services | ticks; ±1 | drive modes | none | none | older port of the same code; superseded by vertueux |
| [kimsniper/ros2_pca9685](https://github.com/kimsniper/ros2_pca9685) | 2 | C++ | BSD-3 | 2026-08 | joint index = channel | hard-coded calibration | degrees (service) / radians (ros2_control) | no | none | none | clean chip driver; servo-only; calibration bug in the ros2_control path |
| [rosblox/pca9685_ros2_control](https://github.com/rosblox/pca9685_ros2_control) | 2 | C++ | Apache-2.0 | 2025-05 | joint index = channel | none (fixed 0.5–2.5 ms, 50 Hz) | ±1 velocity | via ros2_control controllers | none | none | bare ros2_control template |
| [pgaston/RC-ros2](https://github.com/pgaston/RC-ros2) (hardware interface) | 2 (Jazzy) | C++ | Apache-2.0 | 2026-09 | any channel per joint | URDF `<ros2_control>` params, unknown keys warned | radians (servo), rad/s (ESC), 0–1 (LED) | via `bicycle_steering_controller` | ESC watchdog, arming, reverse tap | gtest on the pure modules | best ros2_control design; RC-car specific |
| [tasada038/pca9685_ros2](https://github.com/tasada038/pca9685_ros2) | 2 (Humble) | C++ | MIT | 2024-06 | no interface at all | hard-coded | ticks | no | none | none | a demo, not a driver |
| [RobotX-Workshops/ros2-pca9685](https://github.com/RobotX-Workshops/ros2-pca9685) | 2 | Python | Apache-2.0 | 2025-05 | yes, one topic each | bus/address/frequency | raw ticks | no | none | template only | does not run on its own (imports a sibling project) |
| [TheNoobInventor/lidarbot](https://github.com/TheNoobInventor/lidarbot) (`lidarbot_base`) | 2 (Jazzy) | C++ | BSD-3 | 2026-09 | Waveshare HAT channels only | URDF params | rad/s with encoders | via `diff_drive_controller` | via ros2_control | none | robot-specific; the reference for the Waveshare Motor Driver HAT |
| [dusty-nv/jetbot_ros](https://github.com/dusty-nv/jetbot_ros) (motors) | 2 (Foxy) | Python | MIT | 2022-04 | motor HAT only | rclpy params | ±1 wheel speed | differential | none | none | robot-specific; shows the H-bridge pattern |
| [bradanlane/ros-i2cpwmboard](https://gitlab.com/bradanlane/ros-i2cpwmboard) (GitHub mirror) | 1 | C++ | GPL-3.0 | 2020-09 | yes, up to 62 boards | rosparam YAML or services | ticks; ±1 | drive modes | none | none | the classic ROS 1 driver |
| [dheera/ros-pwm-pca9685](https://github.com/dheera/ros-pwm-pca9685) | 1 | C++ | MIT | 2024-06 | yes, one 16-value message | per-channel arrays | 16-bit raw PWM | no | per-channel timeouts | none | simplest solid design; raw PWM only |
| [cocasema/ros-pca9685](https://github.com/cocasema/ros-pca9685) | 1 | C++ | MIT | 2017-01 | yes | params | duty cycle % or value | no | none | none | unmaintained |
| [liamondrop/ros-pca9685-board](https://github.com/liamondrop/ros-pca9685-board) | 1 | C++ | MIT | 2018-12 | two named servos | rosparam | ticks; ±1 | throttle + steering | none | none | closest ancestor of this package's use case |
| KevWal/ros2_waveshare_motor_driver | 2 | Python | none | 2023-04 | – | – | – | Twist → % | – | – | unfinished; never writes to the chip |
| dennn66/ros_pca9685 | 1 | C++ | none | 2016-08 | – | – | radians for two servo models | no | – | – | unmaintained |

## What each one does, in detail

### ROS 2

**vertueux/i2c_pwm_board** (`ros2/vertueux-i2c_pwm_board`). A faithful ROS 2
port of Bradan Lane's i2cpwm_board, kept alive for the author's SMOV
quadruped. Servos are numbered 1..992 across up to 62 boards on one bus. Topics
`servos_absolute_N` (ticks), `servos_proportional_N` (±1 around a configured
`center`/`range`/`direction`) and `servos_drive_N` (Twist, with ackerman,
differential and mecanum mixing computed from motor RPM, wheel radius and
track). Services set the frequency, configure servos, configure the drive mode
and stop everything. The bus number is a command-line argument and becomes a
topic suffix. A separate `i2c_pwm_board_calibration` node steps a servo with
the arrow keys to find its centre and end points. Things to know: the ROS 1
way of loading `servo_config` from a YAML file does not survive the port (ROS 2
parameters cannot hold nested structs and the node never declares them), so
every start-up needs service calls; values are 12-bit ticks, not
microseconds, so they change meaning when the frequency changes;
`set_pwm_frequency` sleeps for a full second; each channel write is four
separate SMBus byte writes; there is no command timeout; it depends on a
vendored `xmlrpcpp`; and the whole thing is GPL-3.0, which is fine to use but
obliges anyone who distributes a modified copy to publish their changes.

**telemething/i2cpwm_board** (`ros2/telemething-i2cpwm_board`). The same code
base ported a year earlier, with a full copy of `xmlrpcpp` inside the
repository (left out of the snapshot). Its launch file passes `servo_config:
1`, which confirms the YAML configuration path is gone in ROS 2. Nothing it
does is missing from vertueux's port.

**kimsniper/ros2_pca9685** (`ros2/kimsniper-ros2_pca9685`). A tidy C++ chip
driver (register-level, SMBus block writes, general-call reset) with two
front ends. The service `/pca9685/set_pwm` takes a channel and an angle in
degrees and maps 0–180° onto 1 ms + `min_offset_ms` … 2 ms + `max_offset_ms`.
The ros2_control plugin exports a `position` interface per joint, joint *i*
driving channel *i*, and converts radians the same way, but its offsets are
hard-coded to 0.5 and 2.5, which gives 1.5–4.5 ms pulses, far beyond any
servo; that looks like a bug. Bus, address and frequency are hard-coded in the
plugin, there are no per-joint limits, no timeout, and the HAL calls `exit(1)`
when the bus fails to open, which takes the whole controller manager down.

**rosblox/pca9685_ros2_control** (`ros2/rosblox-pca9685_ros2_control`). A
150-line ros2_control `SystemInterface`: one `velocity` command interface per
joint, command clamped to ±1 and mapped to 0.5–2.5 ms at a fixed 50 Hz, joint
*i* on channel *i*, no parameters, no README, no timeout. Fine as a template
for writing your own plugin; not a driver on its own.

**pgaston/RC-ros2** (`ros2/pgaston-rc-ros2-pca9685_hardware_interface`). Part
of a full RC-car stack on a Jetson Orin (RealSense, Isaac ROS, Nav2,
`bicycle_steering_controller`), still being worked on this month. The
hardware interface reads every setting from the URDF `<ros2_control>` block
and warns about keys it does not use, exactly the way this package warns about
unknown YAML keys. Servos take `min_angle`/`max_angle`/`offset` and
`min`/`neutral`/`max_pulse_us`; ESCs get a pure, unit-tested state machine
(`pwm_motor_controller.cpp`) with an arming sequence, a dead-band and
`forward_offset`/`reverse_offset` so the first moving command clears the ESC's
dead zone, a `max_output` cap, a watchdog, and a neutral → short tap → neutral
dance because the author's ESC only accepts reverse after that "tap". Its
issue #11 is a good read on why calibration numbers must live in exactly one
place and mapping code must be testable without hardware. Downsides: it needs
the ros2_control stack, a URDF and a controller; there is no plain topic
interface; it is tuned around one car; and the repository root has no
LICENSE file (the package declares Apache-2.0).

**tasada038/pca9685_ros2** (`ros2/tasada038-pca9685_ros2`). A C++ node whose
constructor opens two boards at 0x40 and 0x41, sweeps servo 0 through a few
angles with hard-coded tick limits, then spins with no subscriptions or
services. It documents Jetson bus quirks nicely, but it is an example, not a
driver.

**RobotX-Workshops/ros2-pca9685** (`ros2/robotx-workshops-ros2-pca9685`).
Sixteen `Int32` topics named `/pwm_channel_N` carrying raw ticks, bus and
address parameters, and (oddly) parameters for Python's garbage collector. It
uses the legacy `Adafruit_PCA9685` library and imports `py_gap_follower.gc`
from another of the author's projects, so it fails to start outside their
workspace. No calibration, limits or safety.

**TheNoobInventor/lidarbot** (`ros2/lidarbot-lidarbot_base`). A well-known
Jazzy robot on a Raspberry Pi 4. The PCA9685 is inside the Waveshare Motor
Driver HAT (PCA9685 driving a TB6612 H-bridge: one PWM channel plus two
direction channels per motor), and the hardware component wraps Waveshare's C
code plus WiringPi encoder interrupts under `diff_drive_controller`. It is the
thing to copy if you own that HAT and want proper odometry; it is not a
general PCA9685 driver.

**dusty-nv/jetbot_ros** (`ros2/jetbot_ros-motors`). Foxy-era motor nodes:
Twist → left/right wheel speeds in ±1 using `max_rpm` and wheel geometry, then
`Adafruit_MotorHAT` (PCA9685 at 0x60 plus TB6612) with per-side trim
parameters. Same H-bridge pattern as lidarbot, in Python.

### ROS 1

**bradanlane/ros-i2cpwmboard** (`ros1/bradanlane-i2cpwm_board`). The 2016
original of the i2cpwm_board family, 1600 lines of C++ in one file, with the
user documentation in `doc/`. Everything vertueux has plus YAML configuration
that works, because ROS 1 parameters can hold arrays of structs. GPL-3.0.

**dheera/ros-pwm-pca9685** (`ros1/dheera-ros-pwm-pca9685`). One `command`
topic with an `Int32MultiArray` of sixteen 16-bit values (-1 leaves a channel
alone), per-channel `timeout`, `timeout_value`, `pwm_min` and `pwm_max`
arrays, no dependencies beyond `libi2c-dev`, block writes. A negative timeout
means "value has not changed for N ms", a stuck-publisher detector. The README
documents the Adafruit Motor HAT channel map (PWM, IN1, IN2 per motor). Raw
PWM only, so servo calibration is the user's problem.

**cocasema/ros-pca9685** (`ros1/cocasema-ros-pca9685`). Services that set a
duty cycle (value or percent) on one or several pins, BeagleBone-oriented,
untouched since 2017.

**liamondrop/ros-pca9685-board** (`ros1/liamondrop-ros-pca9685-board`). A
Donkey-car node: two named servos (`throttle`, `steering`) with
`channel`/`center`/`range`/`direction` from rosparam, `servos_drive` (Twist,
±1) and `servo_absolute` for calibration, WiringPi for I2C. Its README already
notes that a car ESC "needs a reverse pulse twice before it will respond".
This is essentially what the original version of this package did, done more
cleanly, in ROS 1.

### Not copied

**KevWal/ros2_waveshare_motor_driver**: the author of issue #2 started his own
driver for the Waveshare HAT after finding this package unusable. It computes
left/right percentages from a Twist and stops there; there is no I2C code and
no license. **dennn66/ros_pca9685**: 2016, hard-codes two Futaba servo models,
license "TODO".

Seen in searches but not examined: `MibuchiYuta/ROS_ServoDriverHAT` and
`pushkalkatara/pca9685-rosjetson` (ROS 1 servo nodes), `moritzboeker/ros_raspi_car`
(ROS 1 Ackermann car), `Skammi/ROS-cpp-for-Jetbot` (ROS 1 JetBot).

## Which one for an RC car on a Raspberry Pi

For a joystick- or Nav2-driven car with an ESC and one or two steering servos,
the ranking is:

1. **This package.** It is the only ROS 2 driver that combines a validated
   configuration file, real units (degrees and microseconds), per-channel
   limits, `cmd_vel` mixing, command timeouts, a defined shutdown, chip-reset
   recovery, live tuning and tests, and it needs no compiler and no
   third-party Python libraries. Its gap is ESC-specific behaviour (see below).
2. **vertueux/i2c_pwm_board** if you prefer C++ and want several boards behind
   one node or the built-in mecanum mixing. You will configure it with service
   calls at every start-up, think in ticks, and accept GPL-3.0 and no
   timeouts.
3. **pgaston/RC-ros2's hardware interface** if the car is going to use
   ros2_control anyway (`bicycle_steering_controller`, `ackermann_steering_controller`,
   Nav2 with a proper controller chain). Expect to trim it out of its parent
   project and re-tune the ESC numbers.
4. **kimsniper** or **rosblox** as starting points only if you want to write
   your own ros2_control plugin.

Everything else is a demo, unfinished, ROS 1, or a robot-specific package.

## What is worth borrowing

In rough order of value for this package:

1. **ESC handling** (from pgaston and liamondrop). Hobby ESCs need a neutral
   signal for a second or two before they arm, ignore small throttle
   fractions (dead-band), and many only go into reverse after neutral → a
   short reverse tap → neutral. Today a `continuous` channel just maps
   throttle linearly to a pulse. Proposal: optional `deadband`,
   `forward_offset`, `reverse_offset` and `reverse_tap` settings on
   `continuous` channels, implemented as a small pure-Python state machine in
   `channels.py` with unit tests, driven from the existing watchdog timer.
2. **An H-bridge motor type** (from dheera's README, jetbot_ros and lidarbot).
   The Waveshare Motor Driver HAT, the Adafruit DC Motor HAT and the JetBot all
   drive brushed motors through the PCA9685 with one PWM channel and two
   direction channels. A `type: motor` with `in1_channel`/`in2_channel`,
   commanded as ±1, would have answered issue #2 outright and costs about
   forty lines.
3. **An interactive calibration command** (from vertueux's calibration node
   and liamondrop's `servo_absolute` workflow): `ros2 run ros2_pca9685
   calibrate steering` stepping the pulse width with the arrow keys and
   printing the YAML lines to paste. The `pulse_width` topic already does the
   work; this would only add convenience.
4. **Speed in metres per second** (from i2cpwm_board and jetbot_ros). The
   `twist` gains do this today (gain = 1 / top speed in m/s), but a
   `max_speed_mps` alias would read better in the config. Documentation
   change at most.
5. **A ros2_control hardware interface**. That is where the ROS 2 ecosystem
   is heading, and it is the only way to use the standard controllers.
   Hardware interfaces must be C++ plugins, so this would be a second,
   optional package (`ros2_pca9685_control`) reading the same channel schema
   from URDF parameters. pgaston's `pwm_motor_controller` and rosblox's
   template are the models.

Things the survey confirmed this package already does better than the field:
validated YAML with unknown-key warnings (only pgaston does this too),
microsecond and degree units instead of ticks, tolerant numeric types,
simulation mode, automatic recovery after a brown-out, `ros2 param dump` as
a complete config, and a test suite.

## Licenses of the copied code

| Directory | License |
|---|---|
| `ros2/vertueux-i2c_pwm_board`, `ros2/telemething-i2cpwm_board`, `ros1/bradanlane-i2cpwm_board` | GPL-3.0 |
| `ros2/kimsniper-ros2_pca9685`, `ros2/lidarbot-lidarbot_base` | BSD-3-Clause |
| `ros2/rosblox-pca9685_ros2_control`, `ros2/pgaston-rc-ros2-pca9685_hardware_interface`, `ros2/robotx-workshops-ros2-pca9685` | Apache-2.0 (declared in package.xml / README) |
| `ros2/tasada038-pca9685_ros2`, `ros1/dheera-ros-pwm-pca9685`, `ros1/cocasema-ros-pca9685`, `ros1/liamondrop-ros-pca9685-board` | MIT |
| `ros2/jetbot_ros-motors` | MIT-style NVIDIA license |

These snapshots are here for evaluation. Copying code out of a GPL-3.0
directory into the Apache-2.0 driver would put the driver under the GPL;
ideas are free, code is not.
