# ros2_pca9685

A ROS 2 driver for the **PCA9685** 16-channel, 12-bit PWM controller, the chip
on the Adafruit 16-Channel Servo HAT / Bonnet / breakout and their many clones.

Every one of the 16 outputs can be a positional servo, an ESC or continuous
rotation servo, or a plain PWM output (LEDs, motor driver inputs). All of it is
described in one YAML file; nothing about your robot is hard-coded in the node.

```yaml
/**:
  ros__parameters:
    i2c_bus: 1
    i2c_address: 64            # 0x40
    pwm_frequency: 50.0
    channels: [pan, tilt, drive]
    pan:   {channel: 0, home: 90.0, home_on_start: true}
    tilt:  {channel: 1, min_limit: 40.0, max_limit: 140.0}
    drive: {channel: 4, type: continuous, home_on_start: true, timeout: 0.5,
            twist: {linear_x: 0.5}}
```

```bash
ros2 topic pub --once /pca9685/pan/angle std_msgs/msg/Float64 "{data: 45.0}"
```

## Features

- **All 16 channels**, each with a name, a type and its own calibration.
- **Three channel types**: `servo` (degrees), `continuous` (throttle -1..1 for
  ESCs and continuous rotation servos) and `pwm` (duty cycle 0..1).
- **Per-channel topics** plus a raw pulse-width topic for calibration.
- **`cmd_vel` mixing**: any channel can follow any combination of Twist axes,
  so Ackermann cars, differential drives and mecanum bases work with
  `teleop_twist_joy`, `teleop_twist_keyboard` and Nav2.
- **JointState in and out**: drive servos from `sensor_msgs/JointState`
  (for example from `joint_state_publisher_gui` or MoveIt) and publish the
  commanded angles for `robot_state_publisher` and RViz.
- **ESC support**: a configurable arming sequence for whatever your speed
  controller expects at power-up, a dead-band with start offsets so small
  commands still move the car, and the brake-then-reverse dance that
  forward/brake/reverse ESCs need.
- **Safety**: optional home position at start-up, per-channel command
  timeouts, configurable shutdown behaviour, and automatic re-initialisation
  if the chip loses power for a moment.
- **Live tuning** of limits, calibration, home positions and PWM frequency
  with `ros2 param set`, and `ros2 param dump` to save the result.
- **No Python dependencies** beyond ROS 2: the node talks to `/dev/i2c-N`
  directly, so there is no Adafruit/Blinka stack to fight with.
- **Simulation mode** to try a configuration without hardware.

## Requirements

- ROS 2 Humble, Jazzy, Kilted or newer (Python 3.10+).
- A Linux board with an I2C bus: Raspberry Pi (enable I2C with
  `sudo raspi-config` → Interface Options), NVIDIA Jetson, etc.
- Permission to use the bus: `sudo usermod -aG i2c $USER`, then log out and
  back in. Check with `ls -l /dev/i2c-*` and `i2cdetect -y 1` (from the
  `i2c-tools` package): the board shows up as `40` (plus `70`, the chip's
  all-call address).

## Wiring

| PCA9685 pin | Connect to                                          |
|-------------|-----------------------------------------------------|
| VCC         | 3.3 V (logic supply; 5 V boards accept 3.3 V too)   |
| GND         | GND                                                 |
| SDA         | SDA (Raspberry Pi header pin 3, bus 1)              |
| SCL         | SCL (Raspberry Pi header pin 5, bus 1)              |
| V+          | Separate 5-6 V servo supply (2 A or more for servos) |

Servos and ESCs plug into the three-pin headers (signal / V+ / GND). Do not
feed V+ from the computer's 5 V rail: servos draw current spikes that will
brown out the board. The supply and the computer must share a ground.

The default address is 0x40 (64). Solder address jumpers A0-A5 to add 1, 2,
4, 8, 16 or 32 to it; up to 62 boards fit on one bus, each driven by its own
copy of this node (see [Several boards](#several-boards)).

## Install and build

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/stevej52/ros2_pca9685.git
cd ~/ros2_ws
colcon build --packages-select ros2_pca9685 --symlink-install
source install/setup.bash
```

## Run

Start with one of the example files in [`config/`](config) and edit a copy:

- [`config/example.yaml`](config/example.yaml) shows every option with its default.
- [`config/sixteen_servos.yaml`](config/sixteen_servos.yaml) is sixteen plain servos, `servo0` to `servo15`.
- [`config/rc_car.yaml`](config/rc_car.yaml) is the RC car with ESC, steering and rear steering driven from `/cmd_vel` that this package started out as.

```bash
ros2 launch ros2_pca9685 pca9685.launch.py config:=$HOME/my_robot.yaml
# or, without the launch file:
ros2 run ros2_pca9685 pca9685_node --ros-args --params-file $HOME/my_robot.yaml
```

Then move things:

```bash
ros2 topic pub --once /pca9685/pan/angle std_msgs/msg/Float64 "{data: 120.0}"
ros2 topic pub --once /pca9685/drive/throttle std_msgs/msg/Float64 "{data: 0.25}"
ros2 topic pub --once /pca9685/headlight/duty_cycle std_msgs/msg/Float64 "{data: 0.8}"
ros2 topic pub --once /pca9685/pan/pulse_width std_msgs/msg/Float64 "{data: 1500.0}"
```

The launch file also accepts `name:=` and `namespace:=`.

## Configuration

The parameter file is a normal ROS 2 parameter file. `/**:` at the top makes
it apply whatever the node is called. Write numbers with a decimal point
(`90.0`, not `90`): the node accepts both, but `ros2 param set` will only accept
decimals later on if the file used them.

### Node settings

| Parameter                  | Default    | Meaning                                                                                  |
|----------------------------|------------|------------------------------------------------------------------------------------------|
| `i2c_bus`                  | `1`        | Bus number: the node opens `/dev/i2c-<bus>`.                                             |
| `i2c_address`              | `64`       | I2C address in decimal (64 = 0x40).                                                      |
| `pwm_frequency`            | `50.0`     | PWM frequency in Hz for all outputs (24-1526). 50 Hz suits servos; analog RC ESCs are fine up to about 100 Hz. |
| `oscillator_frequency`     | `25000000` | The chip's internal clock. Trim it if a 1500 µs command measures long or short on a scope. |
| `simulate`                 | `false`    | `true` runs without hardware and logs every command instead.                             |
| `channels`                 | *required* | List of channel names. Each name gets a block of settings below.                          |
| `twist_topic`              | `cmd_vel`  | `geometry_msgs/Twist` topic followed by channels that have `twist` gains.                |
| `joint_state_topic`        | `''`       | If set, `sensor_msgs/JointState` messages on this topic drive servo channels (radians).  |
| `joint_state_publish_rate` | `0.0`      | If above 0, the commanded servo angles are published on `joint_states` at this rate (Hz). |

### Channel settings

Each name listed in `channels` has its own block. Defaults reproduce the
Adafruit ServoKit behaviour (750-2250 µs over 0-180°), so numbers calibrated
with that library carry over.

| Setting             | Default          | Meaning                                                                                        |
|---------------------|------------------|------------------------------------------------------------------------------------------------|
| `channel`           | *required*       | PCA9685 output number, 0 to 15.                                                                |
| `type`              | `servo`          | `servo`, `continuous` or `pwm` (see below).                                                    |
| `min_pulse_us`      | `750.0`          | Pulse width at `min_angle` (servo) or full reverse (continuous).                               |
| `max_pulse_us`      | `2250.0`         | Pulse width at `max_angle` (servo) or full forward (continuous).                               |
| `neutral_pulse_us`  | midway           | Continuous only: pulse width for throttle 0.                                                   |
| `min_angle`         | `0.0`            | Servo only: the angle that produces `min_pulse_us`. Use `-90.0`/`90.0` for a zero-centred convention. |
| `max_angle`         | `180.0`          | Servo only: the angle that produces `max_pulse_us`.                                            |
| `min_limit`         | full range       | Lowest command allowed; anything below is clamped. In degrees, throttle or duty as per `type`. |
| `max_limit`         | full range       | Highest command allowed.                                                                       |
| `invert`            | `false`          | Mirror the direction of travel (servo/continuous) or drive an active-low load (pwm).           |
| `home`              | middle of limits (servo), `0.0` (others) | Value written at start-up (if `home_on_start`), when a timeout expires and, optionally, at shutdown. |
| `home_on_start`     | `false`          | Move to `home` as soon as the node starts. Otherwise the output stays off (servo limp) until the first command. |
| `timeout`           | `0.0`            | Seconds without a command before the channel returns to `home`. `0` disables the timeout.      |
| `on_shutdown`       | `off`            | What happens when the node exits: `off` (no more pulses), `home`, or `hold` (keep the last value). Write `"off"` with quotes: YAML reads a bare `off` as false (the node treats that as `off` too). |
| `joint`             | channel name     | Name used in `JointState` messages.                                                            |
| `twist.linear_x` … `twist.angular_z` | `0.0` | Gains for following the Twist topic (see below).                                     |
| `esc.arming.values` / `esc.arming.durations` | none | Continuous only: throttle steps and their lengths in seconds sent at start-up before commands are accepted (see [ESCs](#escs)). |
| `esc.to_reverse.values` / `esc.to_reverse.durations` | none | Continuous only: steps sent before the first reverse command after driving forward. |
| `esc.to_forward.values` / `esc.to_forward.durations` | none | Continuous only: steps sent before the first forward command after reversing. |
| `esc.deadband`      | `0.0`            | Continuous only: commands this close to zero are sent as neutral.                              |
| `esc.forward_start` / `esc.reverse_start` | `0.0` | Continuous only: the output at which the motor actually starts to move; commands are spread between it and the limit. |

Channel types and their command units:

| `type`       | Command                              | Topic         | Typical use                                   |
|--------------|--------------------------------------|---------------|-----------------------------------------------|
| `servo`      | angle in degrees                     | `angle`       | Positional hobby servos                       |
| `continuous` | throttle from -1.0 to 1.0, 0 = stop  | `throttle`    | ESCs, continuous rotation servos, RC motor controllers |
| `pwm`        | duty cycle from 0.0 to 1.0           | `duty_cycle`  | LEDs, brushed motor driver PWM inputs         |

Calibration and limits are separate on purpose: `min_pulse_us`/`max_pulse_us`
and `min_angle`/`max_angle` describe the servo, `min_limit`/`max_limit`
describe how far your mechanism may travel.

### Following `cmd_vel`

A channel with one or more `twist` gains is driven by the Twist topic:

```
command = home + linear_x * gain_linear_x + angular_z * gain_angular_z + …
```

clamped to the channel's limits. Examples:

```yaml
    # Ackermann car: throttle from linear.x, steering from angular.z.
    throttle: {channel: 0, type: continuous, home_on_start: true, timeout: 0.5,
               twist: {linear_x: 0.5}}                 # throttle per m/s
    steering: {channel: 1, home: 90.0, min_limit: 45.0, max_limit: 135.0,
               twist: {angular_z: -30.0}}              # degrees per rad/s

    # Differential drive with two ESCs, 0.3 m between the wheels.
    left:  {channel: 2, type: continuous, timeout: 0.5,
            twist: {linear_x: 1.0, angular_z: -0.15}}
    right: {channel: 3, type: continuous, timeout: 0.5, invert: true,
            twist: {linear_x: 1.0, angular_z: 0.15}}
```

With a `timeout` on every driven channel the robot stops by itself when the
teleop or navigation node goes away.

## ESCs

A hobby electronic speed controller is not a servo. Three things about it can
be described on a `continuous` channel under `esc:`. Leave the block out for a
continuous-rotation servo or a motor driver that behaves like one.

**Arming.** Every ESC waits for a particular signal after power-up before it
will drive the motor, and they do not agree on what that signal is. Describe
yours as a list of steps, each a throttle value and a number of seconds. The
node sends them when it starts, before it accepts commands for the channel,
and logs `arming the ESC` and `ESC armed`:

```yaml
    drive:
      type: continuous
      esc:
        arming:                      # most car ESCs: neutral for a couple of seconds
          values: [0.0]
          durations: [2.0]
```

Other procedures you may meet:

```yaml
        arming:                      # minimum throttle first, then neutral
          values: [-1.0, 0.0]        # (many brushless ESCs, especially non-car ones)
          durations: [2.0, 0.5]

        arming:                      # neutral, a small nudge, neutral
          values: [0.0, 0.05, 0.0]   # (an ESC that ignores a plain neutral signal)
          durations: [2.5, 0.5, 0.5]
```

Commands that arrive during the sequence are kept, and the latest one is
applied when it ends. The sequence runs again by itself after the chip loses
power, and on request, for example when the ESC was switched on after the
node:

```bash
ros2 service call /pca9685/drive/arm std_srvs/srv/Trigger
```

**Dead-band and start offsets.** ESCs ignore throttle close to neutral, and a
car usually needs a fair fraction of throttle before it moves at all. With
`deadband`, commands within that distance of zero are sent as neutral, and
with `forward_start`/`reverse_start` the remaining commands are spread
between the point where the motor starts to move and the channel limit, so
that a small command from Nav2 creeps instead of doing nothing and the limits
stay the most the ESC is ever sent:

```yaml
        deadband: 0.03               # 0.03 and below: neutral
        forward_start: 0.12          # 0.031 -> 0.12 output, max_limit -> max_limit
        reverse_start: 0.08
```

Find the numbers with the `pulse_width` topic: the first pulse that moves the
car, as a fraction of the distance from neutral to the end pulse.

**Reverse on forward/brake/reverse ESCs.** Many car ESCs treat the first
reverse command after driving forward as a brake, and only reverse after the
throttle has returned to neutral and been pulled back again. Describe that
first pull as `to_reverse` steps and the node performs them before every
reverse that follows a forward run. `to_forward` does the same in the other
direction for ESCs that need a pause before going forward again:

```yaml
        to_reverse:                  # brake tap, neutral, then the reverse command
          values: [-0.3, 0.0]
          durations: [0.15, 0.15]
        to_forward:                  # neutral for a moment, then forward
          values: [0.0]
          durations: [0.2]
```

Step values are raw throttle fractions, clamped to the channel limits; the
dead-band and start offsets only shape commands, not sequence steps. A raw
`pulse_width` command cancels any running sequence.

## Topics

All command topics live under the node name (default `/pca9685`) and use
`std_msgs/msg/Float64`. Subscriptions are best-effort so they match any
publisher.

| Topic                          | Direction | Meaning                                                                        |
|--------------------------------|-----------|--------------------------------------------------------------------------------|
| `~/<name>/angle`               | in        | Servo angle in degrees (`type: servo`).                                        |
| `~/<name>/throttle`            | in        | Throttle -1.0 … 1.0 (`type: continuous`).                                      |
| `~/<name>/duty_cycle`          | in        | Duty cycle 0.0 … 1.0 (`type: pwm`).                                            |
| `~/<name>/pulse_width`         | in        | Raw pulse width in microseconds, bypassing calibration and limits. `0` switches the output off. Handy for finding a servo's real end points. |
| `<twist_topic>` (`cmd_vel`)    | in        | `geometry_msgs/msg/Twist`, only subscribed when some channel has `twist` gains. |
| `<joint_state_topic>`          | in        | `sensor_msgs/msg/JointState`; `position` in radians drives servo channels whose `joint` matches `name`. |
| `joint_states`                 | out       | `sensor_msgs/msg/JointState` with the commanded servo angles, when `joint_state_publish_rate` is set. |
| `~/<name>/arm`                 | service   | `std_srvs/srv/Trigger`, continuous channels: run the ESC arming sequence again.  |

Commands are clamped to the limits, written to the chip immediately, and
logged at debug level (`--ros-args --log-level debug`).

## Driving servos from JointState

Set `joint_state_topic` and give each servo the joint name from your URDF.
Use a zero-centred calibration so 0 rad is the middle of the travel:

```yaml
    joint_state_topic: joint_states
    joint_state_publish_rate: 0.0     # leave off: the GUI publishes the states
    shoulder: {channel: 0, min_angle: -90.0, max_angle: 90.0, joint: shoulder_joint}
    elbow:    {channel: 1, min_angle: -90.0, max_angle: 90.0, joint: elbow_joint}
```

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui --ros-args -p robot_description:="$(cat arm.urdf)"
```

Every slider now moves a servo. For the opposite direction, set
`joint_state_publish_rate` and leave `joint_state_topic` empty: the node
publishes what it commanded and `robot_state_publisher` shows the arm in RViz.

## Live tuning

Everything except the channel list, output numbers, types, joint names and the
bus settings can be changed while the node runs. Changes are validated first,
then applied to the outputs straight away:

```bash
ros2 param set /pca9685 steering.max_limit 140.0
ros2 param set /pca9685 steering.home 88.0
ros2 param set /pca9685 esc.neutral_pulse_us 1520.0
ros2 param set /pca9685 pwm_frequency 100.0
ros2 param dump /pca9685 > tuned.yaml      # save the result (edit the node name into '/**' if you like)
```

The node fills in every derived default at start-up, so the dump is a complete
configuration file.

## Safety behaviour

- Outputs stay off until they are commanded, unless `home_on_start` is set.
  Off means no pulses: a servo goes limp, an ESC sees no signal.
- `timeout` returns a channel to `home` when commands stop arriving. Give every
  drive channel one.
- On exit (Ctrl-C, `ros2 launch` shutdown, SIGTERM) each channel follows its
  `on_shutdown` setting; the default switches the output off.
- The node checks every two seconds that the chip still holds its
  configuration and re-programs it (and every active output) after a power
  glitch.
- I2C errors are logged and the node keeps running, so a loose wire does not
  take the rest of the system down.

## Several boards

Run one node per board, each with its own parameter file and node name:

```bash
ros2 launch ros2_pca9685 pca9685.launch.py config:=arm.yaml name:=arm_pwm
ros2 launch ros2_pca9685 pca9685.launch.py config:=base.yaml name:=base_pwm
```

Topics are private to the node (`/arm_pwm/shoulder/angle`,
`/base_pwm/left/throttle`), so they never collide.

## Running without hardware

Set `simulate: true` (or pass `-p simulate:=true`) and the node logs every
write instead of touching the bus. The example files can be tried this way on
any machine:

```bash
ros2 run ros2_pca9685 pca9685_node --ros-args --params-file config/rc_car.yaml -p simulate:=true
ros2 topic pub --once /pca9685/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 1.0}}"
```

## Troubleshooting

- **`can't copy 'resource/ros2_pca9685'` while building** – fixed: the marker
  file is now part of the repository.
- **`Package 'ros2_pca9685' not found` after building** – source the
  workspace: `source ~/ros2_ws/install/setup.bash`.
- **`Permission denied: /dev/i2c-1`** – add yourself to the `i2c` group (see
  Requirements) or run the node with `sudo -E`.
- **`/dev/i2c-1 does not exist`** – enable I2C in `raspi-config`, or pick the
  right `i2c_bus`. On a Jetson Nano the 40-pin header carries bus 1 on pins
  3/5 and bus 0 on pins 27/28; other Jetsons number them differently, so
  check `ls /dev/i2c-*` and `i2cdetect -y <bus>`.
- **`No response from the PCA9685`** – wrong bus, wrong address, missing
  ground, or a board without power on VCC. `i2cdetect -y <bus>` must show `40`.
- **Servo buzzes or hits an end stop** – lower `max_limit` / raise `min_limit`,
  or calibrate `min_pulse_us`/`max_pulse_us` with the `pulse_width` topic.
- **ESC does not arm** – describe its power-up procedure with `esc.arming`
  (see [ESCs](#escs)); neutral for two seconds is the usual one. Switch the
  ESC on before the node, or call the `~/<name>/arm` service afterwards.
- **Car only brakes when reversing** – add `esc.to_reverse` steps.
- **Small speeds do nothing** – set `esc.forward_start`/`esc.reverse_start`.
- **`setup.py install is deprecated` warnings from colcon** – harmless, they
  come from setuptools and affect every Python ROS package.

## Migrating from the old version

The original node was the ROS 2 tutorial listener with the RC car's servo
numbers and limits typed into the source. Everything it did is in
`config/rc_car.yaml`; the differences are:

- Executables `listener` and `talker` are gone. Run `pca9685_node` with a
  parameter file instead.
- The package now sits at the root of the repository (previously it was in a
  sub-directory). Cloning into `src/` works as before.
- Pulse widths follow the datasheet formula. The Adafruit library was off by
  about 1.5 % at 100 Hz, so a servo may sit a degree or two from where it used
  to. Adjust `home` and the limits if needed.
- The rear steering servo used to be initialised to 90 but mirrored around 85
  once driving; it now uses one `home` value.
- The throttle in `config/rc_car.yaml` is a `continuous` channel with an ESC
  arming sequence instead of a servo commanded in "degrees"; the limits give
  the same pulses as before.
- Commands are logged at debug level instead of printing every message.

## License

Apache License 2.0, see [LICENSE](LICENSE).
