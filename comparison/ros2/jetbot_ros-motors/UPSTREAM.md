# Snapshot of https://github.com/dusty-nv/jetbot_ros

- Upstream commit: `d8e5ee1b17f5c66d038017b86f8920a496197ea9` (2022-04-27)
- Copied: only: jetbot_ros/motors.py, jetbot_ros/motors_waveshare.py, jetbot_ros/motors_nvidia.py, LICENSE.md, README.md
- Upstream license: MIT-style (NVIDIA). The license of this snapshot is the upstream
  license, not the Apache-2.0 license of the ros2_pca9685 package.
- Why it is here: Only the motor nodes: Twist to differential wheel speeds, written to a PCA9685-based motor HAT through Adafruit_MotorHAT.

This copy exists so the projects can be compared side by side; see
`comparison/README.md`. It is not built (`COLCON_IGNORE`) and not linted
(`AMENT_IGNORE`). Go upstream for the current version.
