# Snapshot of https://github.com/pgaston/RC-ros2

- Upstream commit: `79d5e7b86611536b7d1c81ebad835834213c33b2` (2026-09-14)
- Copied: only: src/RCCar/pca9685_hardware_interface, src/RCCar/rc_hardware_control/description/description.urdf.xacro
- Upstream license: Apache-2.0 (declared in package.xml, no LICENSE file upstream). The license of this snapshot is the upstream
  license, not the Apache-2.0 license of the ros2_pca9685 package.
- Why it is here: The PCA9685 ros2_control hardware interface from an RC-car project, with its ESC state machine and tests, plus the URDF that configures it.

This copy exists so the projects can be compared side by side; see
`comparison/README.md`. It is not built (`COLCON_IGNORE`) and not linted
(`AMENT_IGNORE`). Go upstream for the current version.
