# Snapshot of https://github.com/telemething/i2cpwm_board

- Upstream commit: `32530c9cfd97e1fa7035990b21b89a5f32f36872` (2023-12-11)
- Copied: the whole repository except .github, xmlrpcpp
- Upstream license: GPL-3.0. The license of this snapshot is the upstream
  license, not the Apache-2.0 license of the ros2_pca9685 package.
- Why it is here: Earlier ROS 2 port of i2cpwm_board. The bundled copy of the third-party xmlrpcpp library was left out.

This copy exists so the projects can be compared side by side; see
`comparison/README.md`. It is not built (`COLCON_IGNORE`) and not linted
(`AMENT_IGNORE`). Go upstream for the current version.
