# ROS2 PCA9685

A modular ROS 2 package that contains a node for interacting with a PCA9685 PWM board. 

This package is designed to be built in a standalone ROS 2 workspace or included as a subrepository (subrepo) in larger projects.

## Features
- Contains a ROS 2 python node.
- Easily integrated as a subrepo in a parent workspace.
- Uses standard ROS 2 tools (colcon, rosdep) for building and dependency management.

## Prerequisites

- ROS 2 (e.g., Foxy, Humble, or later) installed and sourced.
- colcon build tool.
- rosdep for dependency management.

## Installation & Build

### Standalone Installation
Clone the repository:

```bash
git clone <repo-url>
```

Create and set up your ROS 2 workspace:

```bash
mkdir -p ~/<your_workspace>/src
cd ~/<your_workspace>/src
git clone <repo-url> 
```

Install dependencies:

```bash
cd ~/<your_workspace>
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
colcon build --symlink-install
Source the setup file:
```

```bash
source ~/<your_workspace>/install/setup.bash
```

Run your node:

```bash
ros2 run pca9685 pca9685_node
```

## Including as a Subrepository (Subrepo)

To include this package in a parent project:

Add as a submodule:

```bash
cd ~/<your_workspace>/src
git submodule add <your-repo-url>  pca9685
```

Build the parent workspace:

```bash
cd ~/<your_workspace>
colcon build --symlink-install
source install/setup.bash
```

Run your node as usual with a Bus and Address parameter:

```bash
ros2 run pca9685 pca9685_node --ros-args -p bus:=1 -p address:=65
```

Send a pulse command to the board. In the example send a pulse to channel 1 with a pulse width of 300:
```bash
ros2 topic pub /pwm_command std_msgs/msg/Int32MultiArray "{data: [1, 300]}" --once
```

## Usage

After building and sourcing your workspace, run your node using:

```bash
ros2 run pca9685 pca9685_node
```

## License

This project is licensed under the Apache License 2.0.

## Maintainers

[Andrew Johnson](https://github.com/anjrew) – Maintainer – andrewmjohnson549@gmail.com

## Contributing

Contributions are welcome via PR!
