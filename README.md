# HyperHorizon Workspace

A ROS 2-based robotic command and control system that enables movement control through a structured publish-subscribe architecture.

## System Overview

HyperHorizon is a ROS 2 workspace that implements a robot control system consisting of three primary nodes:

1. **Command Publisher** - Generates random movement commands at regular intervals
2. **Command Processor** - Translates commands into velocity actions
3. **System Monitor** - A placeholder for future system monitoring capabilities

The system follows a publisher-subscriber pattern where command messages flow through the system to produce movement control outputs.

## Architecture

```
CommandPublisher Node → [commands topic] → CommandProcessor Node → [cmd_vel topic] → Robot
```

### Data Flow

- `CommandPublisher` randomly selects and publishes movement commands as `String` messages to the `commands` topic every 2 seconds
- `CommandProcessor` subscribes to the `commands` topic and translates received commands into appropriate `Twist` messages
- The `Twist` messages are published to the `cmd_vel` topic, which would typically be consumed by a robot or simulator

## Supported Commands

The system supports five basic movement commands:

| Command | Description | Velocity Parameters |
|---------|-------------|---------------------|
| MOVE_FORWARD | Move the robot forward | linear.x = 0.5, angular.z = 0.0 |
| MOVE_BACKWARD | Move the robot backward | linear.x = -0.5, angular.z = 0.0 |
| TURN_LEFT | Rotate the robot left | linear.x = 0.0, angular.z = 0.5 |
| TURN_RIGHT | Rotate the robot right | linear.x = 0.0, angular.z = -0.5 |
| STOP | Stop all movement | linear.x = 0.0, angular.z = 0.0 |

## Prerequisites

- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 Humble Hawksbill
- Python 3.10+

## Installation

### 1. Install ROS 2 Humble

Follow the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for Ubuntu 22.04.

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y python3-rosdep python3-colcon-common-extensions
```

### 2. Create a Workspace

```bash
mkdir -p ~/hyperhorizon_ws/src
cd ~/hyperhorizon_ws/src
```

### 3. Create the Package

```bash
ros2 pkg create --build-type ament_python system_engineering --dependencies rclpy std_msgs geometry_msgs
```

### 4. Add Source Files

Copy the provided node files to the appropriate locations in the package:

```bash
cp command_publisher.py ~/hyperhorizon_ws/src/system_engineering/system_engineering/
cp command_processor.py ~/hyperhorizon_ws/src/system_engineering/system_engineering/
cp system_monitor.py ~/hyperhorizon_ws/src/system_engineering/system_engineering/
```

### 5. Update Setup.py

Ensure the entry points for all three nodes are defined in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'command_publisher = system_engineering.command_publisher:main',
        'command_processor = system_engineering.command_processor:main',
        'system_monitor = system_engineering.system_monitor:main',
    ],
},
```

### 6. Build the Package

```bash
cd ~/hyperhorizon_ws
colcon build --symlink-install
```

### 7. Source the Setup Files

```bash
source ~/hyperhorizon_ws/install/setup.bash
```

For persistent sourcing, add to your .bashrc:

```bash
echo "source ~/hyperhorizon_ws/install/setup.bash" >> ~/.bashrc
```

## Running the System

Open three terminal windows:

### Terminal 1: Launch the Command Publisher

```bash
source ~/hyperhorizon_ws/install/setup.bash
ros2 run system_engineering command_publisher
```

### Terminal 2: Launch the Command Processor

```bash
source ~/hyperhorizon_ws/install/setup.bash
ros2 run system_engineering command_processor
```

### Terminal 3: Monitor ROS 2 Topics

```bash
# To view command messages
source ~/hyperhorizon_ws/install/setup.bash
ros2 topic echo /commands

# OR to view velocity control messages
source ~/hyperhorizon_ws/install/setup.bash
ros2 topic echo /cmd_vel
```

## Project Structure

```
hyperhorizon_ws/
├── build/               # Build artifacts
├── install/             # Installed executables and libraries
├── log/                 # Build and runtime logs
└── src/
    └── system_engineering/
        ├── package.xml              # Package metadata
        ├── setup.cfg                # Python package configuration
        ├── setup.py                 # Package installation setup
        ├── resource/                # Resource files
        ├── system_engineering/      # Source code
        │   ├── __init__.py
        │   ├── command_processor.py # Process commands into velocities
        │   ├── command_publisher.py # Generate random movement commands
        │   └── system_monitor.py    # System monitoring placeholder
        └── test/                    # Test files
```

## Extending the System

### Adding New Commands

To add new commands:

1. Add the command string to the `commands` list in `command_publisher.py`
2. Add corresponding processing logic in the `command_callback` method in `command_processor.py`

Example of adding a new "SPIN" command:

```python
# In command_publisher.py
self.commands = [
    "MOVE_FORWARD",
    "MOVE_BACKWARD",
    "TURN_LEFT",
    "TURN_RIGHT",
    "STOP",
    "SPIN"  # New command
]

# In command_processor.py
elif msg.data == "SPIN":
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 1.0  # Full rotational speed
```

### Implementing System Monitor

The `system_monitor.py` file is currently a placeholder. You can extend it to:

- Monitor node health
- Track command frequency
- Log system performance
- Implement safety features

## Testing

Run the tests with:

```bash
cd ~/hyperhorizon_ws
colcon test --packages-select system_engineering
```

## Troubleshooting

### Common Issues

1. **Node not found**
   - Ensure you've sourced the setup files
   - Verify the package was built successfully with `colcon build`

2. **Topic not publishing**
   - Check for Python errors in the terminal where the node is running
   - Use `ros2 topic list` to verify topic existence
   - Use `ros2 topic info /commands` to check publisher/subscriber counts

3. **Missing dependencies**
   - Install ROS 2 dependencies with `rosdep install --from-paths src --ignore-src -r -y`

## Maintenance

Maintained by:
- Name: Teja
- Email: telagathotiteja6522@gmail.com

## Disclaimer
This code was written as part of a technical interview assignment. It is not intended for reuse or distribution without explicit permission.

