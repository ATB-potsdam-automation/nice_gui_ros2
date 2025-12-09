# Nice GUI Core Plugins

This ROS2 package provides a collection of core UI cards for building robot monitoring and control interfaces using the NiceGUI framework. Each card is designed to visualize specific types of robot data or provide control capabilities.

## Available Cards

### 1. BatteryStateCard

**Purpose**: Displays real-time battery status information from ROS2 battery state messages.

**Features**:
- Shows battery voltage, current, charge level, and status
- Visual table format with name-value pairs
- Real-time updates from battery state topic
- Status interpretation (charging, discharging, etc.)

**Configuration Parameters**:
- `topic`: Battery state topic to subscribe to

### 2. ControlCard

**Purpose**: Provides manual robot control interface with gamepad-like controls.

**Features**:
- Linear velocity control (forward/backward)
- Angular velocity control (rotation)
- Turbo mode for increased speed limits
- Visual joystick interface
- Safety controls and activation/deactivation
- Customizable speed limits

**Configuration Parameters**:
- `cmd_vel_topic`: Topic to publish velocity commands
- `max_speed.linear`: Maximum linear velocity (m/s)
- `max_speed.angular`: Maximum angular velocity (rad/s)
- `max_speed.linear_turbo`: Turbo mode linear velocity
- `max_speed.angular_turbo`: Turbo mode angular velocity

### 3. DiagnosticCard

**Purpose**: Displays ROS2 diagnostic messages for system health monitoring.

**Features**:
- Real-time diagnostic status visualization
- Filters out OK status to highlight issues
- Detailed diagnostic information display
- Color-coded status indicators
- Interactive diagnostic details dialog

**Subscribed Topics**:
- `/diagnostics`: Standard ROS2 diagnostic array topic

### 4. GPSInfoCard

**Purpose**: Displays GPS/GNSS position information from multiple GPS receivers.

**Features**:
- Multi-GPS receiver support
- Latitude, longitude, and altitude display
- GPS fix status and quality indicators
- Real-time position updates
- Configurable GPS topic sources

**Configuration Parameters**:
- `gps_topics`: Array of GPS topic names to subscribe to

### 5. GridVisualizationCard

**Purpose**: Provides 3D visualization of robot position, orientation, and path planning.

**Features**:
- 3D scene rendering of robot position
- Real-time odometry visualization
- Path planning display from multiple sources
- Interactive 3D camera controls
- Robot orientation and movement tracking

**Configuration Parameters**:
- `odom_topic`: Odometry topic for robot position
- `path_subscriber_topics`: Array of path planning topics

### 6. ImageCard

**Purpose**: Displays camera feeds and image data from ROS2 image topics.

**Features**:
- Support for both compressed and uncompressed images
- Configurable image scaling and update frequency
- Real-time image streaming
- Interactive image display with zoom/pan
- Automatic image format conversion

**Configuration Parameters**:
- `topic`: Image topic to subscribe to
- `compressed`: Boolean for compressed image format
- `scale`: Image scaling factor
- `frequency`: Update frequency (Hz)

### 7. LinkCard

**Purpose**: Provides quick navigation links to external resources and web interfaces.

**Features**:
- Configurable web links with custom names
- Button-based navigation interface
- Support for multiple links per card
- External URL opening

**Configuration Parameters**:
- `links`: Array of link configurations
- For each link:
  - `url`: Target web address
  - `name`: Display name for the link

### 8. OdometryCard

**Purpose**: Displays robot odometry data with velocity plotting and position information.

**Features**:
- Real-time velocity and position display
- Interactive velocity plots over time
- Configurable plot ranges and update rates
- Historical data visualization
- Position, orientation, and velocity metrics

**Configuration Parameters**:
- `topic`: Odometry topic to subscribe to
- `min_vel_x`/`max_vel_x`: Velocity plot range
- `plot.min_y`/`plot.max_y`: Plot Y-axis range
- `plot.update_every`: Plot update interval
- `plot.limit_number`: Maximum data points to plot

### 9. SceneCard

**Purpose**: Provides GPS-based mapping and robot location visualization using interactive maps.

**Features**:
- Interactive leaflet map integration
- Real-time robot position marker
- GPS coordinate display
- Map zoom and pan controls
- Center-on-robot functionality
- Configurable map center and options

**Configuration Parameters**:
- `gps_topic`: GPS topic to subscribe to

## Installation and Usage

### Prerequisites
- ROS2 (tested on ROS2 Humble/Iron)
- nice_gui_core package
- Python dependencies: nicegui, rclpy

### Building the Package
```bash
cd /path/to/your/ros2_ws
colcon build --packages-select nice_gui_core_plugins
source install/setup.bash
```

### Using the Cards

Cards are typically instantiated and configured through ROS2 parameters in launch files:

```python
from nice_gui_core_plugins.cards import BatteryStateCard

# Create card instance
battery_card = BatteryStateCard(node, "battery_card_params")
battery_card.create_card()
```

### Parameter Configuration

Each card expects its parameters to be namespaced under a prefix. Example parameter structure:

```yaml
battery_card_params:
  name: "Battery Status"
  topic: "/battery_state"

control_card_params:
  name: "Robot Control"
  cmd_vel_topic: "/cmd_vel"
  max_speed:
    linear: 1.0
    angular: 1.0
    linear_turbo: 2.0
    angular_turbo: 2.0
```

## Development

### Adding New Cards

1. Create a new card class inheriting from `nice_gui_core.cards.Card`
2. Implement `__init__()` method for parameter loading and subscriber setup
3. Implement `create_card()` method for UI creation
4. Add any necessary callback methods for ROS message handling
5. Update this README with card documentation

### Dependencies

- **nice_gui_core**: Base card framework and utilities
- **rclpy**: ROS2 Python client library
- **nicegui**: Web-based UI framework
- **sensor_msgs**: ROS2 sensor message types
- **nav_msgs**: ROS2 navigation message types
- **geometry_msgs**: ROS2 geometry message types
- **diagnostic_msgs**: ROS2 diagnostic message types

## Contributing

Please follow the existing code style and ensure all cards follow the established patterns for parameter handling, ROS topic subscription, and UI creation.

## License

TODO: License declaration

## Maintainer

Daniel Suhr (dsuhr@atb-potsdam.de)
