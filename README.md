# NiceGui ROS2 Concept

A modular NiceGUI application designed to interface seamlessly with ROS2. It provides a flexible way to display robot information, trigger actions, call services, and orchestrate complex behaviors through state machines — all defined through simple YAML configuration files.

## Features

### YAML-Driven UI
Define the entire application using YAML: pages, cards, actions, services, and state machines.

### Plugin-Based Architecture
Each plugin provides a set of **cards**, keeping the codebase clean, modular, and dependency-light.

### Minimal ROS Footprint
Each plugin and card subscribes, publishes, or calls services only when necessary. The application maintains a minimal number of subscribers, publishers, and timers to reduce overhead and improve performance.

### Cards
Self-contained UI components (e.g., control panels, battery indicators). Each card represents exactly one functional unit.

### Action Clients
Non-blocking, callback-based ROS2 action clients for triggering robot behaviors with minimal backend interruption.

### Service Clients
Callback-driven ROS2 service clients, following the same non-blocking approach as actions.

### State Machines
Define multi-step workflows using actions and services. Transitions are based on success/failure, and execution is fully callback-based to avoid blocking.

### Unified Logging
ROS2 and NiceGUI logs are combined, enabling clear visibility from the GUI or the ROS2 logging system. The NiceGUI log can be hidden when desired.

## Getting Started

### Running the Example with Docker Compose

To quickly get started with a working example, you can use Docker Compose to build and run the application:

1. **Build the Docker image:**
   ```bash
   docker compose build
   ```

2. **Start the application:**
   ```bash
   docker compose up
   ```

3. **Access the web interface:**
   Open your browser and navigate to `http://localhost` (or the host where you're running the container)

The example application includes:
- **Battery State Card**: Displays battery status information
- **Image Card**: Shows camera feed from `/camera/image_raw` topic
- **Scene Card**: Provides a 3D scene visualization
- **GPS Info Card**: Displays GPS information from `/fix` topic
- **Diagnostic Card**: Shows system diagnostics
- **Navigation**: Links between different pages

The application will be accessible on port 80 and includes two pages:
- Main page (`/`) with battery, camera, scene, and navigation cards
- Secondary page (`/secondary`) with GPS info, diagnostics, and navigation

**Note:** The Docker container runs in host network mode, so make sure the required ROS2 topics are available on your system for full functionality.
