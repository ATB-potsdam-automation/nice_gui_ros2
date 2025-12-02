# NiceGui ROS2 Concept

A modular NiceGUI application designed to interface seamlessly with ROS2. It provides a flexible way to display robot information, trigger actions, call services, and orchestrate complex behaviors through state machines — all defined through simple YAML configuration files.

## Features

### YAML-Driven UI
Define the entire application using YAML: pages, cards, actions, services, and state machines.

### Plugin-Based Architecture
Each plugin provides a set of **cards**, keeping the codebase clean, modular, and dependency-light.

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
