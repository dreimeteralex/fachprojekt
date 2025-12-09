# AuNa: Autonomous Navigation Racing Simulator

[![License](https://img.shields.io/badge/License-See%20Packages-blue.svg)](packages/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Docker](https://img.shields.io/badge/docker compose-blue)](https://docs.docker.com/compose/)

This repository contains a student project based on the **AuNa – Autonomous Navigation Racing Simulator**.  
The framework is ROS2-based and provides a hands-on environment for learning autonomous navigation, control algorithms and robot racing using simulated robots. Deployment is handled via Docker Compose, with Gazebo for 3D simulation and RViz for visualization.

![Gazebo Simulation](media/gazeboSimulation.gif)

## 📌 Project status

This fork is used as a teaching and project environment.  
So far, the following functionalities have been implemented:

- **Automatic Emergency Braking (AEB)**  
  A safety node that monitors laser scan data and triggers braking when obstacles are detected within a critical distance.

- **Wallfollowing**  
  A control node that keeps the robot at a desired distance to a wall using laser scan data (e.g. for corridor / racetrack navigation).

**Current work (in progress):**

- Integration of **slam_toolbox** for online mapping  
- Setup of a **mapping pipeline** (map creation, saving, and usage for navigation)

## 🧩 Implemented components (project-relevant)
### Automatic Emergency Braking (AEB)

- **Package:** `auna_aeb`  
- Monitors laser scan data for obstacles in front of the vehicle  
- Publishes safe velocity commands if an obstacle is detected within a safety margin  
- Can be combined with other control nodes (e.g. wallfollowing) as an additional safety layer  

### Wallfollowing

- **Package:** `fp_wallfollowing`  
- Uses laser scan data to follow a wall at a desired distance  
- Suitable as a base behavior for navigation on tracks or corridors  
- Implemented as a ROS2 node, subscribes to sensor topics and publishes velocity commands  

### Work in Progress: SLAM & Mapping

- **Package:** `fp_slam` (and related configuration in `auna_common`)  
- Integration of **slam_toolbox** for:
  - Online map creation while driving through the environment  
  - Storing maps for later navigation runs  
- **Goal:** Combine SLAM-based maps with navigation and control to enable more robust autonomous driving.  

## 🚀 Key Features

- **Educational Focus**  
  Designed for teaching ROS2, control and navigation in a simulated racing scenario.
- **Single Robot Racing**  
  Autonomous and semi-autonomous racing on predefined tracks.
- **Docker-based Deployment**  
  Reproducible environment using Docker Compose – no complex local ROS installation required.
- **Real-time Visualization**  
  Gazebo for simulation, RViz for visualization and debugging of topics, frames and navigation behavior.
- **Extendable Architecture**  
  ROS2 packages for control, simulation, mapping and teleoperation can be extended with own algorithms (e.g. AEB, wallfollowing, SLAM).

## 🛠️ Installation

**Prerequisites**: Docker with docker compose support

1. **Clone the repository**:
   ```bash
   git clone https://github.com/dreimeteralex/fachprojekt.git
   cd fachprojekt
   ```

## 🚀 Usage

### Running Simulations

The only supported method is through Docker Compose. You only need to specify the world name:

```bash
# Default world (racetrack_decorated)
docker compose --profile racing up

# Specify custom world
WORLD_NAME=arena docker compose --profile racing up
```

### Available Worlds

- `racetrack_decorated` - Racing track environment (default)

After launching, **Gazebo** and **RViz** windows will start automatically.

## 🎮 RViz Configuration

Once the simulation starts, you'll see both **Gazebo** (simulation) and **RViz** (visualization) windows.

### Setting Up Robot Namespace in RViz

1. **Enter the namespace** in RViz:
   - Format: `robot1` (currently supports single robot)

2. **Select input source** after entering the namespace:
   - **Default**: `off` (no autonomous control)
   - **Available sources**:
     - `teleop` - Manual keyboard control  

### Control Mode Selection

- Use `teleop` for manual control practice

### Example Usage Flow

1. Start simulation:

   ```bash
   docker compose --profile racing up
   ```

2. In RViz window:
   - Enter namespace: `robot1`
   - Select input source: `teleop` (for manual racing)

## 📁 Package Overview

The framework consists of several ROS2 packages organized by functionality:

### Core Simulation & Control

- **`auna_gazebo`** - Gazebo simulation environment and robot models
- **`auna_control`** - Input source selection and command multiplexing
- **`auna_ground_truth`** - Ground truth localization from simulation

### Localization & Transforms

- **`auna_ekf`** - Extended Kalman Filter for sensor fusion
- **`auna_tf`** - Transform frame management and broadcasting
- **`fp_slam`** - SLAM integration and mapping experiments (in progress)

### Safety and Behaviour
 - **`fp_wallfollowing`** - Transform frame management and broadcasting
- **`auna_aeb`** - SLAM integration and mapping experiments (in progress)

### Messaging

- **`auna_msgs`** - Custom ROS2 message and service definitions

### User Interface & Control

- **`auna_teleoperation`** - Manual keyboard control

### Utilities & Templates

- **`auna_common`** - Shared utilities and helper functions
- **`auna_template`** - Template package for new development

## 🤝 Contributing

### Development with GitHub Copilot

This repository includes comprehensive GitHub Copilot instructions to help you develop more effectively. See [`.github/copilot-instructions.md`](.github/copilot-instructions.md) for:

- Project-specific coding patterns and conventions
- ROS2 development guidelines
- Multi-robot system considerations
- Package-specific development tips

### General Contributing Guidelines

1. **Fork the repository**
2. **Create a feature branch**:

   ```bash
   git checkout -b feature/my-new-feature
   ```

3. **Make changes and commit**:

   ```bash
   git commit -am 'Add some feature'
   ```

4. **Push to the branch**:

   ```bash
   git push origin feature/my-new-feature
   ```

5. **Create a Pull Request**

## 📄 License

Licensing is package-specific.
Please refer to the license files inside each package and, if in doubt, clarify usage with the original project maintainers and your supervisor.

## 🙏 Acknowledgments

- **ROS2 Community**: For the robust robotics framework
- **Navigation2 Team**: For the navigation stack
- **Gazebo Team**: For the simulation environment
- **Contributors**: All researchers and developers who contributed to this project

**Happy Autonomous Navigation!** 🚗🤖
