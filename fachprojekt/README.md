# AuNa: Autonomous Navigation Racing Simulator

[![License](https://img.shields.io/badge/License-See%20Packages-blue.svg)](packages/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Docker](https://img.shields.io/badge/docker compose-blue)](https://docs.docker.com/compose/)

An educational ROS2-based framework for teaching autonomous navigation and robot racing. This system provides a hands-on learning environment where students can learn ROS2 fundamentals, navigation algorithms, and autonomous racing techniques using simulated robots. The system uses Docker Compose for easy deployment and provides Gazebo simulation with RViz visualization.

![Gazebo Simulation](media/gazeboSimulation.gif)

## 🚀 Key Features

- **Educational Focus**: Designed for teaching ROS2 and autonomous navigation concepts
- **Single Robot Racing**: Learn autonomous racing and navigation algorithms
- **Docker-based Deployment**: Simple containerized deployment - no manual installation required
- **Real-time Visualization**: Gazebo 3D simulation with RViz integration

## 🛠️ Installation

**Prerequisites**: Docker with docker compose support

1. **Clone the repository**:
   ```bash
   git clone https://github.com/HarunTeper/AuNa.git
   cd AuNa
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
   

## 🐛 Troubleshooting

### Common Issues

1. **Docker containers fail to start**:

   ```bash
   # Check Docker daemon
   sudo systemctl status docker
   
   # Rebuild containers
   docker compose build --no-cache
   ```

2. **Gazebo crashes or has poor performance**:

   ```bash
   # Check GPU drivers (for NVIDIA GPUs)
   nvidia-smi
   
   # Reduce graphics quality in Gazebo settings
   # Or disable GPU acceleration: LIBGL_ALWAYS_SOFTWARE=1
   ```

3. **RViz namespace not working**:
   - Ensure you enter the exact namespace format: `robot1`
   - Check that the robot containers are running: `docker compose ps`

4. **Input source changes not taking effect**:
   - Wait a few seconds after selecting the input source
   - Check the robot's status in the control panel interface

## 📁 Package Overview

The framework consists of several ROS2 packages organized by functionality:

### Core Simulation & Control

- **`auna_gazebo`** - Gazebo simulation environment and robot models
- **`auna_control`** - Input source selection and command multiplexing
- **`auna_ground_truth`** - Ground truth localization from simulation

### Localization & Transforms

- **`auna_ekf`** - Extended Kalman Filter for sensor fusion
- **`auna_tf`** - Transform frame management and broadcasting

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

This project's licensing is under development. Please refer to individual package licenses for specific components and check with the maintainers for usage permissions.

## 🙏 Acknowledgments

- **ROS2 Community**: For the robust robotics framework
- **Navigation2 Team**: For the navigation stack
- **Gazebo Team**: For the simulation environment
- **Contributors**: All researchers and developers who contributed to this project

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/HarunTeper/AuNa/issues)
- **Discussions**: [GitHub Discussions](https://github.com/HarunTeper/AuNa/discussions)
- **Email**: <harun.teper@tu-dortmund.de>

---

**Happy Autonomous Navigation!** 🚗🤖
