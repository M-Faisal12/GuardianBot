# 🤖 MallGuard-Robot

## Autonomous AI-Based Mall Security Patrol Robot

MallGuard-Robot is a ROS2-based autonomous security patrol robot designed for indoor mall surveillance. The system combines autonomous navigation, computer vision-based threat detection, obstacle avoidance, and real-time communication to assist security teams.

The robot can operate in **autonomous patrol mode** or **manual control mode**, continuously monitoring the environment and generating security alerts when abnormal events are detected.

---

## 🚀 Features

### Autonomous Navigation
- A* based 8-directional path planning
- Dynamic coordinate-based navigation
- Goal-oriented patrol system
- Real-time path updates

### AI Vision Security System
Computer vision modules detect:

- 🔥 Fire
- 🌫️ Smoke
- 🔫 Weapons
- ⚠️ Physical fights / conflicts
- 🚧 Obstacles and hurdles

### Robot Safety
- Real-time hurdle detection
- Local obstacle avoidance
- Navigation interruption during hazards
- Safe movement decisions

### Communication System
- MQTT-based communication
- ROS2 topic-based internal communication
- ESP32 motor controller interface
- Security alert publishing

### Operating Modes

**Autonomous Mode**
