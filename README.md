# 🤖 Autonomous Mall Security Robot

A ROS2-based autonomous security robot designed for **indoor mall
patrolling, real-time surveillance, and security threat detection**.

The system integrates: - Autonomous navigation - Computer vision based
threat detection - Obstacle avoidance - MQTT communication - ESP32
robotic control

The robot can patrol a mall autonomously or operate under manual control
while detecting incidents such as **fire, smoke, weapons, and fights**
and reporting them with robot coordinates.

------------------------------------------------------------------------

# 🚀 Features

## 🧭 Autonomous Patrol System

-   Grid-based indoor navigation
-   A\* path planning algorithm
-   8-directional movement
-   Dynamic route generation
-   Real-time position tracking
-   Obstacle-aware navigation

## 👁️ Computer Vision Security System

Built using OpenCV and AI detection models.

Detection modules: - 🔥 Fire Detection - 💨 Smoke Detection - 🔫 Weapon
Detection - ⚠️ Fight Detection - 🚧 Hurdle Detection

Each incident generates:

``` json
{
  "alert": "weapon_detected",
  "location": [4,7],
  "time": "14:30:22"
}
```

------------------------------------------------------------------------

# 🏗️ System Architecture

    Camera
     |
    Vision Package
     |
     +-- Stream Node
     +-- Fire Detection
     +-- Smoke Detection
     +-- Weapon Detection
     +-- Fight Detection
     |
    Navigation Package
     |
    A* Planner
     |
    Communicator Package
     |
    MQTT
     |
    ESP32
     |
    Robot Hardware

------------------------------------------------------------------------

# 📦 ROS2 Packages

## Vision Package

Responsible for camera streaming and AI detection.

Commands:

``` bash
ros2 run Vision Stream
ros2 run Vision Detect_Hurdle
ros2 run Vision Detect_Weapon
ros2 run Vision Detect_Fire
ros2 run Vision Detect_Fight
```

Responsibilities: - Capture frames - Run detection models - Publish
security events - Provide obstacle information

------------------------------------------------------------------------

# 🧭 Navigation Package

Run:

``` bash
ros2 run Navigation Navigate
```

Features: - A\* path planning - 8 directional movement - Position
tracking - Safe path generation

Grid example:

    0 0 0 0 0
    0 1 1 0 0
    0 0 R 0 G
    0 0 0 0 0

    0 = Free Space
    1 = Obstacle
    R = Robot
    G = Goal

------------------------------------------------------------------------

# 📡 Communicator Package

Communication:

    ROS2 ⇄ MQTT ⇄ ESP32 ⇄ Robot

## Alert Center

``` bash
ros2 run Communicator Alert_Center
```

Sends alerts:

``` json
{
 "alert":"fire_detected",
 "location":[3,5],
 "time":"15:20:10"
}
```

## Command Center

``` bash
ros2 run Communicator Command_Center
```

Sends robot commands:

``` json
{
 "command":"MOVE_FORWARD",
 "speed":50
}
```

------------------------------------------------------------------------

# 🛠️ Installation

Requirements:

-   Ubuntu 22.04
-   ROS2 Humble
-   Python 3
-   OpenCV
-   MQTT Broker
-   ESP32

Create workspace:

``` bash
mkdir -p ~/mall_robot_ws/src
cd ~/mall_robot_ws/src
```

Clone:

``` bash
git clone <repository-link>
```

Build:

``` bash
cd ~/mall_robot_ws
colcon build
source install/setup.bash
```

Install Python dependencies:

``` bash
pip install opencv-python
pip install numpy
pip install paho-mqtt
pip install ultralytics
```

MQTT:

``` bash
sudo apt install mosquitto mosquitto-clients
sudo systemctl start mosquitto
```

------------------------------------------------------------------------

# ▶️ Running

Source workspace:

``` bash
source ~/mall_robot_ws/install/setup.bash
```

Start nodes:

``` bash
ros2 run Vision Stream
ros2 run Vision Detect_Hurdle
ros2 run Vision Detect_Weapon
ros2 run Vision Detect_Fire
ros2 run Vision Detect_Fight
ros2 run Navigation Navigate
ros2 run Communicator Alert_Center
ros2 run Communicator Command_Center
```

------------------------------------------------------------------------

# 🔄 Communication Flow

    Camera
     |
    Vision Models
     |
    Alerts
     |
    Navigation
     |
    Command Center
     |
    MQTT
     |
    ESP32 Robot

------------------------------------------------------------------------

# 🛡️ Safety System

Monitors:

-   Obstacles
-   Fire
-   Smoke
-   Weapons
-   Violence

On detection:

    Detect Event
          |
    Stop Navigation
          |
    Save Coordinates
          |
    Send Alert
          |
    Notify Security

------------------------------------------------------------------------

# 🔧 Hardware

Supported:

  Component             Purpose
  --------------------- ------------------
  ESP32                 Robot controller
  Motor Driver          Motor control
  DC Motors             Movement
  Camera                Vision
  Laptop/Raspberry Pi   ROS2 computer

------------------------------------------------------------------------

# 🧰 Technologies

  Component       Technology
  --------------- --------------------
  Framework       ROS2 Humble
  Language        Python
  Vision          OpenCV + AI Models
  Navigation      A\* Algorithm
  Communication   MQTT
  Controller      ESP32

------------------------------------------------------------------------

# 📌 Future Improvements

-   SLAM integration
-   LiDAR support
-   Multi robot coordination
-   Web dashboard
-   Security database
-   Voice communication
-   Cloud monitoring

------------------------------------------------------------------------

# 📜 License

MIT License

------------------------------------------------------------------------

# ⭐ Project Goal

Develop an intelligent autonomous security robot capable of indoor
patrol, threat detection, and real-time communication with security
teams.
