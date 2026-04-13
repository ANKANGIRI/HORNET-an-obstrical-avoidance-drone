# HORNET-an-obstrical-avoidance-drone
This project is a real-time Autonomous Obstacle Avoidance System for drones, built with MAVSDK (Python) and a 2D RPLidar sensor. It navigates dynamically using a modified Vector Field Histogram (VFH) algorithm.
# MAVSDK LiDAR Obstacle Avoidance

An advanced, real-time obstacle avoidance script for autonomous drones using MAVSDK-Python and RPLidar. This system builds a dynamic local certainty grid and utilizes polar histograms to calculate safe flight paths, injecting on-the-fly sub-missions to steer the drone away from danger.

## 🚀 Features

* **Real-Time LiDAR Processing:** Uses multi-threading to continuously gather environmental data without blocking the drone's flight control loops.
* **Dynamic Certainty Grid:** Maintains a local map around the drone that shifts intelligently based on GPS displacement.
* **Polar Histogram Navigation:** Evaluates 360 degrees of surrounding space to find the widest, safest angle of travel.
* **Critical Proximity Protocol:** If an obstacle is detected within a critical threshold, the drone overrides standard navigation to prioritize an immediate escape vector.
* **Autonomous Altitude Adjustment:** Automatically commands an altitude increase if the horizontal environment is entirely blocked.
* **Micro-Mission Nudging:** Uses MAVSDK `MissionItem` generation to create smooth, localized nudges rather than jerky offboard velocity commands.

## 🛠️ Prerequisites

### Hardware
* A drone running PX4 Autopilot (or a simulated environment like PX4 SITL).
* An RPLidar sensor connected via USB/Serial to the companion computer.

### Software dependencies
* Python 3.7+
* `mavsdk`
* `rplidar-roboticia` (or standard `rplidar` depending on your specific hardware)
* `numpy`
* `matplotlib` (for real-time algorithm visualization)
* `pymavlink`

You can install the required Python packages using:
```bash
pip install mavsdk numpy matplotlib rplidar-roboticia pymavlink
