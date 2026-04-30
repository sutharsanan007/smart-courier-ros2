# 🚀 SmartCourier: ROS 2 Autonomous Waypoint Navigation

An end-to-end autonomous navigation stack built in **ROS 2**, designed to guide 6-DOF robots through urban environments using global map data and local sensor fusion.

## 📖 Overview
This project bridges the gap between cloud-based logistics and local hardware execution. It allows a robot to receive a target destination anywhere in the world, fetch the optimal walking path, and physically drive itself there while reacting to real-time sensor data. 

Unlike basic navigation scripts, this system is built with physical hardware and real-world noise in mind. It uses an **Extended Kalman Filter (EKF)** to fuse high-frequency IMU data with low-frequency GPS data, ensuring smooth and continuous movement tracking even if the GPS signal drops.

## ✨ Key Features
* **🌍 Global Path Planning (OSRM):** Automatically fetches real-world geometric routing data via the OpenStreetMap API to generate accurate global waypoints.
* **🧠 6-DOF Sensor Fusion:** Integrates `robot_localization` (EKF) to fuse 9-axis IMU data (accelerometer, gyroscope, magnetometer) with standard GPS fixes.
* **🛡️ Smart Failsafes:** Features a dynamic fallback system. If the EKF loses tracking, the logic engine automatically bypasses it and falls back to raw GPS coordinates to prevent catastrophic failure.
* **☁️ Cloud Dispatching (MQTT):** Features a dual-listener architecture. The robot can receive new coordinates locally via ROS 2 topics, or remotely via a secure MQTT broker connection.
* **🔌 Hardware-Ready:** Includes a custom serial bridge to convert raw, noisy strings from physical microcontrollers into perfectly stamped ROS 2 `sensor_msgs`.

## 🛠️ Tech Stack
* **Framework:** ROS 2 (Humble/Iron)
* **Languages:** Python 3
* **Libraries:** `paho-mqtt`, `utm`, `requests`, `pyserial`
* **Core Nodes:** `robot_localization` (EKF), custom OSRM-Planner, custom Hardware Bridge
