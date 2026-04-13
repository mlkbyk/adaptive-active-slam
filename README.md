# 🔄 Adaptive Circular SLAM (ACS)

![ROS](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)
![Robot](https://img.shields.io/badge/Robot-AgileX_Limo-orange.svg)

This repository contains a modular, health-aware active exploration and autonomous navigation framework built on **ROS Noetic**. It enables mobile robots (specifically tested on AgileX Limo) to autonomously explore unknown environments while monitoring localization stability and safety.

---

## ✨ Key Features

* **360° Candidate Generation:** Dynamically generates navigation goals in a circular pattern around the robot.
* **SLAM Health Monitoring:** Real-time monitoring of AMCL covariance and TF stability to determine localization quality.
* **Adaptive Behavior Modes:**
    * `EXPLORE`: Aggressive exploration when SLAM health is high.
    * `RECOVER`: Conservative movement and re-localization when health drops.
    * `PAUSE`: Complete halt if SLAM stability is compromised.
* **Safety Footprint Overrides:** Custom launch parameters to adjust robot radius and inflation layers without changing global robot settings.
* **Metrics Logging:** Automatically saves session data (health, scores, modes) to CSV for post-mission analysis.

---

## 🧩 System Architecture

The package consists of several specialized nodes:

1.  **`circle_generator_node`**: Creates potential goal candidates in a circular/arc formation.
2.  **`gain_evaluator_node`**: Scores candidates based on information gain (unknown cells) and obstacle risk.
3.  **`slam_health_monitor_node`**: Publishes a normalized health metric (0.0 - 1.0) based on AMCL and TF jumps.
4.  **`decision_manager_node`**: Selects the optimal goal based on score, health, and hysteresis.
5.  **`motion_executor_node`**: Manages the `move_base` action client and prevents goal oscillations.

---

## 🛠 Installation

1. Clone the repository into your catkin workspace:
   ```bash
   cd ~/msb_agilex_ws/src
   git clone [https://github.com/YOUR_USERNAME/adaptive_circular_slam.git](https://github.com/YOUR_USERNAME/adaptive_circular_slam.git)
