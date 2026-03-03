# Adaptive Active SLAM

A modular health-aware active exploration framework for mobile robots built on ROS (Noetic).

This project implements an adaptive circular candidate generation strategy combined with information gain scoring and SLAM health monitoring to autonomously select safe and informative navigation goals.

---

## 🚀 Overview

Autonomous exploration requires balancing:

- Information gain (unknown space discovery)
- Localization stability
- Navigation safety

This framework introduces a health-aware decision pipeline:

1. **Circular Candidate Generation**
   - Generates candidate goals around the robot in the map frame.
   - Radius and resolution configurable at runtime.

2. **Information Gain Evaluation**
   - Scores candidates based on nearby unknown cell density.
   - Penalizes obstacle proximity.
   - Incorporates SLAM health into total score.

3. **SLAM Health Monitoring**
   - Monitors TF stability and localization consistency.
   - Publishes a normalized health metric (0–1).

4. **Adaptive Decision Manager**
   - Selects best candidate with hysteresis.
   - Supports exploration modes:
     - `EXPLORE`
     - `RECOVER`
     - `PAUSE`
   - Prevents unstable or oscillatory goal switching.

5. **Safe Goal Execution**
   - Sends goals via `move_base`.
   - Includes timeout and minimum interval safeguards.

---

## 🧩 System Architecture
circle_generator_node
↓
gain_evaluator_node
↓
slam_health_monitor_node
↓
decision_manager_node
↓
motion_executor_node → move_base


Optional:
- `candidate_visualizer_node`
- `metrics_logger_node`
- `acs_control_panel_node`

---

## 📦 Nodes

| Node | Description |
|------|------------|
| `circle_generator_node` | Generates circular candidate goals around robot |
| `gain_evaluator_node` | Computes information gain & risk scores |
| `slam_health_monitor_node` | Publishes localization health metric |
| `decision_manager_node` | Selects optimal candidate with adaptive logic |
| `motion_executor_node` | Sends goal to move_base |
| `candidate_visualizer_node` | Publishes RViz markers |
| `metrics_logger_node` | Logs runtime metrics to CSV |
| `acs_control_panel_node` | Lightweight ROS control interface |

---

## 🛠 Requirements

- ROS Noetic
- move_base
- TF2
- nav_msgs
- visualization_msgs

---

## ▶️ Run (Real Robot)

Assumes:
- `/map` available
- TF tree available
- `move_base` running

```bash
roslaunch adaptive_circular_slam acs_real.launch

▶️ Run (Simulation)
roslaunch adaptive_circular_slam acs_sim.launch

📊 Monitoring

Useful tools:

rqt_plot /slam_health /acs/best_score
rostopic echo /acs/dashboard_text

⚙️ Design Principles

Modular architecture

Runtime tunable parameters

Safety-first goal switching

Minimal dependency coupling

Local network execution (ROS1)
