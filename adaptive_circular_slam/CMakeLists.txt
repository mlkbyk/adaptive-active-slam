cmake_minimum_required(VERSION 3.0.2)
project(adaptive_circular_slam)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  actionlib
  move_base_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp std_msgs std_srvs geometry_msgs nav_msgs visualization_msgs
    tf2 tf2_ros tf2_geometry_msgs actionlib move_base_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# --- Nodes ---
add_executable(circle_generator_node src/circle_generator.cpp)
target_link_libraries(circle_generator_node ${catkin_LIBRARIES})

add_executable(gain_evaluator_node src/gain_evaluator_node.cpp)
target_link_libraries(gain_evaluator_node ${catkin_LIBRARIES})

add_executable(slam_health_monitor_node src/slam_health_monitor_node.cpp)
target_link_libraries(slam_health_monitor_node ${catkin_LIBRARIES})

add_executable(decision_manager_node src/decision_manager_node.cpp)
target_link_libraries(decision_manager_node ${catkin_LIBRARIES})

add_executable(motion_executor_node src/motion_executor_node.cpp)
target_link_libraries(motion_executor_node ${catkin_LIBRARIES})

add_executable(candidate_visualizer_node src/candidate_visualizer_node.cpp)
target_link_libraries(candidate_visualizer_node ${catkin_LIBRARIES})

add_executable(metrics_logger_node src/metrics_logger_node.cpp)
target_link_libraries(metrics_logger_node ${catkin_LIBRARIES})

add_executable(acs_control_panel_node src/acs_control_panel_node.cpp)
target_link_libraries(acs_control_panel_node ${catkin_LIBRARIES})

# Optional: if you add the dashboard server C++ node
# add_executable(acs_dashboard_server_node src/acs_dashboard_server_node.cpp)
# target_link_libraries(acs_dashboard_server_node ${catkin_LIBRARIES})

# Optional legacy node (only if you still keep it)
# add_executable(circle_scoring_node src/circle_scoring.cpp)
# target_link_libraries(circle_scoring_node ${catkin_LIBRARIES})
