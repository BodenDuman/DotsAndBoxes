cmake_minimum_required(VERSION 3.0.2)
project(DotsAndBoxes)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/vision_node.py
  scripts/game_node.py
  scripts/move_robot.py
  scripts/camera_calibration.py
  scripts/homography_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
