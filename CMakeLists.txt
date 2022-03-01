cmake_minimum_required(VERSION 3.12)
project(irex_demo)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  control_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  tf2
  trajectory_msgs
  geometry_msgs
)

# for ur_pose_selector
#find_package(ImGui REQUIRED)

catkin_package(
	CATKIN_DEPENDS
	control_msgs
	geometry_msgs
	moveit_core
	moveit_msgs
	moveit_ros_planning_interface
	roscpp
	tf2
	trajectory_msgs
	)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(ur_pose_selector src/ur_pose_selector.cpp)
add_executable(single_glass_placer src/single_glass_placer.cpp)
add_executable(dualsense_node src/dualsense.cpp)
add_executable(test_open_dualsense src/test_open_dualsense.cpp)
add_executable(ur_velocity_test src/ur_velocity_test.cpp)
add_executable(ur_teleop_test src/ur_teleop.cpp)
add_executable(test_ur_vel_handler src/test_ur_vel_handler.cpp)

target_link_libraries(single_glass_placer ${catkin_LIBRARIES})
target_link_libraries(dualsense_node ${catkin_LIBRARIES})
target_link_libraries(test_open_dualsense ${catkin_LIBRARIES})
#target_link_libraries(ur_pose_selector ${IMGUI_LIBRARIES})
target_link_libraries(ur_velocity_test ${catkin_LIBRARIES})
target_link_libraries(ur_teleop_test ${catkin_LIBRARIES})
target_link_libraries(test_ur_vel_handler ${catkin_LIBRARIES})
