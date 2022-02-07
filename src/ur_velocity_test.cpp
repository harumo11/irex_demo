#include <controller_manager_msgs/SwitchController.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>

int main(int argc, char *argv[])
{

    std::vector<std::string> stop_controllers = {
        "scalled_pos_joint_traj_controller",
        "scaled_vel_joint_traj_controller",
        "pos_joint_traj_controller",
        "vel_joint_traj_controller",
        "forward_joint_traj_controller",
        "pose_based_cartesian_traj_controller",
        "joint_based_cartesian_traj_controller",
        "forward_cartesian_traj_controller",
        "joint_group_vel_controller"};

    std::vector<std::string> start_controllers = {"twist_controller"};

    // ROS Initialization
    ros::init(argc, argv, "twist_commander_node");
    ros::NodeHandle node_handler;

    // Call service to start twist controller
    ros::ServiceClient controller_switcher =
        node_handler.serviceClient<controller_manager_msgs::SwitchController>(
            "controller_manager/switch_controller");

    // Create SwitchController service
    controller_manager_msgs::SwitchController switch_service;
    switch_service.request.strictness = switch_service.request.BEST_EFFORT;
    switch_service.request.start_controllers = start_controllers;
    switch_service.request.stop_controllers = stop_controllers;
    switch_service.request.timeout = 5.0;
    switch_service.request.start_asap = true;

    if (controller_switcher.call(switch_service))
    {
        ROS_INFO_STREAM("Result of switch controller : " << switch_service.response.ok);
        ROS_INFO_STREAM("Controller is switched to twist controller");
    }

    return 0;
}
