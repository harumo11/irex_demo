#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>

int main(int argc, char *argv[])
{

    std::vector<std::string> stop_controllers = {
        "scaled_pos_joint_traj_controller",
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

    // Call service to load twist controller
    ros::ServiceClient controller_loader =
        node_handler.serviceClient<controller_manager_msgs::LoadController>(
            "/controller_manager/load_controller");

    // Create LoadController service
    controller_manager_msgs::LoadController load_service;
    load_service.request.name = "twist_controller";

    // Call service to start twist controller
    ros::ServiceClient controller_switcher =
        node_handler.serviceClient<controller_manager_msgs::SwitchController>(
            "/controller_manager/switch_controller");

    // Create SwitchController service
    controller_manager_msgs::SwitchController switch_service;
    switch_service.request.strictness = switch_service.request.STRICT;
    switch_service.request.start_controllers = start_controllers;
    switch_service.request.stop_controllers = stop_controllers;
    switch_service.request.timeout = 5.0;
    switch_service.request.start_asap = true;

    if (controller_loader.call(load_service))
    {
        if (load_service.response.ok == 1)
        {
            ROS_INFO_STREAM("Twist controller is loaded");
        }
        else
        {
            ROS_WARN_STREAM("Twist controller is NOT loaded. That controller may have been loaded already.");
        }
    }
    else
    {
        ROS_WARN_STREAM("Can't load twist controller."
                        << "Run first roslaunch irex_demo enable_robot.launch");
    }

    if (controller_switcher.call(switch_service))
    {
        if (switch_service.response.ok == 1)
        {
            ROS_INFO_STREAM("Twist controller is switched");
        }
        else
        {
            ROS_WARN_STREAM("Twist controller is NOT switched."
                            << " Twist controller is loaded?"
                            << " Please check running controllers are shown with rosservice call /controller_manager/list_controllers");
        }
    }
    else
    {
        ROS_WARN_STREAM("Can't switch twist controller"
                        << " Run first roslaunch irex_demo enable_robot.launch");
    }

    // switch back to default controller
    stop_controllers = {"twist_controller"};

    start_controllers = {"scaled_pos_joint_traj_controller"};

    switch_service.request.start_controllers = start_controllers;
    switch_service.request.stop_controllers = stop_controllers;

    if (controller_switcher.call(switch_service))
    {
        if (switch_service.response.ok == 1)
        {
            ROS_INFO_STREAM("The default controller is switched");
        }
        else
        {
            ROS_WARN_STREAM("The default controller(scaled_pos_joint_traj_controller) can NOT be switched."
                            << " That controller may have been already running.");
        }
    }
    else
    {
        ROS_WARN_STREAM("Can't switch the default controller"
                        << "Run first roslaunch irex_demo enable_robot.launch");
    }

    // create message for unload twist controller
    // Call service to unload twist controller
    ros::ServiceClient controller_unloader =
        node_handler.serviceClient<controller_manager_msgs::UnloadController>(
            "/controller_manager/unload_controller");

    // Create UnloadController service
    controller_manager_msgs::UnloadController unload_service;
    unload_service.request.name = "twist_controller";

    if (controller_unloader.call(unload_service))
    {
        if (unload_service.response.ok == 1)
        {
            ROS_INFO_STREAM("Twist controller is unloaded");
        }
        else
        {
            ROS_WARN_STREAM("Twist controller is NOT unloaded. That controller may have been loaded already.");
        }
    }
    else
    {
        ROS_WARN_STREAM("Can't unload twist controller."
                        << "Run first roslaunch irex_demo enable_robot.launch");
    }

    return 0;
}
