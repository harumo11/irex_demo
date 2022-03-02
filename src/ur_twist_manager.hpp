#pragma once

#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <functional>

class ur_twist_manager
{

public:
    ur_twist_manager(ros::NodeHandle nh);
    ~ur_twist_manager();
    void enable_twist_mode();
    void disalbe_twist_mode();
    bool load_twist_controller();
    bool switch_to_twist_controller();
    bool switch_to_default_controller();
    bool unload_twist_controller();

private:
    ros::NodeHandle node_handler;
    std::vector<std::string> stop_controllers_for_twist_controller = {
        "scaled_pos_joint_traj_controller",
        //"scaled_vel_joint_traj_controller",
        //"pos_joint_traj_controller",
        //"vel_joint_traj_controller",
        //"forward_joint_traj_controller",
        //"pose_based_cartesian_traj_controller",
        //"joint_based_cartesian_traj_controller",
        //"forward_cartesian_traj_controller",
        //"joint_group_vel_controller"
    };

    std::vector<std::string> start_controllers_for_twist_controller = {"twist_controller"};
    std::vector<std::string> start_controllers_for_default_controller = {"scaled_pos_joint_traj_controller"};
    std::vector<std::string> stop_controllers_for_defalut_controller = {"twist_controller"};
};

ur_twist_manager::ur_twist_manager(ros::NodeHandle nh) : node_handler(nh)
{
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("Use ros::init() ,before use ur_vel_handler");
        std::exit(1);
    }
}

ur_twist_manager::~ur_twist_manager()
{
    std::cout << "destructor" << std::endl;
    this->disalbe_twist_mode();
}

void ur_twist_manager::enable_twist_mode()
{
    this->load_twist_controller();
    this->switch_to_twist_controller();
}

void ur_twist_manager::disalbe_twist_mode()
{
    this->switch_to_default_controller();
    this->unload_twist_controller();
}

bool ur_twist_manager::load_twist_controller()
{

    ros::ServiceClient controller_loader =
        node_handler.serviceClient<controller_manager_msgs::LoadController>(
            "/controller_manager/load_controller");

    controller_manager_msgs::LoadController load_service;
    load_service.request.name = "twist_controller";

    bool result_of_load_controller = true;
    if (controller_loader.call(load_service))
    {
        if (load_service.response.ok == 1)
        {
            ROS_INFO_STREAM("Twist controller is loaded");
        }
        else
        {
            ROS_WARN_STREAM("Twist controller is NOT loaded. That controller may have been loaded already.");
            result_of_load_controller = false;
        }
    }
    else
    {
        ROS_WARN_STREAM("Can't load twist controller."
                        << "Run first roslaunch irex_demo enable_robot.launch");
        result_of_load_controller = false;
    }

    return result_of_load_controller;
}

bool ur_twist_manager::switch_to_twist_controller()
{
    ros::ServiceClient controller_switcher =
        node_handler.serviceClient<controller_manager_msgs::SwitchController>(
            "/controller_manager/switch_controller");

    controller_manager_msgs::SwitchController switch_service;
    switch_service.request.strictness = switch_service.request.STRICT;
    switch_service.request.start_controllers = this->start_controllers_for_twist_controller;
    switch_service.request.stop_controllers = this->stop_controllers_for_twist_controller;
    switch_service.request.timeout = 5.0;
    switch_service.request.start_asap = true;

    bool result_of_switch_controller = true;
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
            result_of_switch_controller = false;
        }
    }
    else
    {
        ROS_WARN_STREAM("Can't switch twist controller"
                        << " Run first roslaunch irex_demo enable_robot.launch");
        result_of_switch_controller = false;
    }

    return result_of_switch_controller;
}

bool ur_twist_manager::switch_to_default_controller()
{
    ros::ServiceClient controller_switcher =
        node_handler.serviceClient<controller_manager_msgs::SwitchController>(
            "/controller_manager/switch_controller");

    controller_manager_msgs::SwitchController switch_service;
    switch_service.request.strictness = switch_service.request.STRICT;
    switch_service.request.start_controllers = this->start_controllers_for_default_controller;
    switch_service.request.stop_controllers = this->stop_controllers_for_defalut_controller;
    switch_service.request.timeout = 5.0;
    switch_service.request.start_asap = true;

    bool result_of_switch_controller = true;
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
            result_of_switch_controller = false;
        }
    }
    else
    {
        ROS_WARN_STREAM("Can't switch the default controller"
                        << "Run first roslaunch irex_demo enable_robot.launch");
        result_of_switch_controller = false;
    }

    return result_of_switch_controller;
}

bool ur_twist_manager::unload_twist_controller()
{
    ros::ServiceClient controller_unloader =
        node_handler.serviceClient<controller_manager_msgs::UnloadController>(
            "/controller_manager/unload_controller");

    controller_manager_msgs::UnloadController unload_service;
    unload_service.request.name = "twist_controller";

    bool result_of_unload_controller = true;
    if (controller_unloader.call(unload_service))
    {
        if (unload_service.response.ok == 1)
        {
            ROS_INFO_STREAM("Twist controller is unloaded");
        }
        else
        {
            ROS_WARN_STREAM("Twist controller is NOT unloaded. That controller may have been loaded already.");
            result_of_unload_controller = false;
        }
    }
    else
    {
        ROS_WARN_STREAM("Can't unload twist controller."
                        << "Run first roslaunch irex_demo enable_robot.launch");
        result_of_unload_controller = false;
    }

    return result_of_unload_controller;
}