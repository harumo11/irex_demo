// グラスを持ち上げて，違う場所へ置くだけのデモ用プログラム

#include <iostream>
#include <cmath>
#include <fstream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// Calculates trajectory plan to given target pose and moves the robot.
// If calculates plan successfuly, return true. Otherwise false.
bool move_robot(moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::Pose &target_pose)
{
    // Set target pose
    move_group.setPoseTarget(target_pose);

    // Planning
    moveit::planning_interface::MoveGroupInterface::Plan target_plan;
    if (move_group.plan(target_plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO_STREAM("Planning is success!");
        move_group.move();
        return true;
    }
    else
    {
        ROS_WARN_STREAM("Failed to calculate a plan");
        return false;
    }
}

int main(int argc, char *argv[])
{

    // ROS initialize
    ROS_INFO_STREAM("Single glass placer node starts");
    ROS_INFO_STREAM("ROS Initialize starts");
    ros::init(argc, argv, "single_glass_placer_node");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_INFO_STREAM("MoveIT Initialize starts");
    const std::string planning_group = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(planning_group);

    //auto current_rpy = move_group.getCurrentRPY();
    //ROS_INFO_STREAM("Current robot RPY : " << current_rpy.at(0) << " , " << current_rpy.at(1) << " , " << current_rpy.at(2));
    //if (std::abs(int(current_rpy.at(0))) != 3)
    //{
    //    ROS_FATAL_STREAM("Before program starts, align robot flange with z-axis");
    //    std::exit(-1);
    //}

    // Set path constraint
    //moveit_msgs::OrientationConstraint ocm;
    //ocm.link_name = "tool0";
    //ocm.header.frame_id = "base_link";
    //ocm.orientation.x = 0.7;
    //ocm.orientation.y = -0.7;
    //ocm.absolute_x_axis_tolerance = 0.2;
    //ocm.absolute_y_axis_tolerance = 0.2;
    //ocm.weight = 1.0;
    //moveit_msgs::Constraints path_constraints;
    //path_constraints.orientation_constraints.push_back(ocm);
    //move_group.setPathConstraints(path_constraints);

    //move_group.setPlannerId("RRTstar");
    move_group.setPlannerId("FMT");
    ROS_INFO_STREAM("Robot try to pick and place a glass. Check the safty of the surroundings");
    ROS_WARN_STREAM("Robot goes to pre-pick place");
    geometry_msgs::Pose pre_pick_pose;
    pre_pick_pose.position.x = 0.03;
    pre_pick_pose.position.y = -0.8;
    pre_pick_pose.position.z = 0.2;
    pre_pick_pose.orientation.x = 0.5882529395484373;
    pre_pick_pose.orientation.y = -0.808676541868581;
    pre_pick_pose.orientation.z = -0.0006571113984997596;
    pre_pick_pose.orientation.w = 0.0005458467787818878;
    move_robot(move_group, pre_pick_pose);

    ROS_WARN_STREAM("Robot goes to pick place");
    geometry_msgs::Pose pick_pose = pre_pick_pose;
    pick_pose.position.y -= 0.05;
    move_robot(move_group, pick_pose);

    ROS_WARN_STREAM("Robot goes to post-pick place");
    geometry_msgs::Pose post_pick_pose = pick_pose;
    post_pick_pose.position.z += 0.05;
    move_robot(move_group, post_pick_pose);

    ROS_WARN_STREAM("Robot goes to pre-place place");
    geometry_msgs::Pose pre_place_pose = pre_pick_pose;
    pre_place_pose.position.x = 0.8;
    pre_place_pose.position.y = 0.1;
    pre_place_pose.position.z = 0.2;
    move_robot(move_group, pre_place_pose);

    ROS_WARN_STREAM("Robot goes to place place");
    geometry_msgs::Pose place_pose = pre_place_pose;
    place_pose.position.z -= 0.05;
    move_robot(move_group, place_pose);

    ROS_WARN_STREAM("Robot goes to post-place");
    geometry_msgs::Pose post_place_pose = place_pose;
    post_place_pose.position.x -= 0.05;
    move_robot(move_group, post_place_pose);

    return 0;
}
