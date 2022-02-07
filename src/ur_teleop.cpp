// This program allows user to do teloperation.
// This program has a tiny functionanality to control cartesian velocity TCP.
// This program subscribes sensor_msgs/joy and publishes geometry_msgs/twist.

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

geometry_msgs::Twist create_initialized_twist_msgs()
{
    geometry_msgs::Twist twist_msgs;
    twist_msgs.linear.x = 0.0;
    twist_msgs.linear.y = 0.0;
    twist_msgs.linear.z = 0.0;
    twist_msgs.angular.x = 0.0;
    twist_msgs.angular.y = 0.0;
    twist_msgs.angular.z = 0.0;

    return twist_msgs;
}

class JoySubscriber
{
public:
    sensor_msgs::Joy recieved_message;

    JoySubscriber()
    {
        this->clear_joy_message();
        ROS_INFO_STREAM("JoySubscriber is initialized");
    };

    void on_message_recieved(const sensor_msgs::Joy &joy_msgs)
    {
        this->recieved_message = joy_msgs;
    };

    void clear_joy_message()
    {
        this->recieved_message.axes = std::vector<float>(8, 0);
        this->recieved_message.axes.at(3) = 1; //L2 has spcific initial value as 1
        this->recieved_message.axes.at(4) = 1; //R2 has speciic initial value as 1
        this->recieved_message.buttons = std::vector<int>(14, 0);
        this->recieved_message.header.stamp = ros::Time::now();
    };
};

int main(int argc, char *argv[])
{
    const int CONTROL_HZ = 100;
    const double TCP_VEL_SCALL = 0.05;

    ros::init(argc, argv, "ur_teleop_node");
    ros::NodeHandle node_handler;
    ros::Publisher ur_tcp_vel_publisher = node_handler.advertise<geometry_msgs::Twist>("twist_controller/command", 1);
    JoySubscriber joy_listener;
    ros::Subscriber joy_subscriber = node_handler.subscribe("joy", 1, &JoySubscriber::on_message_recieved, &joy_listener);
    ros::Rate timer(CONTROL_HZ);

    while (ros::ok())
    {
        ros::spinOnce();
        auto recieved_joy_message = joy_listener.recieved_message;

        auto tcp_vel_message = create_initialized_twist_msgs();
        tcp_vel_message.linear.x = TCP_VEL_SCALL * recieved_joy_message.axes.at(1);
        tcp_vel_message.linear.y = TCP_VEL_SCALL * recieved_joy_message.axes.at(0);
        tcp_vel_message.linear.z = TCP_VEL_SCALL * recieved_joy_message.axes.at(7);

        //ROS_INFO_STREAM(joy_listener.recieved_message);
        ur_tcp_vel_publisher.publish(tcp_vel_message);

        timer.sleep();
    }

    return 0;
}
