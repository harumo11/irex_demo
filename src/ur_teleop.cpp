// This program allows user to do teloperation.
// This program has a tiny functionanality to control cartesian velocity TCP.
// This program subscribes sensor_msgs/joy and publishes geometry_msgs/twist.

#include "/home/harumo/catkin_ws/src/irex_demo/src/ur_twist_manager.hpp"
#include "/home/harumo/catkin_ws/src/irex_demo/src/preshape.hpp"
#include <algorithm>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <vector>

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

class limiter
{
private:
    const double max_limit;
    const double min_limit;

public:
    limiter(const double limit) : max_limit(limit), min_limit(-1 * limit)
    {
    }

    double limit(const double input)
    {
        double limited_output;
        limited_output = std::min(this->max_limit, input);
        limited_output = std::max(this->min_limit, limited_output);
        return limited_output;
    }
};

int main(int argc, char *argv[])
{
    const int CONTROL_HZ = 100;
    const double TCP_VEL_SCALL = 0.15;
    const double VEL_LIMIT = 0.1; //Limit max velocity between -0.5 and 0.5 ms
    limiter vel_limiter(VEL_LIMIT);

    ros::init(argc, argv, "ur_teleop_node");
    ros::NodeHandle node_handler;
    ros::Publisher ur_tcp_vel_publisher = node_handler.advertise<geometry_msgs::Twist>("twist_controller/command", 1);
    JoySubscriber joy_listener;
    ros::Subscriber joy_subscriber = node_handler.subscribe("joy", 1, &JoySubscriber::on_message_recieved, &joy_listener);
    ros::Rate timer(CONTROL_HZ);
    ur_twist_manager ur_manager(node_handler);
    ur_manager.enable_twist_mode();
    preshape preshape_x(false);
    preshape preshape_y(false);
    preshape preshape_z(false);

    while (ros::ok())
    {
        auto recieved_joy_message = joy_listener.recieved_message;

        auto tcp_vel_message = create_initialized_twist_msgs();
        if (recieved_joy_message.axes.at(4) < 0)
        {
            tcp_vel_message.linear.x = preshape_x.step(TCP_VEL_SCALL * (recieved_joy_message.axes.at(1)));
            tcp_vel_message.linear.y = preshape_y.step(TCP_VEL_SCALL * (recieved_joy_message.axes.at(0)));
            tcp_vel_message.linear.z = preshape_z.step(TCP_VEL_SCALL * (recieved_joy_message.axes.at(6)));
        }

        tcp_vel_message.linear.x = TCP_VEL_SCALL * (recieved_joy_message.axes.at(1));
        tcp_vel_message.linear.y = TCP_VEL_SCALL * (recieved_joy_message.axes.at(0));
        tcp_vel_message.linear.z = TCP_VEL_SCALL * (recieved_joy_message.axes.at(6));

        //ROS_INFO_STREAM(joy_listener.recieved_message);
        ur_tcp_vel_publisher.publish(tcp_vel_message);

        ros::spinOnce();
        timer.sleep();
    }

    return 0;
}
