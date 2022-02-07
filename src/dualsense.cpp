#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sstream>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

int try_open_joystick()
{
    std::string device_path_without_num = "/dev/input/js";

    int fd = open("/dev/input/js0", O_RDONLY);
    ROS_INFO_STREAM("FD : " << fd);
    if (fd > 0)
    {
        return fd;
    }
    else
    {
        return -1;
    }
}

int main(int argc, char *argv[])
{
    // Dualsense initialize
    // CAUTION!!
    // Initialize ROS node after get fd. Otherwise wrong fd is got.
    const double limit = 32767.0;
    int num_of_axis = 0;
    int num_of_button = 0;
    char name_of_joystick[80];
    std::vector<int> buttons;
    std::vector<double> axes;
    const int fd = try_open_joystick();
    if (fd < 0)
    {
        ROS_FATAL_STREAM("Can not open the joystick. Please confirm joystick "
                         "is connected to your PC");
        std::exit(-1);
    }

    ioctl(fd, JSIOCGAXES, &num_of_axis);
    ioctl(fd, JSIOCGBUTTONS, &num_of_button);
    ioctl(fd, JSIOCGNAME(80), &name_of_joystick);
    buttons.resize(num_of_button, 0);
    axes.resize(num_of_axis, 0);
    ROS_INFO_STREAM("Number of axis : " << num_of_axis);
    ROS_INFO_STREAM("Number of button : " << num_of_button);
    ROS_INFO_STREAM("Joystick name : " << name_of_joystick);
    fcntl(fd, F_SETFL, O_NONBLOCK);

    // ROS initialize
    ros::init(argc, argv, "joystick_node");
    ros::NodeHandle node_handler;
    ros::Publisher joystick_publisher =
        node_handler.advertise<sensor_msgs::Joy>("joystick", 1);
    ros::Rate timer(200);
    ROS_INFO_STREAM("Node was initialized");

    while (ros::ok())
    {
        js_event js;
        read(fd, &js, sizeof(js_event));
        switch (js.type & ~JS_EVENT_INIT)
        {
        case JS_EVENT_AXIS:
            if ((int)js.number >= axes.size())
            {
                ROS_INFO("ERROR");
                continue;
            }
            axes[(int)js.number] = js.value;
            break;

        case JS_EVENT_BUTTON:
            if ((int)js.number >= buttons.size())
            {
                ROS_INFO("ERROR");
                continue;
            }
            buttons[(int)js.number] = js.value;
            break;
        }

        // convert from std::vector to sensor_msgs::Joy
        sensor_msgs::Joy joy_msgs;
        joy_msgs.buttons = buttons;
        for (auto &&axis : axes)
        {
            joy_msgs.axes.push_back(static_cast<float>(axis) / limit);
        }

        // Append time information
        joy_msgs.header.stamp = ros::Time::now();

        // Publish
        joystick_publisher.publish(joy_msgs);
        timer.sleep();
    }

    ros::shutdown();

    return 0;
}
