#include <iostream>
#include "/home/harumo/catkin_ws/src/irex_demo/src/ur_twist_manager.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_ur_handler_node");
    ros::NodeHandle nh;
    ur_twist_manager ur_twist_handler(nh);
    ur_twist_handler.enable_twist_mode();

    return 0;
}
