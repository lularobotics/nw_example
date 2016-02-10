
// ROS
#include <ros/ros.h>

#include "simple_move_robot_client.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nw_mico_simple_move_client");

    // make sure the main thread ros sleep is woken up
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // create the client run the client
    SimpleMoveRobotClient simple_move_robot_client;
    simple_move_robot_client.Run();

    ROS_INFO("Terminating client.");

    return 0;
}
