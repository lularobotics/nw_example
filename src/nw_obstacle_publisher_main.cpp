
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nw_obstacle_publisher");
    ros::NodeHandle nh;

    // make sure the main thread ros sleep is woken up
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std_msgs::String sphere_x_y_z_r;
    nh.getParam("/obstacle", sphere_x_y_z_r);

    ros::Publisher obstable_publisher =
        nh.advertise<std_msgs::String>("/obstable", 0);

    if (!sphere_x_y_z_r.data.empty())
    {
        ROS_INFO("Publishing obstacle '%s'.", sphere_x_y_z_r.data.c_str());
    }
    else
    {
        ROS_INFO("Publishing no obstacle.");
    }

    while (ros::ok())
    {
        obstable_publisher.publish(sphere_x_y_z_r);
        ros::Rate(100).sleep();
    }

    ROS_INFO("Publishing obstacle stopped");

    return 0;
}
