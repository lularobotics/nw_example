

#include <vector>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <boost/thread/mutex.hpp>

/**
 * @brief Simulates a joint state publisher. The publisher starts broadcasting a
 *     an initial state. A newly recieved trajectory joint state becomes the new
 *     joint state that is published
 */
class EchoJointStatePublisher
{
public:
    /**
     * @brief Creates a EchoJointStatePublisher with initial joints q, the
     *     velocity qq_init is set to zero
     */
    EchoJointStatePublisher(const std::string& joint_state_topic,
                            const std::string& trajectory_joint_state_topic,
                            const sensor_msgs::JointState& joint_state_init,
                            double rate)
    {
        ros::NodeHandle nh;
        trigger_ = nh.createTimer(
            ros::Duration(1. / rate),
            boost::bind(&EchoJointStatePublisher::PublishCallback, this),
            false,
            false);
        current_state_ = joint_state_init;

        joint_state_publiser_ =
            nh.advertise<sensor_msgs::JointState>(joint_state_topic, 0);
        trajectory_joint_state_subscriber_ =
            nh.subscribe(trajectory_joint_state_topic,
                         0,
                         &EchoJointStatePublisher::TrajectoryJointStateCallback,
                         this);
    }

    void StartPublishing() { trigger_.start(); }
    void StopPublishing() { trigger_.stop(); }
    void PublishCallback()
    {
        boost::unique_lock<boost::mutex> scoped_locl(current_state_mutex_);
        joint_state_publiser_.publish(current_state_);
    }
    void TrajectoryJointStateCallback(
        const sensor_msgs::JointStateConstPtr& joint_state)
    {
        boost::unique_lock<boost::mutex> scoped_locl(current_state_mutex_);
        current_state_ = *joint_state;
        for (int i = 0; i < current_state_.position.size(); ++i)
        {
            std::cout << current_state_.position[i] << " ";
        }
        std::cout << std::endl;
    }

protected:
    boost::mutex current_state_mutex_;
    ros::Timer trigger_;
    sensor_msgs::JointState current_state_;
    ros::Publisher joint_state_publiser_;
    ros::Subscriber trajectory_joint_state_subscriber_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "echo_joint_state_publisher");
    ros::NodeHandle nh;

    // get parameters
    double rate = 100.;
    std::string joint_state_topic;
    std::string trajectory_joint_state_topic;
    std::vector<double> q_init;
    sensor_msgs::JointState joint_state_init;

    nh.getParam("/joint_state_topic", joint_state_topic);
    nh.getParam("/trajectory_joint_state_topic", trajectory_joint_state_topic);
    nh.getParam("/q_init", q_init);

    if (q_init.size() == 0)
    {
        ROS_WARN("/q_init is empty. Please specify the initial robot position");
        return 1;
    }

    for (int i = 0; i < q_init.size(); ++i)
    {
        joint_state_init.position.push_back(q_init[i]);
        joint_state_init.velocity.push_back(0.0);
    }

    EchoJointStatePublisher publisher(joint_state_topic,
                                      trajectory_joint_state_topic,
                                      joint_state_init,
                                      rate);

    publisher.StartPublishing();
    ROS_INFO("Echo joint state started. Publishing to %s at %f Hz",
             joint_state_topic.c_str(), rate);

    ros::spin();

    return 0;
}
