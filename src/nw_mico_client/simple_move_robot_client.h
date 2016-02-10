// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// RieMO move action
#include <riemo_move_action/PlanAction.h>
// RieMO move service
#include <riemo_move_action/BroadcastTrajectory.h>
#include <riemo_move_action/QueryTrajectory.h>

class SimpleMoveRobotClient
{
public:
    /**
     * @brief Planning action client used to send planning requests to the
     * riemo planning server
     */
    typedef actionlib::SimpleActionClient<riemo_move_action::PlanAction>
        PlanningClient;

public:
    /**
     * @brief Creates a SimpleMoveRobotClient
     */
    SimpleMoveRobotClient();

    /**
     * @brief Runs the client which starts action requests to planning server to
     *     plan a trajectory and request joint trajectory broadcast
     */
    void Run();

    /**
     * @brief Creates planning request from ros parameter server
     */
    riemo_move_action::PlanGoal CreatePlanningRequest();

    /**
     * @brief Creates a requests for trajectory broadcast with a specific time
     *     dilation factor and a policy execution start time
     * @param dilation_factor
     *     Time dilation factor. The value 1.0 requests the origianl trajectory
     *     values higher than 1.0 slows down the trajectory and values small
     *     than 1.0 speeds up the motion.
     * @param policy_execution_start_time
     * @return
     */
    riemo_move_action::BroadcastTrajectory::Request CreateBroadcastRequest(
        double dilation_factor,
        double policy_execution_start_time);

    /**
     * @brief Sends the specified planning request which contains the target
     *     position and motion constraint settings
     * @param goal
     */
    void SendMotionPlanningRequest(const riemo_move_action::PlanGoal& goal);

    /**
     * @brief Requests a trajectory broadcast with a specific time dilation
     *     factor and a policy execution start time
     */
    riemo_move_action::BroadcastTrajectory::Response
    SendTrajectoryBroadastingRequest(
        const riemo_move_action::BroadcastTrajectory::Request& request);

    /**
     * @brief Callback function which is called when the planning server has
     *     started the planning process
     */
    void PlanningStartedCallback();

    /**
     * @brief Callback function which is called when the planning server has
     *     completed or aborted the planning request
     */
    void PlanningDoneCallback(
        const actionlib::SimpleClientGoalState& goal_state,
        const riemo_move_action::PlanResultConstPtr& result);

protected:
    /**
     * @brief Waits till conntected to the planning server
     */
    void WaitForServer();

protected:
    PlanningClient planning_client_;
    ros::ServiceClient trajectory_query_client_;
    ros::ServiceClient broadcast_trajectory_client_;
};
