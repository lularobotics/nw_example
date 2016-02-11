
#include "simple_move_robot_client.h"

SimpleMoveRobotClient::SimpleMoveRobotClient()
    : planning_client_(riemo_move_action::PlanGoal::DEFAULT_ACTION_NAME, true)
{
    ros::NodeHandle nh;
    trajectory_query_client_ =
        nh.serviceClient<riemo_move_action::QueryTrajectory>(
            riemo_move_action::QueryTrajectory::Request::DEFAULT_SERVICE_NAME);
    broadcast_trajectory_client_ = nh.serviceClient<
        riemo_move_action::BroadcastTrajectory>(
        riemo_move_action::BroadcastTrajectory::Request::DEFAULT_SERVICE_NAME);
}

void SimpleMoveRobotClient::Run()
{
    ros::NodeHandle nh;
    WaitForServer();

    // set obstacle if needed here otherwise may be set in a config
    // nh.setParam("/sphere_obstacle_constraint_csv", "-0.3 0.5 0");
    // nh.setParam("/sphere_obstacle_frame_id", "root");

    // create and send a planning request
    riemo_move_action::PlanGoal request = CreatePlanningRequest();
    SendMotionPlanningRequest(request);

    while (ros::ok())
    {
        if (!planning_client_.isServerConnected())
        {
            ROS_WARN("Disconnedted from planning server");
            WaitForServer();
        }

        ros::Rate(100).sleep();
    }
}

riemo_move_action::PlanGoal SimpleMoveRobotClient::CreatePlanningRequest()
{
    riemo_move_action::PlanGoal goal;
    ros::NodeHandle nh;

    // read from config

    // set target
    nh.getParam("/x_target_x", goal.target.x);
    nh.getParam("/x_target_y", goal.target.y);
    nh.getParam("/x_target_z", goal.target.z);

    // set constraints
    nh.getParam("/approach_constraint_csv", goal.approach_constraint_csv);
    nh.getParam("/obstacle_linearization_constraint_csv",
                goal.obstacle_linearization_constraint_csv);
    nh.getParam("/passthrough_constraint_csv", goal.passthrough_constraint_csv);

    bool temp;
    nh.getParam("/use_upright_orientation_constraint", temp);
    goal.use_upright_orientation_constraint = temp;
    nh.getParam("/use_upright_orientation_constraint_end_only", temp);
    goal.use_upright_orientation_constraint_end_only = temp;

    return goal;
}

riemo_move_action::BroadcastTrajectory::Request
SimpleMoveRobotClient::CreateBroadcastRequest(
    double dilation_factor,
    double policy_execution_start_time)
{
    riemo_move_action::BroadcastTrajectory::Request goal;
    goal.trajectory_timing.dilation_factor = dilation_factor;
    goal.trajectory_timing.policy_execution_start_time =
        policy_execution_start_time;

    return goal;
}

void SimpleMoveRobotClient::SendMotionPlanningRequest(
    const riemo_move_action::PlanGoal& goal)
{
    ROS_INFO("Sending motion planning request ...");

    planning_client_.sendGoal(
        goal,
        boost::bind(&SimpleMoveRobotClient::PlanningDoneCallback, this, _1, _2),
        boost::bind(&SimpleMoveRobotClient::PlanningStartedCallback, this));
}

riemo_move_action::BroadcastTrajectory::Response
SimpleMoveRobotClient::SendTrajectoryBroadastingRequest(
    const riemo_move_action::BroadcastTrajectory::Request& request)
{
    ROS_INFO("Sending trajectory broadcasting request ...");

    riemo_move_action::BroadcastTrajectory srv;
    srv.request = request;

    if (!broadcast_trajectory_client_.call(srv))
    {
        ROS_ERROR("Failed to send broadcast trajectory request");
    }

    return srv.response;
}

void SimpleMoveRobotClient::WaitForServer()
{
    ROS_INFO("Waiting for riemo planning server ...");
    planning_client_.waitForServer();
    ROS_INFO("Connected to riemo planning server.");
}

void SimpleMoveRobotClient::PlanningStartedCallback()
{
    ROS_INFO("Server response: planning started");
}

void SimpleMoveRobotClient::PlanningDoneCallback(
    const actionlib::SimpleClientGoalState& goal_state,
    const riemo_move_action::PlanResultConstPtr& result)
{
    switch (goal_state.state_)
    {
        case actionlib::SimpleClientGoalState::SUCCEEDED:
        {
            ros::Time start_time = ros::Time::now();

            ROS_INFO("Server response: planning done");
            ROS_INFO("Requesting trajectory broadcast...");

            double dilation_factor = 1.0;
            double policy_execution_start_time = 0.0;

            riemo_move_action::BroadcastTrajectory::Request request;
            request = CreateBroadcastRequest(dilation_factor,
                                             policy_execution_start_time);

            ROS_INFO("move");
            SendTrajectoryBroadastingRequest(request);
//            ros::Duration(2.0).sleep();

//            ROS_INFO("slow down");
//            request = CreateBroadcastRequest(
//                4., ros::Time::now().toSec() -
//                start_time.toSec());
//            SendTrajectoryBroadastingRequest(request);
//            ros::Duration(2.0).sleep();

//            ROS_INFO("speed up");
//            request = CreateBroadcastRequest(
//                1., ros::Time::now().toSec() -
//                start_time.toSec());
//            SendTrajectoryBroadastingRequest(request);
//            ros::Duration(2.0).sleep();

//            ROS_INFO("slow down again");
//            request = CreateBroadcastRequest(
//                3., ros::Time::now().toSec() -
//                start_time.toSec());
//            SendTrajectoryBroadastingRequest(request);
            ros::shutdown();

            break;
        }
        default:
            ROS_WARN("%s", goal_state.toString().c_str());
            break;
    }
}
