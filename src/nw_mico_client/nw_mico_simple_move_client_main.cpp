

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// RieMO move action
#include <riemo_move_action/PlanAction.h>
#include <riemo_move_action/BroadcastTrajectoryAction.h>
#include <riemo_move_action/QueryTrajectory.h>

/* ========================================================================== */
/* == Declarations ========================================================== */
/* ========================================================================== */

class SimpleMoveRobotClient
{
public:
    /**
     * @brief Planning action client used to send planning requests to the
     * riemo planning server
     */
    typedef actionlib::SimpleActionClient<riemo_move_action::PlanAction>
        PlanningClient;

    typedef actionlib::SimpleActionClient<
        riemo_move_action::BroadcastTrajectoryAction> BroadcastClient;

public:
    SimpleMoveRobotClient()
        : planning_client_(riemo_move_action::PlanGoal::DEFAULT_ACTION_NAME,
                           true),
          broadcasting_client_(
              riemo_move_action::BroadcastTrajectoryGoal::DEFAULT_ACTION_NAME,
              true)
    {
        ros::NodeHandle nh;
        trajectory_query_client_ = nh.serviceClient<
            riemo_move_action::QueryTrajectory>(
            riemo_move_action::QueryTrajectory::Request::DEFAULT_SERVICE_NAME);
    }

    void Run()
    {
        ros::NodeHandle nh;
        WaitForServer();

        // set obstacle if needed here otherwise may be set in a config
        // nh.setParam("/sphere_obstacle_constraint_csv", "-0.3 0.5 0");
        // nh.setParam("/sphere_obstacle_frame_id", "root");

        // create and send a planning request
        riemo_move_action::PlanGoal goal = CreatePlanningRequest();
        SendMotionPlanningRequest(goal);

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

    riemo_move_action::PlanGoal CreatePlanningRequest()
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
        nh.getParam("/passthrough_constraint_csv",
                    goal.passthrough_constraint_csv);

        bool temp;
        nh.getParam("/use_upright_orientation_constraint", temp);
        goal.use_upright_orientation_constraint = temp;
        nh.getParam("/use_upright_orientation_constraint_end_only", temp);
        goal.use_upright_orientation_constraint_end_only = temp;

        return goal;
    }

    riemo_move_action::BroadcastTrajectoryGoal CreateBroadcastRequest(
        double dilation_factor,
        double policy_execution_start_time)
    {
        riemo_move_action::BroadcastTrajectoryGoal goal;
        goal.trajectory_timing.dilation_factor = dilation_factor;
        goal.trajectory_timing.policy_execution_start_time =
            policy_execution_start_time;

        return goal;
    }

    void SendMotionPlanningRequest(const riemo_move_action::PlanGoal& goal)
    {
        ROS_INFO("Sending motion planning request ...");

        planning_client_.sendGoal(
            goal,
            boost::bind(
                &SimpleMoveRobotClient::PlanningDoneCallback, this, _1, _2),
            boost::bind(&SimpleMoveRobotClient::PlanningStartedCallback, this));
    }

    void SendTrajectoryBroadastingRequest(
        const riemo_move_action::BroadcastTrajectoryGoal& goal)
    {
        ROS_INFO("Sending trajectory broadcasting request ...");

        broadcasting_client_.sendGoal(
            goal,
            boost::bind(
                &SimpleMoveRobotClient::BroadcastDoneCallback, this, _1, _2),
            boost::bind(&SimpleMoveRobotClient::BroadcastStartedCallback,
                        this));
    }

    void WaitForServer()
    {
        ROS_INFO("Waiting for riemo planning server ...");
        planning_client_.waitForServer();
        ROS_INFO("Connected to riemo planning server.");
    }

    void PlanningStartedCallback()
    {
        ROS_INFO("Server response: planning started");
    }

    void BroadcastStartedCallback()
    {
        ROS_INFO("Server response: broadcasting trajectory ...");
    }

    void PlanningDoneCallback(
        const actionlib::SimpleClientGoalState& goal_state,
        const riemo_move_action::PlanResultConstPtr& result)
    {
        switch (goal_state.state_)
        {
            case actionlib::SimpleClientGoalState::SUCCEEDED:
            {
                ROS_INFO("Server response: planning done");
                ROS_INFO("Requesting trajectory broadcast...");
                double dilation_factor = 1.0;
                double policy_execution_start_time = 0.0;
                riemo_move_action::BroadcastTrajectoryGoal request =
                    CreateBroadcastRequest(dilation_factor,
                                           policy_execution_start_time);

                SendTrajectoryBroadastingRequest(request);
                                ros::Duration(2.0).sleep();

                                request = CreateBroadcastRequest(100., 0.);
                                SendTrajectoryBroadastingRequest(request);
                                ros::Duration(2.0).sleep();

                                request = CreateBroadcastRequest(1., 0.);
                                SendTrajectoryBroadastingRequest(request);
                                ros::Duration(2.0).sleep();

                                request = CreateBroadcastRequest(3., 0.);
                                SendTrajectoryBroadastingRequest(request);

                break;
            }
            default:
                ROS_WARN("%s", goal_state.toString().c_str());
                break;
        }
    }

    void BroadcastDoneCallback(
        const actionlib::SimpleClientGoalState& goal_state,
        const riemo_move_action::BroadcastTrajectoryResultConstPtr& result)
    {
        switch (goal_state.state_)
        {
            case actionlib::SimpleClientGoalState::SUCCEEDED:
            {
                ROS_INFO("Server response: broadcasting ended");
                ros::shutdown();
                break;
            }
            default:
                ROS_WARN("%s", goal_state.toString().c_str());
                break;
        }
    }

public:
    PlanningClient planning_client_;
    BroadcastClient broadcasting_client_;
    ros::ServiceClient trajectory_query_client_;
};

/* ========================================================================== */
/* == Entry point =========================================================== */
/* ========================================================================== */

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

/* ========================================================================== */
/* == Implementation ======================================================== */
/* ========================================================================== */
