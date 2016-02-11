// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// RieMO move action
#include <riemo_move_action/PlanAction.h>

// RieMO move service
#include <riemo_move_action/BroadcastTrajectory.h>
#include <riemo_move_action/QueryTrajectory.h>

// Uncomment to play around with modulating the speed.
//#define MODULATE_SPEED

typedef actionlib::SimpleActionClient<riemo_move_action::PlanAction>
    PlanningClient;

// Setup and a plan "goal". "Goal" is somewhat of a misnomer, but you can think
// of it as the "goal" os optimization, i.e. the constraints and objective. In
// this case, it includes a target, end-effector constraints, and behavioral
// constraints.
//
// This method pulls the parameters from the ROS parameter server. Make sure
// those values are set property.
riemo_move_action::PlanGoal CreatePlanningRequest() {
  riemo_move_action::PlanGoal goal;
  ros::NodeHandle nh;

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

// Setup a BroadcastTrajectory request
riemo_move_action::BroadcastTrajectory::Request CreateBroadcastRequest(
    double time_from_start, double dilation_factor) {
  riemo_move_action::BroadcastTrajectory::Request goal;
  goal.trajectory_timing.dilation_factor = dilation_factor;
  goal.trajectory_timing.time_from_start = time_from_start;

  return goal;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "nw_mico_simple_move_client");

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup a client to query a trajectory once planning is complete and publish
  // it to the /joint_trajectory topic immediately on the server side.
  // Alternatively, one can use the riemo_move_action::QueryTrajectory service
  // with the same calling API to receive the trajectory in the service response
  // rather than performing a server-side publish.
  auto broadcast_trajectory_client = nh.serviceClient<
      riemo_move_action::BroadcastTrajectory>(
      riemo_move_action::BroadcastTrajectory::Request::DEFAULT_SERVICE_NAME);

  // Setup the planning client that will perform the planning request.
  PlanningClient planning_client(
      riemo_move_action::PlanGoal::DEFAULT_ACTION_NAME, true);
  planning_client.waitForServer();  // Wait until the connection is ready.

  // Send the planning request and wait for planning to complete. Alternatively,
  // rather than waiting here, one can specify a "planning done" callback that
  // will be called once planning has completed. In that case, planning is
  // performed asynchronously, and execution continues immediately after the
  // request is sent.
  ROS_INFO("Sending motion planning request ...");
  auto final_state = planning_client.sendGoalAndWait(CreatePlanningRequest());
  if (final_state.state_ != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("Planning failed.");
    return 1;
  }

  // Request a trajectory from the trajectory query service. In this case we're
  // using the BroadcastTrajectory service which specified that the trajectory
  // be broadcast immediately on the serverside. Alternatively, one can use the
  // QueryTrajectory service, which will specify that the trajectory be returned
  // in the response to the caller.
  //
  // Dilation factors > 1 slow it down and factors < 1 speed it up. The
  // remainder of this code modulates the speed if a MODULATE_SPEED is defined
  // at compile time.
  double time_from_start = 0.0;
  double dilation_factor = 1.0;
  riemo_move_action::BroadcastTrajectory srv;
  ROS_INFO("Requesting a basic trajectory to be calculated and broadcast...");
  srv.request = CreateBroadcastRequest(time_from_start, dilation_factor);
  if (!broadcast_trajectory_client.call(srv)) {
    ROS_ERROR("Failed to send broadcast trajectory request");
  }

#ifdef MODULATE_SPEED
  // After 2 seconds, slow the execution down by a factor of 2.
  double sleep_time = 1.;
  ros::Duration(sleep_time).sleep();
  time_from_start += sleep_time;
  dilation_factor = 2.0;
  ROS_INFO("Slowing down...");
  srv.request = CreateBroadcastRequest(time_from_start, dilation_factor);
  if (!broadcast_trajectory_client.call(srv)) {
    ROS_ERROR("Failed to send broadcast trajectory request");
  }

  // After 3 seconds, speed it up beyond the initial speed
  sleep_time = 2.;
  ros::Duration(sleep_time).sleep();
  time_from_start += sleep_time;
  dilation_factor = .75;
  ROS_INFO("Speeding up...");
  srv.request = CreateBroadcastRequest(time_from_start, dilation_factor);
  if (!broadcast_trajectory_client.call(srv)) {
    ROS_ERROR("Failed to send broadcast trajectory request");
  }

  // After 1 second of that, reset it to normal speed.
  sleep_time = 1.;
  ros::Duration(sleep_time).sleep();
  time_from_start += sleep_time;
  dilation_factor = 1.;
  ROS_INFO("Resetting to normal...");
  srv.request = CreateBroadcastRequest(time_from_start, dilation_factor);
  if (!broadcast_trajectory_client.call(srv)) {
    ROS_ERROR("Failed to send broadcast trajectory request");
  }
#endif  // MODULATE_SPEED

  srv.response.trajectory_duration.sleep();

  // Done!
  ROS_INFO("Planning client completed successfully.");
  return 0;
}
