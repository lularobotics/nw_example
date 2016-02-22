// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <actionlib/client/simple_action_client.h>

// RieMO move action
#include <riemo_move_action/PlanAction.h>

// RieMO move service
#include <riemo_move_action/BroadcastTrajectory.h>
#include <riemo_move_action/QueryTrajectory.h>

#include <iostream>
using std::cout;
using std::endl;
using std::flush;

// Uncomment to play around with modulating the speed.
//#define MODULATE_SPEED

typedef actionlib::SimpleActionClient<riemo_move_action::PlanAction>
    PlanningClient;

template <class T>
T Get(const YAML::Node& node, const std::string& name, const T& default_value) {
  auto field = node[name];
  if (!field) return default_value;
  return field.as<T>();
}

// Setup and a plan "goal". "Goal" is somewhat of a misnomer, but you can think
// of it as the "goal" os optimization, i.e. the constraints and objective. In
// this case, it includes a target, end-effector constraints, and behavioral
// constraints.
//
// This method pulls the parameters from the ROS parameter server. Make sure
// those values are set property.
riemo_move_action::PlanGoal CreatePlanningRequest(
    double target_x,
    double target_y,
    double target_z,
    const YAML::Node& task_config) {
  riemo_move_action::PlanGoal goal;

  goal.target.x = target_x;
  goal.target.y = target_y;
  goal.target.z = target_z;

  // Set approach parameters
  goal.approach_constraint_csv =
      Get<std::string>(task_config, "approach_constraint_csv", "");
  goal.shape_approach = Get<bool>(task_config, "shape_approach", false);

  // Set upright orientation constraint parameters
  goal.upright_constraint_direction_csv = Get<std::string>(
      task_config, "upright_constraint_direction_csv", "0.,0.,1.");
  goal.use_upright_orientation_constraint =
      Get<bool>(task_config, "use_upright_orientation_constraint", false);
  goal.use_upright_orientation_constraint_end_only = Get<bool>(
      task_config, "use_upright_orientation_constraint_end_only", false);

  // Set behavioral constraints
  goal.behavioral_type = Get<std::string>(task_config, "behavioral_type", "");
  goal.obstacle_linearization_constraint_csv =
      Get<std::string>(task_config, "obstacle_linearization_constraint_csv", "");
  goal.passthrough_constraint_csv =
      Get<std::string>(task_config, "passthrough_constraint_csv", "");

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

void PrintUsage() {
  cout << "nw_mico_simple_move_client <task_config> <x> <y> <z>" << endl;
  cout << "  <task_config> : yaml config specifying the motion task parameters." << endl;
  cout << "  (<x>, <y>, <z>) : specifies the target location." << endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "nw_mico_simple_move_client");

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  if (argc != 5) {
    cout << "ERROR -- invalid number of arguments." << endl;
    PrintUsage();
    return 1;
  }

  // Parse the arguments.
  std::string rel_task_config = argv[1];
  double target_x = std::stod(argv[2]);
  double target_y = std::stod(argv[3]);
  double target_z = std::stod(argv[4]);
  cout << "x_target: (" << target_x << ", " << target_y << ", " << target_z
       << ")" << endl;

  // Load the yaml task config.
  auto task_config_path = ros::package::getPath("nw_mico_client") +
                          ((rel_task_config[0] == '/') ? "" : "/") +
                          rel_task_config;
  auto task_config = YAML::LoadFile(task_config_path);

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
  auto planning_request =
      CreatePlanningRequest(target_x, target_y, target_z, task_config);
  auto final_state = planning_client.sendGoalAndWait(planning_request);
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
