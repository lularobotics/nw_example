#!/usr/bin/env python

import sys
import rospy
import actionlib

from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

from riemo_move_action.srv import *
from riemo_move_action.msg import *

# This method pulls the parameters from the ROS parameter server. Make sure
# those values are set property.
def CreatePlanningRequest():
  goal = PlanGoal

  goal.target = Point()
  goal.target.x = rospy.get_param('/x_target_x')  
  goal.target.y = rospy.get_param('/x_target_y') 
  goal.target.z = rospy.get_param('/x_target_z') 
  goal.approach_constraint_csv = rospy.get_param('/approach_constraint_csv')
  goal.obstacle_linearization_constraint_csv = rospy.get_param('/obstacle_linearization_constraint_csv')
  goal.passthrough_constraint_csv = rospy.get_param('/passthrough_constraint_csv')
  goal.use_upright_orientation_constraint = rospy.get_param('/use_upright_orientation_constraint', False)
  goal.use_upright_orientation_constraint_end_only = rospy.get_param('/use_upright_constraint_end_only', False)

  return goal

# Prints the planning request via ros log
def PrintPlanningRequest(request):
  message = (
    "Planning request:\n"
    "  - target: (%s %s %s)\n" 
    "  - approach_constraint_csv: %s\n" 
    "  - obstacle_linearization_constraint_csv: %s\n"
    "  - passthrough_constraint_csv: %s\n"
    "  - use_upright_orientation_constraint: %s\n"
    "  - use_upright_orientation_constraint_end_only: %s"
    %
    (
      request.target.x, request.target.y, request.target.z, 
      request.approach_constraint_csv,
      request.obstacle_linearization_constraint_csv,
      request.passthrough_constraint_csv,
      request.use_upright_orientation_constraint,
      request.use_upright_orientation_constraint_end_only
    )
  )
  rospy.loginfo(message)


# This is a simple mico move client sample program which sends a planning request
# to the planning server and requests a broadcast of the planned trajectory once
# the planning is done.
def nw_mico_simple_move_client_main():
  rospy.init_node("nw_mico_simple_move_client")
  rospy.loginfo("Running NW MICO simple move client")

  # Setup a client to query a trajectory once planning is complete and publish
  # it to the /joint_trajectory topic immediately on the server side.
  planning_client = actionlib.SimpleActionClient(
    PlanGoal.DEFAULT_ACTION_NAME, PlanAction)
  rospy.loginfo("Waiting for planning server")
  planning_client.wait_for_server()

  # Setup the broadcast request service
  broadcast_trajectory = rospy.ServiceProxy(
    BroadcastTrajectoryRequest.DEFAULT_SERVICE_NAME, BroadcastTrajectory)
  rospy.loginfo("Waiting for trajectory broadcasting service")
  rospy.wait_for_service(BroadcastTrajectoryRequest.DEFAULT_SERVICE_NAME)

  # Send the planning request and wait for planning to complete. 
  planning_request = CreatePlanningRequest()
  rospy.loginfo("Sending planning request ...") 
  PrintPlanningRequest(planning_request)
  planning_status = planning_client.send_goal_and_wait(planning_request)

  if planning_status != GoalStatus.SUCCEEDED:
    rospy.logerr("Planning failed") 
    return

  rospy.loginfo("Planning done and succeeded.")

  # Send the trajectory broadcast request and wait for the execution to finish. 
  rospy.loginfo("Sending trajectory broadcasting request")
  time_from_start = 0.0
  dilation_factor = 1.0
  timing = TrajectoryTiming(time_from_start, dilation_factor)
  srv_result = broadcast_trajectory(trajectory_timing=timing)

  rospy.loginfo("Waiting till trajectory has been executed.")
  rospy.sleep(srv_result.trajectory_duration);

  rospy.loginfo("Exciting planning client.")

if __name__ == "__main__":
  nw_mico_simple_move_client_main()
