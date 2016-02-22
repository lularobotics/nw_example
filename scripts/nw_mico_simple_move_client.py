#!/usr/bin/env python

# Note: this client is identical to the c++ version and it can be used in the
# same way

import sys
import os
import rospy
import rospkg
import actionlib

import yaml

from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

from riemo_move_action.srv import *
from riemo_move_action.msg import *

# This method pulls the parameters from the ROS parameter server. Make sure
# those values are set property.
def CreatePlanningRequest(target_x, target_y, target_z, task_config):
  goal = PlanGoal

  goal.target = Point()
  goal.target.x = float(target_x)
  goal.target.y = float(target_y)
  goal.target.z = float(target_z)
  goal.approach_constraint_csv = task_config['approach_constraint_csv']
  goal.shape_approach = bool(task_config['shape_approach'])
  goal.upright_constraint_direction_csv = task_config['upright_constraint_direction_csv']
  goal.use_upright_orientation_constraint = bool(task_config['use_upright_orientation_constraint'])
  goal.use_upright_orientation_constraint_end_only = bool(task_config['use_upright_constraint_end_only'])

  goal.behavioral_type = task_config['behavioral_type']
  goal.obstacle_linearization_constraint_csv = task_config['obstacle_linearization_constraint_csv']
  goal.passthrough_constraint_csv = task_config['passthrough_constraint_csv']

  return goal

# Prints the planning request via ros log
def PrintPlanningRequest(request):
  message = (
    "Planning request:\n"
    "  - target: (%s %s %s)\n" 
    "  - approach_constraint_csv: %s\n" 
    "  - shape_approach: %s\n"
    "  - upright_constraint_direction_csv: %s\n"
    "  - use_upright_orientation_constraint: %s\n"
    "  - use_upright_orientation_constraint_end_only: %s\n"
    "  - behavioral_type: %s\n"
    "  - obstacle_linearization_constraint_csv: %s\n"
    "  - passthrough_constraint_csv: %s\n"
    %
    (
      request.target.x, request.target.y, request.target.z, 
      request.approach_constraint_csv,
      request.shape_approach,
      request.upright_constraint_direction_csv,
      request.use_upright_orientation_constraint,
      request.use_upright_orientation_constraint_end_only,
      request.behavioral_type,
      request.obstacle_linearization_constraint_csv,
      request.passthrough_constraint_csv
    )
  )
  rospy.loginfo(message)

def PrintUsage(): 
  print "nw_mico_simple_move_client.py <task_config> <x> <y> <z>\n"
  print "  <task_config> : yaml config specifying the motion task parameters.\n"
  print "  (<x>, <y>, <z>) : specifies the target location.\n"


# This is a simple mico move client sample program which sends a planning request
# to the planning server and requests a broadcast of the planned trajectory once
# the planning is done.
def main(args):
  rospy.init_node("nw_mico_simple_move_client")
  rospy.loginfo("Running NW MICO simple move client")

  #############################################
  ## Prepare task config ######################
  #############################################

  if len(args) != 4:
    rospy.logerr("Invalid number of arguments")
    PrintUsage()
    return

  # default task configuration
  task_config = {
    'approach_constraint_csv' : "" ,
    'shape_approach' : False,
    'upright_constraint_direction_csv' : "0.,0.,1.",
    'use_upright_orientation_constraint' : False,
    'use_upright_constraint_end_only' : False, 
    'behavioral_type' : "",
    'obstacle_linearization_constraint_csv' : "",
    'passthrough_constraint_csv' : ""
  }

  # load user specific task configuration from the specified yaml file
  rospack = rospkg.RosPack()
  config_file = os.path.join(rospack.get_path('nw_mico_client'), args[0]) 
  with open(config_file, 'r') as config_content:
    loaded_task_config = yaml.load(config_content)

  # update the default values with user config
  task_config.update(loaded_task_config)

  # get target position
  target_x = args[1]
  target_y = args[2]
  target_z = args[3]

  #############################################
  ## Create planning client ###################
  #############################################

  # Setup a client to query a trajectory once planning is complete and publish
  # it to the /joint_trajectory topic immediately on the server side.
  planning_client = actionlib.SimpleActionClient(
    PlanGoal.DEFAULT_ACTION_NAME, PlanAction)
  rospy.loginfo("Waiting for planning server")
  planning_client.wait_for_server()

  #############################################
  ## Create trajectory service ################
  #############################################

  # Setup the broadcast request service
  broadcast_trajectory = rospy.ServiceProxy(
    BroadcastTrajectoryRequest.DEFAULT_SERVICE_NAME, BroadcastTrajectory)
  rospy.loginfo("Waiting for trajectory broadcasting service")
  rospy.wait_for_service(BroadcastTrajectoryRequest.DEFAULT_SERVICE_NAME)

  #############################################
  ## Create and send planning request #########
  #############################################

  # Send the planning request and wait for planning to complete. 
  planning_request = CreatePlanningRequest(target_x, target_y, target_z, task_config)
  rospy.loginfo("Sending planning request ...") 
  PrintPlanningRequest(planning_request)
  planning_status = planning_client.send_goal_and_wait(planning_request)

  if planning_status != GoalStatus.SUCCEEDED:
    rospy.logerr("Planning failed") 
    return

  rospy.loginfo("Planning done and succeeded.")

  #############################################
  ## Reuqest trajectory broadcast #############
  #############################################

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
  main(sys.argv[1:])
