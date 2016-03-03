#!/usr/bin/env python
#
# Note: This client is new than the c++ version. It queries for the trajectory
# and converts the finger joints values into command values before publishing
# both it and the original "raw" trajectory to the /joint_trajectory and
# /joint_trajectory_raw topics, respectively.

import sys
import os
import rospy
import rospkg
import actionlib

import yaml

import trajectory_transform as tt

from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory

from riemo_move_action.srv import *
from riemo_move_action.msg import *

# This method pulls the parameters from the task_config dict.
def CreatePlanningRequest(target_x, target_y, target_z, task_config):
  goal = PlanGoal

  goal.target = Point()
  goal.target.x = float(target_x)
  goal.target.y = float(target_y)
  goal.target.z = float(target_z)
  goal.approach_constraint_csv = task_config['approach_constraint_csv']
  goal.shape_approach = bool(task_config['shape_approach'])
  goal.pickup = bool(task_config['pickup'])
  goal.putdown = bool(task_config['putdown'])
  goal.upright_constraint_direction_csv = task_config['upright_constraint_direction_csv']
  goal.use_upright_orientation_constraint = bool(
      task_config['use_upright_orientation_constraint'])
  goal.use_upright_orientation_constraint_end_only = bool(
      task_config['use_upright_orientation_constraint_end_only'])
  goal.behavioral_type = task_config['behavioral_type']
  goal.obstacle_linearization_constraint_csv = task_config['obstacle_linearization_constraint_csv']
  goal.passthrough_constraint_csv = task_config['passthrough_constraint_csv']

  return goal

# Prints the planning request via ros log
def PrintPlanningRequest(request):
  message = (
    'Planning request:\n'
    '  - target: (%s %s %s)\n' 
    '  - approach_constraint_csv: %s\n' 
    '  - shape_approach: %s\n'
    '  - pickup: %s\n'
    '  - putdown: %s\n'
    '  - upright_constraint_direction_csv: %s\n'
    '  - use_upright_orientation_constraint: %s\n'
    '  - use_upright_orientation_constraint_end_only: %s\n'
    '  - behavioral_type: %s\n'
    '  - obstacle_linearization_constraint_csv: %s\n'
    '  - passthrough_constraint_csv: %s\n'
    %
    (
      request.target.x, request.target.y, request.target.z, 
      request.approach_constraint_csv,
      request.shape_approach,
      request.pickup,
      request.putdown,
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
  print 'nw_mico_simple_move_client.py <task_config> <x> <y> <z>\n'
  print '  <task_config> : yaml config specifying the motion task parameters.\n'
  print '  (<x>, <y>, <z>) : specifies the target location.\n'


# This is a simple mico move client sample program which sends a planning request
# to the planning server and requests a broadcast of the planned trajectory once
# the planning is done.
def main(args):
  rospy.init_node('nw_mico_simple_move_client')
  rospy.loginfo('Running NW MICO simple move client')

  #----------------------------------------------------------------------------
  #- Prepare task config ------------------------------------------------------
  #----------------------------------------------------------------------------

  if len(args) != 4:
    rospy.logerr('Invalid number of arguments')
    PrintUsage()
    return

  # default task configuration
  task_config = {
    'approach_constraint_csv' : '' ,
    'shape_approach' : False,
    'pickup' : False,
    'putdown' : False,
    'upright_constraint_direction_csv' : '0.,0.,1.',
    'use_upright_orientation_constraint' : False,
    'use_upright_orientation_constraint_end_only' : False, 
    'behavioral_type' : '',
    'obstacle_linearization_constraint_csv' : '',
    'passthrough_constraint_csv' : ''
  }

  # Load user specific task configuration from the specified yaml file
  rospack = rospkg.RosPack()
  config_file = os.path.join(rospack.get_path('nw_mico_client'), args[0]) 
  with open(config_file, 'r') as config_content:
    loaded_task_config = yaml.load(config_content)

  # Update the default values with user config
  task_config.update(loaded_task_config)
  for key,value in sorted(task_config.iteritems()):
    print key, value

  # Get target position
  target_x = args[1]
  target_y = args[2]
  target_z = args[3]

  # Create a publisher for the publishing the (finger-command-transformed_
  # final trajectory and the raw (fingers with joint values) trajectory.
  trajectory_pub = rospy.Publisher('joint_trajectory', JointTrajectory, queue_size=10)
  #raw_trajectory_pub = rospy.Publisher('joint_trajectory_raw', JointTrajectory, queue_size=10)

  #--------------------------------------------
  #- Create planning client -------------------
  #--------------------------------------------

  # Setup a client to query a trajectory once planning is complete and publish
  # it to the /joint_trajectory topic immediately on the server side.
  planning_client = actionlib.SimpleActionClient(
    PlanGoal.DEFAULT_ACTION_NAME, PlanAction)
  rospy.loginfo('Waiting for planning server')
  planning_client.wait_for_server()

  #----------------------------------------------------------------------------
  #- Create trajectory service ------------------------------------------------
  #----------------------------------------------------------------------------

  # Setup the broadcast request service
  query_trajectory = rospy.ServiceProxy(
      QueryTrajectoryRequest.DEFAULT_SERVICE_NAME, QueryTrajectory)
  rospy.loginfo('Waiting for trajectory broadcasting service')
  rospy.wait_for_service(BroadcastTrajectoryRequest.DEFAULT_SERVICE_NAME)

  #----------------------------------------------------------------------------
  #- Create and send planning request -----------------------------------------
  #----------------------------------------------------------------------------

  # Send the planning request and wait for planning to complete. 
  planning_request = CreatePlanningRequest(target_x, target_y, target_z, task_config)
  rospy.loginfo('Sending planning request ...') 
  PrintPlanningRequest(planning_request)
  planning_status = planning_client.send_goal_and_wait(planning_request)

  if planning_status != GoalStatus.SUCCEEDED:
    rospy.logerr('Planning failed') 
    return

  rospy.loginfo('Planning done and succeeded.')

  #----------------------------------------------------------------------------
  #- Reuqest trajectory broadcast ---------------------------------------------
  #----------------------------------------------------------------------------

  # Send the trajectory broadcast request and wait for the execution to finish. 
  rospy.loginfo('Sending trajectory broadcasting request')
  time_from_start = 0.0
  dilation_factor = 1.0  # Play with this to speed up (<1) or slow down (>1) execution.
  timing = TrajectoryTiming(time_from_start, dilation_factor)
  srv_result = query_trajectory(trajectory_timing=timing)

  now = rospy.Time.now()
  rospy.set_param('trajectory_start_time_secs', now.secs)
  rospy.set_param('trajectory_start_time_nsecs', now.nsecs)

  trajectory_pub.publish(srv_result.trajectory);
  # Transform the trajectory to convert the finger jont values into commands in
  # the range [0,6000].  Note the joint limit is set to .8 (the real max is
  # 1.2), so the true range of possible commands ends up being [0,4000].
  #transformed_trajectory = tt.TransformFingerCommands(srv_result.trajectory, 6000.)
  #trajectory_pub.publish(transformed_trajectory);
  #raw_trajectory_pub.publish(srv_result.trajectory);

  #rospy.loginfo('Waiting till trajectory has been executed.')
  #rospy.sleep(srv_result.trajectory.points[-1].time_from_start);

  rospy.loginfo('Exiting planning client.')

if __name__ == '__main__':
  main(sys.argv[1:])
