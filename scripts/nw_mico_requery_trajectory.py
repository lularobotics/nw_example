#!/usr/bin/env python
#
# Simple re-query client. Useful for retiming the trajectory mid-execution.

import trajectory_transform as tt
import copy
import sys
import os
import rospy
import rospkg
import actionlib

import yaml

from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory

from riemo_move_action.srv import *
from riemo_move_action.msg import *

def PrintUsage(): 
  print 'nw_mico_requery_trajectory.py <time_dilation_factor>\n'
  print '  <time_dilation_factor> : factor < 1 speeds up traj; factor > 1 slows traj down.\n'

def CurrentTimeAlongTrajectory():
  secs = int(rospy.get_param('trajectory_start_time_secs'))
  nsecs = int(rospy.get_param('trajectory_start_time_nsecs'))
  trajectory_start_time = rospy.Time(secs, nsecs)

  print 'Querying time'
  now = rospy.Time.now()

  time_diff = now - trajectory_start_time
  return time_diff


# This is a simple mico move client sample program which sends a planning request
# to the planning server and requests a query of the planned trajectory once
# the planning is done.
def main(args):
  rospy.init_node('nw_mico_simple_move_client')

  if len(args) != 1:
    rospy.logerr('Invalid number of arguments')
    PrintUsage()
    return

  dilation_factor = float(args[0])
  print 'dilation factor:', dilation_factor

  # Setup the query request service
  query_trajectory = rospy.ServiceProxy(
      BroadcastTrajectoryRequest.DEFAULT_SERVICE_NAME, BroadcastTrajectory)
  rospy.loginfo('Waiting for trajectory querying service')
  rospy.wait_for_service(BroadcastTrajectoryRequest.DEFAULT_SERVICE_NAME)

  # Send the trajectory query request
  time_from_start = CurrentTimeAlongTrajectory().to_sec()
  print 'time_from_start:', time_from_start
  timing = TrajectoryTiming(time_from_start, dilation_factor)
  srv_result = query_trajectory(trajectory_timing=timing)

  rospy.loginfo('Exiting requery client.')

if __name__ == '__main__':
  main(sys.argv[1:])
