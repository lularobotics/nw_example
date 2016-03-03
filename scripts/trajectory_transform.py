import copy

def GetJointIndex(trajectory, joint_name):
  for i, name in enumerate(trajectory.joint_names):
    if joint_name == name: return i

def TransformPoint(point, j1_index, j2_index, max_command):
  new_point = copy.deepcopy(point)

  new_point.positions = list(point.positions)
  new_point.velocities = list(point.velocities)
  new_point.accelerations = list(point.accelerations)

  new_point.positions[j1_index] *= max_command / 1.2;
  new_point.velocities[j1_index] = 0.
  new_point.accelerations[j1_index] = 0.

  new_point.positions[j2_index] *= max_command / 1.2;
  new_point.velocities[j2_index] = 0.
  new_point.accelerations[j2_index] = 0.

  return new_point

def TransformFingerCommands(trajectory, max_command):
  j1_index = GetJointIndex(trajectory, 'mico_joint_finger_1') 
  j2_index = GetJointIndex(trajectory, 'mico_joint_finger_2') 
  transformed_trajectory = copy.deepcopy(trajectory)
  for i in range(len(trajectory.points)):
    transformed_trajectory.points[i] = TransformPoint(
        transformed_trajectory.points[i], j1_index, j2_index, max_command)

  return transformed_trajectory


