# Executes an LQR problem.

X_TARGET_CSV=

################################################################################
#-------------------------------------------------------------------------------
# Problem specification constraint parameters
# 1. USE_UPRIGHT_ORIENTATION_CONSTRAINT: Keep end effector upright throughout
#    motion 
# 2. USE_UPRIGHT_ORIENTATION_CONSTRAINT_END_ONLY: Keep end-effector upright
#    only at final config
# 3. APPROACH_CONSTRAINT_CSV: When set, constraints final configuration to have
#    end-effector z-axis pointed in this direction.
# 4. SPHERE_OBSTACLE_CONSTRAINT_CSV: When set, adds a sphereical obstacle
#    constraint.  Format: 'x,y,z,radius'
# 5. PASSTHROUGH_CONSTRAINT_CSV: When set, adds a constraint specifying a box
#    that the robot should pass through. Format: 'x,y,z,radius,t_fraction'. The
#    constraint is added at t = t_fraction * T. Give the user control over how
#    the robot moves around the object. This version is more restrictive than
#    OBSTACLE_LINEARIZATION_CONSTRAINT_CSV, which just specifies whether the end-
#    effector should pass above or below a plane.
# 6. OBSTACLE_LINEARIZATION_CONSTRAINT_CSV: When set, adds a constraint specifying 
#    a linearization of the obstacle constraint at a pointon the surface. the 
#    constraint will be applied at a single configuration somewhere along the 
#    trajectory (user specified). Format: 'x,y,z,t_fraction'. (x,y,z) specifies
#    a point in 3-space. The point on the surface is deduced as the point where 
#    the ray from the sphere's center to the specified point intersects the surface.
#    The resulting constraint is a linear constraint, with zero value at that 
#    point, increasing positively in the direction of the ray. (The zero set of the 
#    linearization is a tangent plane to the sphere.) t_fraction specifies where 
#    along the trajectory the constraint will be applied as t = T * t_fraction.
#    Usually, t_fraction = .5 (half way through the trajectory) is good. Gives
#    the user control over how the robot moves around the object. This version
#    is less restrictive than PASSTHROUGH_CONSTRAINT_CSV, which specifies a 
#    box in 3-space through which the robot's end-effector should pass.
#-------------------------------------------------------------------------------

################################################################################


roslaunch nw_mico_client nw_mico_simple_client_demo.launch \
  $1 $2 $3 \
  use_upright_orientation_constraint:=$USE_UPRIGHT_ORIENTATION_CONSTRAINT \
  use_upright_orientation_constraint_end_only:=$USE_UPRIGHT_ORIENTATION_CONSTRAINT_END_ONLY \
  approach_constraint_csv:="$APPROACH_CONSTRAINT_CSV" \
  sphere_obstacle_constraint_csv:="$SPHERE_OBSTACLE_CONSTRAINT_CSV" \
  passthrough_constraint_csv:="$PASSTHROUGH_CONSTRAINT_CSV" \
  obstacle_linearization_constraint_csv:="$OBSTACLE_LINEARIZATION_CONSTRAINT_CSV"
