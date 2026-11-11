controller_architecture:
  frame: global_world_frame   # NOT body frame
  command_type: velocity + yaw
  mavros_interface: setpoint_raw/local

control_objective:
  primary_goal: maintain_constant_distance_to_wall
  secondary_goal: move_tangentially_along_wall

geometry:
  plane_normal:
    source: averaged_stable_pointcloud
    filtering:
      temporal_smoothing: true
      outlier_rejection: true
      gating:
        condition: point_distance_to_plane < threshold
    usage:
      - defines_normal_velocity_direction
      - defines_yaw_reference_direction

  tangent_direction:
    computation: cross(z_axis, plane_normal)
    purpose: wall_following_motion

control_law:
  velocity:
    normal_component:
      type: proportional
      input: (distance - target_distance)
      direction: plane_normal
    tangential_component:
      type: constant_or_slow_scan
      direction: tangent_direction

  yaw:
    reference: derived_from_plane_normal
    measurement: imu_yaw
    error: yaw_ref - imu_yaw
    note: NOT computed from instantaneous pointcloud

robustness:
  drift_handling:
    strategy: local_geometric_control
    rationale: control relative to plane, not absolute map

  estimator_constraints:
    plane_update:
      only_use_stable_points: true
      reject_outliers: true

  decoupling:
    yaw_and_translation: independent
    avoid_body_frame_coupling: true

implementation_notes:
  use_guided_mode: true
  avoid_rc_override: true
  avoid_attitude_targets: true



okay now replace the setpoint velocity with setpoint raw local and give the entire code. setpoint point is also possible. but i dont know what is better (assume that you have the position). should i control global frame or relative frame (body)? i think global is safer right? because adjusting the yaw AND giving body relative velocities is going to be globally suboptimal. 

assume : we find the global velocity direction that minimizes the distance. what algorithm can we use to do this? 

for starters, we COULD publish the averaged, stable plane normal called /plane_normal (so this gives us the proper global normal velocity direction and global wall-following (tangential) direction ... this gives us a much more stable and decoupled yaw and thrust positions. 

so that even if the position estimate given to ekf is noisy and drifts over time, i think giving global velocity commands is more stable right? 

then, the controller will be adding normal direction velocity only so long as to keep the robot within the set "distance" given by /hull/state . but for stability purposes, the long term reference 'plane normal angle' for yaw error reference should be updated based on the : 

1. averaged stable plane normal mentioned above 
2. the averaged stable plane normal above should only be updated with hull points which are STABLE and not far in normal distance from the existing point clouds (otherwise, reject these point clouds). 
3. the yaw error produced in the /hull/state should not use the point cloud method, but use the difference between IMU yaw and this stable yaw error reference from the averaged stable plane normal mentioned above. the position estimate can remain as it is in the code.



## second thoughts (R2)



we compute the yaw_error purely from IMU using the previous stable plane normal. from this, we can get the dist. from this, we can project the DVL beam points into both a world point space (used for invalidating this latest beam) and a relative point space (relative to the centroid, like in original code) (IF these current points are valid, they are used for adding to and calculating stable plane normal).

update invalidation : 
1. is plane update valid should use stable plane normal and stable plane centroid
2. If temporar

add points to pc_buffer ONLY if the yaw error is within the bounds allowed (otherwise, risks inaccuracy) - this parameter of the yaw bounds can be adjusted.



the critical part for turning tight corners:
1. TODO. CONTROLLER needs to SLOW DOWN is too much yaw errors ... it may signal a sharp turn. maybe backtrack a little? let the state estimator recover a stable estimate of the perpendicular hull.


## IN the case of huge drift from wall

and there are no updates whatsoever to plane,

the controller should track BACK to the place of the world points where the wall was last detected.
- TODO .very important!


filter smoothen the wall plane normals
- 



### Filters (Update R3)

1. the yaw control is already smooth, it can hold yaw and even correct it
-> if yaw gets high, stop all lateral movement and get close

2. the lateral movement is not smooth if the plane normal changes rapidly -> smoothing Filter for plane normal
- smoothing filter is responsibility of controller, not hull estimator.

3. the distance movement is not smooth - if the distance fluctuates quickly.
-> smoothing filter for distance
-> smoothing filter is responsibility of controller, not hull estimator.
-> if distance increases rapidly, slow down lateral movement and attempt to wait for yaw to return to stable regime first or adapt to new surface 

4. is the position update fast enough to update the controller? does the EKF make use of the accelerator? does the EKF make use of the accelerator? we COULD technically enable VLXY from accel as a high frequency component
- this will introduce drift

5. smoothen the position publication?
- do not do because it will be filtered and the velocity will be filtered.
- keep raw position updates noisy and raw
- ekf trusts timestamps, do not lie to the ekf



