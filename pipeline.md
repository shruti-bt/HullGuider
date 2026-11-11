
A succinct summary of the pipeline

to make velocity gating more accurate, increase gain so that movements are more snappy.


TODO :
1. we invalidate plane normal / reject addition to point buffer if yaw error is too large within the last few frames according to MAVROS IMU

```yaml
state:
  position: [x, y, z_depth]
  orientation: quaternion   # from IMU
  plane_normal: [nx, ny, nz]
  plane_centroid: [cx, cy, cz]
  depth: z_depth

inputs:
  dvl:
    topic: /dvl/data
    data: [beam.distance, beam.valid, beam.velocity for beam in msg.beams]
  imu:
    topic: /mavros/imu/data
    data: orientation, linear_acceleration
  depth:
    topic: /mavros/imu/water_pressure
    data: fluid_pressure / static_pressure

synchronization:
  type: ApproximateTimeSynchronizer
  topics: [dvl, imu, depth]
  queue_size: 30
  slop: 0.05-0.1

pipeline:

  - step: orientation
    source: imu
    output: R_world_from_body

  - step: velocity_reconstruction
    input: dvl_beams
    method: least_squares
    conditions:
      min_valid_beams: 3
      fallback:
        beams_2: reduced_confidence
        beams_<2: reject
        if valid_beams >= 3:
            full solve

        elif valid_beams == 2:
            solve (low confidence)

        else:
            v_world = 0  # or decay

  - step: gating
    checks:
      - stationary_detection:
          source: imu_acceleration
          condition: low_accel AND nonzero_velocity -> reject
      - velocity_limits:
          clamp_max_speed: true
      - beam_consistency:
          high_variance -> reject

  - step: transform_velocity
    operation: v_world = R * v_body

  - step: remove_vertical_component
    operation: v_world[2] = 0

  - step: project_to_plane
    operation: v_world = v_world - dot(v_world, n) * n

  - step: integrate_position
    operation: p_pred = p_prev + v_world * dt

  - step: plane_update
    input: dvl_points
    method: fit_plane (SVD)
    constraint: normal points robot -> plane

  - step: soft_plane_correction
    optional: true
    operation: p_pred = p_pred - alpha * dot(n, (p_pred - centroid)) * n
    alpha: 0.1

  - step: depth_fusion
    operation: p_pred[2] = depth

  - step: update_state
    operation: position = p_pred

output:
  position: world_frame_xyz
  orientation: imu_quaternion
  derived:
    yaw_error: from plane_normal vs imu_yaw
    distance_to_wall: projected distance

properties:
  motion_model: tangent_to_local_plane
  z_source: depth_sensor (absolute)
  xy_source: dvl_velocity (relative)
  drift_control:
    normal_direction: plane_constraint
    vertical: depth_override
```