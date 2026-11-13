#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import Imu, FluidPressure
from mavros_msgs.msg import VfrHud
from sensor_msgs.msg import PointCloud2, PointField

from std_msgs.msg import Float32MultiArray, Float64

from geometry_msgs.msg import PoseStamped, Point
from geometry_msgs.msg import TransformStamped

from visualization_msgs.msg import Marker

from tf2_ros import TransformBroadcaster

from message_filters import Subscriber, ApproximateTimeSynchronizer

from dvl_msgs.msg import DVLDR, DVLBeam, DVL

from hull_guider.utils.transformations import dvl_to_ros_transform, dvl_to_ros_transform_beams
from hull_guider.core.hull_distance import HullDistanceEstimator

import numpy as np
import socket
import json
import time


import struct


freshwater = 997.0
seawater = 1025.0
APPROPRIATE_RHO = freshwater


R_ros_dvl = np.array([
    [0, 0, 1],   # x_ros = z_dvl
    [0, -1, 0],  # y_ros = -y_dvl
    [1, 0, 0]    # z_ros = x_dvl
])


def quat_to_rot(q):
    x, y, z, w = q[0], q[1], q[2], q[3]
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])

def quat_to_rot_2(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])

def euler_to_quat(r, p, y):
    # Convert degrees to radians first
    r = math.radians(r)
    p = math.radians(p)
    y = math.radians(y)
    # 
    cr, sr = np.cos(r/2), np.sin(r/2)
    cp, sp = np.cos(p/2), np.sin(p/2)
    cy, sy = np.cos(y/2), np.sin(y/2)
    return np.array([
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy
    ])

def quat_to_yaw(q):
    R = quat_to_rot(q)
    return np.arctan2(R[1,0], R[0,0])
    

def axis_angle(axis, angle):
    axis = axis / np.linalg.norm(axis)
    K = np.array([
        [0,-axis[2],axis[1]],
        [axis[2],0,-axis[0]],
        [-axis[1],axis[0],0]
    ])
    return np.eye(3) + np.sin(angle)*K + (1-np.cos(angle))*(K@K)

# =========================
# NODE
# =========================
class DVLHullEstimator(Node):

    def __init__(self):
        super().__init__('dvl_hull_estimator')

        # ---------------- CONFIG ----------------
        self.DVL_IP = "192.168.2.95"
        self.DVL_PORT = 16171

        self.N_INIT = 20          # initial alignment frames
        self.WINDOW_SIZE = 30     # sliding window

        # ---------------- STATE ----------------
        self.init_buffer = []
        self.point_buffer = []
        self.min_plane_points = 10 # Required for startup
        self.max_point_buffer = 12 # used for plane normal estimation
        self.max_pc_buffer = 15

        # R2 parameters
        self.yaw_reject_threshold = np.deg2rad(40.0)   # tune (15–30 deg)
        self.pc_add_yaw_threshold = np.deg2rad(15.0)  # stricter than reject

        self.plane_dist_threshold = 0.4  # meters NOTE (tune)

        # SR1
        self.line_outlier_thresh = 0.65 # meters NOTE tune

        self.enable_distance_filter = True
        self.distance_jump_threshold = 0.5 # meters NOTE tune # below this threshold, smoothing filters should take care of (inside controllers)
        self.distance_window = 5 # how many distances to keep an average over. NOTE tune
        self.distance_history = []
        self.last_valid_distance = None

        # SR1.3

        self.use_top_beams_only = True
        self.top_beam_indices = [2, 3] # NOTE tune # 3 and 4 (1-based) ; 2 and 3 (0-based)

        # Perpendicular distance to the hull (replaces compute_distance_2 + filter_distance).
        # Hull angle from the two top-beam hit points, per-beam perpendicular distance,
        # gate without lockout, small Kalman filter. See hull_guider/core/hull_distance.py
        self.hull_dist = HullDistanceEstimator(
            beam_dirs=self.get_beam_dirs(),
            beam_pair=tuple(self.top_beam_indices),
            gate=0.15,          # m, NOTE tune
            max_rejects=3,      # accept a persistent change after this many rejections
            timeout=3.0,        # s without a beam -> distance marked invalid (bags: longest gap 2.6 s)
            backup_beams=(0, 1),  # lower beams, used only when both top beams are lost
            max_range=2.9,      # m, readings at the DVL range limit (~3 m) mean "no hull"
        )
        self.distance_valid = False
        self.last_dist_time = None
        

        self.yaw_ref = None
        self.stable_plane_normal = None
        self.stable_plane_centroid = None
        self.plane_d = None

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.pc_buffer = [] # your accumulated world point cloud buffer


        # -------------------- TF ---------------------
        self.tf_broadcaster = TransformBroadcaster(self)

        ## ------------ SUBSCRIPTIONS -------------------

        # Define the Deep & Reliable Profile
        mavros_qos = QoSProfile(
            # Reliability: BEST_EFFORT
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # History: KEEP_ALL (Results in "Unknown" depth/No fixed limit)
            history=HistoryPolicy.KEEP_LAST,
            depth=1,                        # TODO note 1.1 dramatically reduce queue sizes
            # Durability: VOLATILE
            durability=DurabilityPolicy.VOLATILE,
            # Lifespan: Infinite (Default)
            lifespan=Duration(seconds=0, nanoseconds=0),
            # Deadline: Infinite (Default)
            deadline=Duration(seconds=0, nanoseconds=0),            
            # Liveliness: AUTOMATIC
            liveliness=rclpy.qos.LivelinessPolicy.AUTOMATIC,
            # Liveliness Lease Duration: Infinite (Default)
            liveliness_lease_duration=Duration(seconds=0, nanoseconds=0)
        )

        # DVL (low rate)
        self.dvl_sub = self.create_subscription(
            DVL,
            "/dvl/data",
            self.dvl_main_callback,
            2  # TODO note 1.1 dramatically reduce queue sizes
        )

        # IMU (high rate, BEST_EFFORT)
        self.imu_sub = self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self.imu_fast_callback,
            mavros_qos
        )

        # Depth
        self.depth_sub = self.create_subscription(
            Float64,
            "/mavros/global_position/rel_alt",
            self.depth_fast_callback,
            mavros_qos
        )

        # ---------------- PUBLISHERS ----------------
        self.pub = self.create_publisher(
            Float32MultiArray,
            "/hull/state",
            10
        )

        self.vision_pub = self.create_publisher(
            PoseStamped,
            "/mavros/vision_pose/pose",
            10
        )

        self.surface_pressure = None
        self.latest_depth = None
        self.latest_depth_msg = None
        self.latest_imu_msg = None
        self.freshwater = 997.0
        self.seawater = 1025.0

        self.position = np.array([0, 0, 0])

        self.prev_dist = None

        self.pose_time =  self.get_clock().now()

        self.ongoing = False
        self.have_q = False

        self.pose_viz_pub = self.create_publisher(PoseStamped, "/viz/pose", 2)

        self.pc_pub = self.create_publisher(PointCloud2, "/dvl/points", 2)

        self.marker_pub = self.create_publisher(Marker, "/viz/plane_normal", 2)

        self.distance_marker_pub = self.create_publisher(Marker, "/viz/perp_distance", 2)

        # ---------------- INIT ----------------
        self.reset_dead_reckoning()

        self.get_logger().info("DVL Hull Estimator Initialized")

    # =========================
    # NETWORK COMMAND
    # =========================
    def reset_dead_reckoning(self):
        cmd = {"command": "reset_dead_reckoning"}

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)  # timeout in seconds

            s.connect((self.DVL_IP, self.DVL_PORT))
            s.sendall(json.dumps(cmd).encode() + b"\n")

            s.close()
            self.get_logger().info("Dead reckoning reset sent")

        except socket.timeout:
            self.get_logger().warn("Timeout while resetting dead reckoning")

        except Exception as e:
            self.get_logger().warn(f"Failed to reset DR: {e}")

    # =========================
    # CALLBACKS
    # =========================

    
    def imu_fast_callback(self, msg: Imu):
        self.latest_imu_msg = msg
    
    def depth_fast_callback(self, msg: FluidPressure):
        self.latest_depth_msg = msg

    # def synced_callback(self, dvl_msg, pos_msg):
    # def synced_callback(self, dvl_msg, imu_msg, depth_msg):
    def dvl_main_callback(self, dvl_msg: DVL):

        # =========================
        # 1) DEPTH 
        # =========================

        depth_msg = self.latest_depth_msg
        imu_msg = self.latest_imu_msg

        if not depth_msg or not imu_msg:
            return

        if hasattr(depth_msg, 'fluid_pressure'):
            pressure_pa = depth_msg.fluid_pressure
        elif hasattr(depth_msg, 'altitude'):
            pressure_pa = depth_msg.altitude
        elif hasattr(depth_msg, 'data'):
            pressure_pa = depth_msg.data

        if not self.surface_pressure:
            self.get_logger().warn("PLEASE MAKE SURE YOU ARE ON THE SURFACE FOR SURFACE PRESSURE CALIBRATION")
            self.surface_pressure = pressure_pa
            self.get_logger().warn(f"SURFACE PRESSURE: {pressure_pa:.0f}")
            pass

        depth = self.pressure_to_depth(pressure_pa, self.surface_pressure, rho=APPROPRIATE_RHO)

        print(f"Depth: {depth:.2f} m for pressure {pressure_pa:.2f} and surface {self.surface_pressure:.2f}")
        self.depth = depth

        # =========================
        # 2) IMU ORIENTATION AND LINEAR ACCEL
        # =========================

        q = self.imu_callback(imu_msg)
        self.q = np.array([q.x, q.y, q.z, q.w])
        # self.is_stationary = is_stationary # another auxiliary output
        
        # =========================
        # 3) OBTAIN VELOCITY AND INTEGRATE POSITION - ALONG wall (negate perpendicular motion first)
        # =========================

        position = self.velocity_integrate(dvl_msg)

        # =========================
        # 4) ESTIMATE RELATIVE DISTANCE & YAW ERROR
        # =========================

        dist, yaw_error, _, _ = self.dvl_data_callback(dvl_msg, position)

        # # =========================
        # # 5) CORRECT POSITION PERPENDICULAR TO WALL ; OBTAIN WORLD POINTS
        # # =========================

        # Publish state
        self.publish_state(yaw_error, dist, self.stable_plane_normal)

        # Publish point cloud
        # self.publish_pointcloud(self.pc_buffer) #, current_points=world_points)
        self.publish_pointcloud(self.point_buffer)# , current_points=points)
        
        msg = "Published "
        # Publish pose for visualization
        self.publish_pose(pos=self.position.tolist(), q=self.q.tolist())
        msg += "pose"
        # self.get_logger().info("Published pose")

        self.publish_tf(pos=self.position.tolist(), q=self.q.tolist())
        # self.get_logger().info("Published tf")
        msg += ", tf"

        # Publish pose for MAVLink guided mode via mavros # TODO : fix ... this should be updated more often
        self.publish_vision_pose(pos=self.position.tolist(), q=self.q.tolist())
        # self.get_logger().info("Published mavros vision pose")
        msg += ", mavros vision pose"
        self.get_logger().info(msg)
        

    def dvl_data_callback(self, msg, position_tang):
        """
        msg.data = beam distances (you adapt this)
        """

        world_points = np.empty(0)
        
        beams = msg.beams

        # =========================
        # YAW FROM STABLE NORMAL (FIRST)
        # =========================
        yaw_err = self.compute_yaw_error_2(self.stable_plane_normal, self.q)

        # =========================
        # DISTANCE FROM YAW ERROR
        # =========================

        # Old: raw_dist = self.compute_distance_2(beams, yaw_err); dist = self.filter_distance(raw_dist)
        # (assumed the DVL square to the hull; jumped when one top beam dropped out)
        now = self.get_clock().now()
        dt = 0.0 if self.last_dist_time is None else (now - self.last_dist_time).nanoseconds * 1e-9
        self.last_dist_time = now
        # DVL velocity (DVL frame), only used to confirm a sudden distance change
        vel = None
        if getattr(msg, "velocity_valid", False):
            vel = [msg.velocity.x, msg.velocity.y, msg.velocity.z]
        dist, self.distance_valid = self.hull_dist.update(
            [b.distance for b in beams], [bool(b.valid) for b in beams], dt, velocity=vel)

        info = self.hull_dist.last
        raw = info.get("raw")
        self.get_logger().info(
            f"hull dist: raw={raw if raw is None else round(raw, 3)} "
            f"beams={info.get('n_beams')} "
            f"angle={np.degrees(self.hull_dist.angle or 0.0):.1f}deg "
            f"rejected={info.get('rejected')} vel_confirmed={info.get('velocity_confirmed')} "
            f"valid={self.distance_valid}")

        # No 0.01 m fallback: before the first valid beam there is no distance at all
        if dist is None:
            dist = 0.0

        # -------------------------
        # HARD REJECTION
        # -------------------------
        reject_yaw = False
        if abs(yaw_err) > self.yaw_reject_threshold:
            self.get_logger().warn(f"Yaw reject: {np.degrees(yaw_err):.2f}")
            reject_yaw = True

        # =======================
        # Project into world points for invalidation check
        # =======================

        # First, get relative points

        points, all_points = self.compute_points(beams) # compute points uses the imperfect self.position (not changed yet)


        # =========================
        # CORRECT POSITION PERPENDICULAR TO WALL, for accurate world point transformation
        # =========================


        if self.stable_plane_normal is None:
            # INITIALIZE FROM IMU YAW
            self.initialize_plane_normal()

        if not (np.any(points)):
            # NOthing . position is only updated tangentially
            self.get_logger().info("There is nothing no points")
            return dist, yaw_err, points, world_points


        # Add delta translation along plane normal (only from a valid, filtered distance)
        if self.distance_valid:
            if self.prev_dist is None:
                self.prev_dist = dist
            delta_d = dist - self.prev_dist # Change in distance in the normal direction
            self.prev_dist = dist
        else:
            delta_d = 0.0
        p_pred = position_tang + delta_d * self.stable_plane_normal

        # Set the latest position and dist variables
        self.position = p_pred
        self.prev_yaw_err = yaw_err

        # =========================
        # 6) Obtain WORLD POINTS for invalidation
        # =========================

        world_points = points + self.position

        if self.stable_plane_centroid is None:
            self.stable_plane_centroid = world_points.mean(axis=0)

        # =========================
        # invalidate plane update
        # =========================
        if not self.is_plane_update_valid(world_points):
            self.get_logger().warn("Plane update rejected (distance)")
            return dist, yaw_err, points, world_points

        if reject_yaw:
            self.get_logger().warn("Plane update rejected (yaw error too extreme)")
            return dist, yaw_err, points, world_points

        #### ----- BEAM POINTS VALID ----- ######

        all_world_points = all_points + self.position

        # Get just the valid subset of the beam points
        # idx = self.get_close_beam_indices(all_world_points)
        # valid_points = all_points[idx]

        # valid_wpoints = valid_points + self.position

        valid_points = points
        valid_wpoints = world_points

        # Update world points buffer IF valid
        self.pc_buffer.extend(valid_wpoints.tolist())
        if len(self.pc_buffer) > self.max_pc_buffer:
            self.pc_buffer = self.pc_buffer[-self.max_pc_buffer:]
        
        ### Use points for stable plane update
        # pts_w = valid_points.copy() # R2 TODO . critical update : ONLY copy valid points, otherwise it gets messy and unstable too quickly, this prevents sharp angular plane deviations 
        pts_w = valid_wpoints.copy() # R2 TODO . critical update : ONLY copy valid points, otherwise it gets messy and unstable too quickly, this prevents sharp angular plane deviations 
        # Uses world points and no centroid matching

        # Few beams
        if len(pts_w) < 2: # because with less than this, the alignment strategy after this code block doesn't work
            # Do not update anything
            
            return dist, yaw_err, np.empty(0), world_points

        # =========================
        # FULL ALIGNMENT
        # =========================

        # CENTROID MATCH strategy
        # =========================
        # c_new = pts_w.mean(axis=0)
        # c_old = self.stable_plane_centroid

        # delta = c_old - c_new

        # # FULL centroid match (no projection)
        # pts_w = pts_w + delta

        # BUFFER UPDATE
        self.point_buffer.extend(pts_w.tolist())
        if len(self.point_buffer) > self.max_point_buffer:
            self.point_buffer = self.point_buffer[-self.max_point_buffer:]
        


        # self.stable_plane_normal, self.stable_plane_centroid = self.fit_plane(np.array(self.point_buffer))
        self.stable_plane_normal, self.stable_plane_centroid = self.fit_wall_2d(np.array(self.point_buffer)) # SR1

        return dist, yaw_err, pts_w, world_points

    def initialize_plane_normal(self):
        # Current IMU yaw
        yaw_imu = quat_to_yaw(self.q)
        # Forward direction (robot heading in world)
        forward = np.array([
            np.cos(yaw_imu),
            np.sin(yaw_imu),
            0.0
        ])
        # Plane normal = OUT OF wall, INTO robot → opposite of forward
        n_init = -forward
        # Normalize (just in case)
        n_init /= np.linalg.norm(n_init)
        # Set stable plane
        self.stable_plane_normal = n_init

        self.get_logger().info(
            f"Initialized plane from IMU yaw: {np.degrees(yaw_imu):.2f} deg"
        )

    def velocity_integrate(self, msg):
        # =========================
        # 1. BUILD A, b FROM BEAMS
        # =========================
        beam_dirs = self.get_beam_dirs()
        beam_dirs = dvl_to_ros_transform_beams(beam_dirs) # DVL → ROS

        A = []
        b = []

        for i, beam in enumerate(msg.beams):
            if beam.valid:
                A.append(beam_dirs[i])
                b.append(beam.velocity)

        A = np.array(A)
        b = np.array(b)

        valid_beams = len(A)

        # =========================
        # 2. VELOCITY RECONSTRUCTION
        # =========================
        v_body = np.zeros(3)

        if valid_beams >= 3:
            v_body, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

        elif valid_beams == 2:
            v_body, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            # v_body *= 0.5  # reduced confidence # not correct, least squares gives optimum answer

        else:
            # no usable velocity
            v_body = np.zeros(3)

        # =========================
        # 3. GATING
        # =========================
        reject = False

        speed = np.linalg.norm(v_body)

        # (A) stationary check
        MIN_SPEED = 0.025
        if self.is_stationary and speed < MIN_SPEED: # NOTE tune
            reject = True

        # (B) velocity clamp
        MAX_SPEED = 1.0  # # NOTE tune
        if speed > MAX_SPEED:
            v_body = v_body * (MAX_SPEED / speed)

        # (C) beam consistency (variance) # turn off for now
        # if valid_beams >= 3:
        #     if np.std(b) > 0.2:  # tune
        #         reject = True

        # Too low, reject.
        if speed < 0.04: # < 4 cm/s # NOTE tune . ideally already captured by self.is_stationary
            reject = True

        if reject:
            v_body = np.zeros(3)

        # =========================
        # 4. TRANSFORM TO WORLD
        # =========================
        R = quat_to_rot(self.q)
        v_world = R @ v_body

        # =========================
        # 5. REMOVE VERTICAL (depth handles Z)
        # =========================
        v_world[2] = 0.0
        # remove z first to make speed clamping more accurate
        

        # =========================
        # 6. PROJECT TO PLANE (tangent motion)
        # =========================
        if self.stable_plane_normal is not None:
            n = self.stable_plane_normal / np.linalg.norm(self.stable_plane_normal)
            v_world = v_world - np.dot(v_world, n) * n
        else:
            n = [-1, 0, 0] # BACKWARDS vector # TODO check correct behavior

        # TODO smoothen - based on position .. position movement completely reject and position is only based on DVL distance. there may be vulnerabilities here

        # =========================
        # 7. INTEGRATE POSITION
        # =========================
        if not hasattr(self, "prev_time"):
            self.prev_time = self.get_clock().now()
            return self.position

        now = self.get_clock().now()

        MAX_DT = 0.5   # 500 ms (tune: 0.05–1.0) # NOTE tune . prevent large jumps in position due to lag time in dvl data but high velocity.

        dt = (now - self.prev_time).nanoseconds * 1e-9
        dt = min(dt, MAX_DT)

        self.prev_time = now

        p_pred = self.position + v_world * dt

        # =========================
        # 8. DEPTH FUSION (hard constraint)
        # =========================
        p_pred[2] = self.depth

        # =========================
        # 9. SAVE STATE
        # =========================
        # self.position = p_pred # do NOT save position yet
        self.prev_velocity = v_world

        return p_pred
    
    def filter_distance(self, dist):
        # LEGACY: no longer used, replaced by HullDistanceEstimator (self.hull_dist)
        if not self.enable_distance_filter:
            return dist

        if dist is None:
            return self.last_valid_distance

        # initialize
        if len(self.distance_history) < 2:
            self.distance_history.append(dist)
            self.last_valid_distance = dist
            return dist

        avg = np.mean(self.distance_history)

        if abs(dist - avg) > self.distance_jump_threshold:
            self.get_logger().warn(f"Distance rejected: {dist:.2f} (avg {avg:.2f})")
            return self.last_valid_distance

        # accept
        self.distance_history.append(dist)
        if len(self.distance_history) > self.distance_window:
            self.distance_history.pop(0)

        self.last_valid_distance = dist
        return dist

    def imu_callback(self, msg: Imu):

        # -------------------------
        # 1. orientation
        # -------------------------
        q = msg.orientation
        self.q = np.array([q.x, q.y, q.z, q.w])

        # -------------------------
        # 2. raw acceleration (body frame)
        # -------------------------
        a = msg.linear_acceleration
        acc_body = np.array([a.x, a.y, a.z])

        # -------------------------
        # 3. remove gravity
        # -------------------------
        R = quat_to_rot(self.q)

        g_world = np.array([0, 0, 9.81])
        g_body = R.T @ g_world   # world → body

        acc_motion = acc_body - g_body

        acc_norm = np.linalg.norm(acc_motion)

        self.acc_filt = acc_norm

        # -------------------------
        # 4. low-pass filter (important)
        # -------------------------
        # alpha = 0.2
        # if not hasattr(self, "acc_filt"):
        #     self.acc_filt = acc_norm
        # else:
        #     self.acc_filt = alpha * acc_norm + (1 - alpha) * self.acc_filt

        # For low ping rate (5 - 10 Hz), this is important to sus out any invalid velocities

        # -------------------------
        # 5. stationary detection
        # -------------------------
        THRESH = 0.04   # tune (0.03–0.1 typical) # NOTE tune

        self.is_stationary = self.acc_filt < THRESH

        # -------------------------
        # debug
        # -------------------------
        # self.get_logger().info(f"acc: {self.acc_filt:.3f} | stationary: {self.is_stationary}")

        return q

    def depth_callback(self, msg):
        pressure_pa = msg.fluid_pressure

        depth = self.pressure_to_depth(pressure_pa, self.surface_pressure, rho=self.freshwater)
        # depth = self.pressure_to_depth(pressure_pa, self.surface_pressure, rho=self.seawater)

        print(f"Depth: {depth:.2f} m")
        return depth
    
    def pressure_to_depth(self, pressure_pa, surface_pressure_pa,
                        rho=997.0, g=9.80665):
        """
        Convert absolute pressure (Pa) to water depth (meters).

        Parameters
        ----------
        pressure_pa : float
            Measured absolute pressure in Pascals.
        surface_pressure_pa : float
            Pressure measured at the water surface (Pa).
        rho : float
            Water density (kg/m^3). Default = freshwater.
            Use 1025 for seawater.
        g : float
            Gravitational acceleration (m/s^2).

        Returns
        -------
        depth_m : float
            Depth in meters.
        """

        # altitude = (sensors[i].ground_pressure - corrected_pressure) / 9800.0f / _specific_gravity;
        # where _specific_gravity is determined by the BARO_SPEC_GRAV parameter (as a proxy for the density of the water being operated in), and the ground_pressure is set when the barometer is calibrated at the surface, and stored in the BARO2_GND_PRESS parameter
        # The alt in the VfrHud message in the machine is not the correct depth information. The depth information is in NAV_CONTROLLER_OUTPUT, but there is no. The alt_error field information in NAV_CONTROLLER_OUTPUT is also displayed in QGC, but the ALT_HUD mode can run normally and maintain depth :joy:. Why is this? Any impact?

        if pressure_pa < 1000: # IF already depth
            return pressure_pa

        depth_m = (pressure_pa - surface_pressure_pa) / (rho * g)
        return depth_m

    # =========================
    # CORE FUNCTIONS
    # =========================
    def compute_points(self, beams):
        """
        Convert beam distances → local points
        Only keep valid beams, and also put all into all_points
        """
        valid_points = []
        all_points = []

        beam_dirs = self.get_beam_dirs()
        beam_dirs = dvl_to_ros_transform_beams(beam_dirs) # Critical fix : beam dirs need to be in ROS frame first

        for i, d_beam in enumerate(beams):

            # SR1.3
            if self.use_top_beams_only and i not in self.top_beam_indices:
                continue

            d = d_beam.distance
            valid = d_beam.valid
            if d <= 0:
                valid = False
            if not valid:
                self.get_logger().info(f"BEAM INVALID {i}")
                # continue
            # else:
                # self.get_logger().info(f"VALID {i}")
            # if True:
                # self.get_logger().info(f"RSSI {d_beam.rssi} \t NSD {d_beam.nsd}")
            
            R_imu = self.quat_to_rot(self.q)

            p_dvl_worldframe = d * beam_dirs[i]

            p_wall = R_imu @ p_dvl_worldframe

            if valid:
                valid_points.append(p_wall)
            
            all_points.append(p_wall)

        return np.array(valid_points), np.array(all_points)

    def get_beam_dirs(self):
        """
        YOUR corrected beam model
        """
        a = np.radians(22.5)
        c = np.cos(a)
        s = np.sin(a) / np.sqrt(2)

        return np.array([
            [-s,  s, c],
            [-s, -s, c],
            [ s, -s, c],
            [ s,  s, c],
        ])


    # =========================
    # OTHER HELPERS
    # =========================

    def is_plane_update_valid(self, world_pts): # threshold in meters
        if self.stable_plane_normal is None or self.stable_plane_centroid is None:
            return True

        world_pts = np.array(world_pts)
        d = np.abs((world_pts - self.stable_plane_centroid) @ self.stable_plane_normal)
        # self.stable_plane_centroid is world coordinates

        return np.mean(d) < self.plane_dist_threshold

    def get_close_beam_indices(self, world_pts):
        if self.stable_plane_normal is None or self.stable_plane_centroid is None:
            return np.array([True, True, True, True])

        world_pts = np.array(world_pts)
        d = np.abs((world_pts - self.stable_plane_centroid) @ self.stable_plane_normal)
        # self.stable_plane_centroid is world coordinates

        indices = np.array([0, 1, 2 , 3])

        close = d < self.plane_dist_threshold
        return indices[close]


    # =========================
    # TRANSFORM
    # =========================
    def transform_points(self, points):
        dyaw = self.yaw - self.yaw_ref
        R = self.rotation_matrix(self.roll, self.pitch, dyaw + self.yaw_ref)

        return np.array([R @ p for p in points])

    def rotation_matrix(self, roll, pitch, yaw):
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0, 0, 1]
        ])

        return Rz @ Ry @ Rx
    
    def get_mount_rotation(self):
        # -90 deg about Y axis
        c = 0.0   # cos(-90°)
        s = -1.0  # sin(-90°)

        R_mount = np.array([
            [ c, 0,  s],
            [ 0, 1,  0],
            [-s, 0,  c]
        ])

        return R_mount
    
    def quat_to_rot(self, q):
        x, y, z, w = q

        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
    
    def to_world(self, pts):
        R_dvl = quat_to_rot(self.q)

        R_mount = np.array([
            [0,0,1],
            [0,1,0],
            [-1,0,0]
        ])

        R_conv = np.array([
            [0,0,1],
            [0,-1,0],
            [1,0,0]
        ])

        R = R_conv @ (R_dvl @ R_mount)

        return (R @ pts.T).T
    
    def compute_distance_2(self, beams, yaw_error):
        # LEGACY: no longer used, replaced by HullDistanceEstimator (self.hull_dist).
        # Note: Rz rotates about the boresight, so align_dir is always [0, 0, 1].
        # Computation done in DVL coordinate frame
        beam_dirs = self.get_beam_dirs()

        # rotation to align with plane
        c = np.cos(-yaw_error)
        s = np.sin(-yaw_error)

        Rz = np.array([
            [c, -s, 0],
            [s,  c, 0],
            [0,  0, 1]
        ])

        
        forward = np.array([0, 0, 1])  # DVL forward
        align_dir = Rz @ forward
        align_dir = align_dir / np.linalg.norm(align_dir)

        dist_list = []

        for i, beam in enumerate(beams):

            # SR1.3
            if self.use_top_beams_only and i not in self.top_beam_indices:
                continue

            d = beam.distance
            valid = beam.valid


            if d <= 0 or not valid:
                continue

            # do not invalidate beams based on distance from plane; distance should still be gotten despite invalidness

            dir_i = beam_dirs[i]

            # beam point
            p_i = d * dir_i

            # projection onto alignment direction
            d_proj = np.dot(p_i, align_dir)

            dist_list.append(d_proj)

        if len(dist_list) == 0:
            return None

        return abs(np.mean(dist_list))
    
    # =========================
    # PLANE ESTIMATION
    # =========================
    def estimate_plane(self):
        pts = np.vstack(self.point_buffer)
        n, d, _ = self.fit_plane(pts)
        self.stable_plane_normal = n
        self.plane_d = d
    
    def fit_wall_2d(self, pts):
        """
        Fit wall in XY plane using robust line fitting.
        Returns:
            normal (3,)
            centroid (3,)
        """

        if len(pts) < 2:
            return self.stable_plane_normal, self.stable_plane_centroid

        # --- 1. Project to XY ---
        xy = pts[:, :2]

        # --- 2. Initial line fit (PCA) ---
        mean = np.mean(xy, axis=0)
        centered = xy - mean

        _, _, vh = np.linalg.svd(centered)
        direction = vh[0]  # principal direction

        # normal in XY = perpendicular
        normal_2d = np.array([-direction[1], direction[0]])
        normal_2d /= np.linalg.norm(normal_2d)

        # --- 3. Outlier rejection (distance to line) ---
        dists = np.abs((xy - mean) @ normal_2d)

        inliers = dists < self.line_outlier_thresh

        if np.sum(inliers) >= 2:
            xy = xy[inliers]
            mean = np.mean(xy, axis=0)
            centered = xy - mean

            _, _, vh = np.linalg.svd(centered)
            direction = vh[0]
            normal_2d = np.array([-direction[1], direction[0]])
            normal_2d /= np.linalg.norm(normal_2d)

        # --- 4. Convert to 3D normal ---
        normal = np.array([normal_2d[0], normal_2d[1], 0.0])

        # --- 5. Orientation / temporal consistency ---
        if self.stable_plane_normal is not None:
            if np.dot(normal, self.stable_plane_normal) < 0:
                normal = -normal

            beta = 0.8 # NOTE tune
            normal = beta * self.stable_plane_normal + (1 - beta) * normal
            normal /= np.linalg.norm(normal)

        else:
            # INITIAL ORIENTATION FROM ROBOT HEADING
            if self.position is not None:
                to_robot = self.position[:2] - mean
                if np.dot(normal[:2], to_robot) < 0:
                    normal = -normal

        centroid = np.array([mean[0], mean[1], np.mean(pts[:, 2])])

        return normal, centroid

    def fit_plane(self, pts):

        if len(pts) < 3:
            return self.stable_plane_normal, self.stable_plane_centroid

        prev_normal = self.stable_plane_normal

        mean = np.mean(pts, axis=0)
        centered = pts - mean

        _, _, vh = np.linalg.svd(centered)
        normal = vh[-1]
        normal /= np.linalg.norm(normal)

        # -------------------------
        # 1. TEMPORAL CONSISTENCY (PRIMARY)
        # -------------------------
        if prev_normal is not None:
            if np.dot(normal, prev_normal) < 0:
                normal = -normal

            # smoothing
            beta = 0.8
            normal = beta * prev_normal + (1 - beta) * normal
            normal /= np.linalg.norm(normal)

        # -------------------------
        # 2. INITIAL ORIENTATION (ONLY IF NO PREV)
        # -------------------------
        elif self.position is not None:
            to_robot = self.position - mean
            if np.dot(normal, to_robot) < 0:
                normal = -normal

        return normal, mean

    def fit_plane_3(self, pts):

        if len(pts) < 3:
            return self.stable_plane_normal, self.stable_plane_centroid

        prev_normal = self.stable_plane_normal

        mean = np.mean(pts, axis=0)
        centered = pts - mean

        _, _, vh = np.linalg.svd(centered)
        normal = vh[-1]
        normal /= np.linalg.norm(normal)

        # enforce temporal consistency
        if prev_normal is not None:
            if np.dot(normal, prev_normal) < 0:
                normal = -normal

        return normal, mean

    def fit_plane_2(self, pts):
        mean = np.mean(pts, axis=0)
        centered = pts - mean

        _, _, vh = np.linalg.svd(centered)
        normal = vh[-1]
        normal /= np.linalg.norm(normal)

        # -------------------------
        # 1. TEMPORAL CONSISTENCY (CRITICAL)
        # -------------------------
        if hasattr(self, "plane_normal") and self.stable_plane_normal is not None:
            if np.dot(normal, self.stable_plane_normal) < 0:
                normal = -normal

        # -------------------------
        # 2. OPTIONAL: initial alignment to robot
        # -------------------------
        elif self.position is not None:
            to_robot = self.position - mean
            if np.dot(normal, to_robot) < 0:
                normal = -normal

        return normal, mean

    # =========================
    # OUTPUT
    # =========================
    def compute_yaw_error_2(self, n, q):

        if self.stable_plane_normal is None:
            return 0.0
        
        # -------------------------
        # 1) horizontal normal
        # -------------------------
        n2 = n.copy()
        n2[2] = 0

        if np.linalg.norm(n2) < 1e-6:
            return 0.0

        n2 /= np.linalg.norm(n2)

        # -------------------------
        # 2) desired direction (into wall)
        # -------------------------
        # normal points toward robot → flip it
        dir_desired = -n2

        yaw_plane = np.arctan2(dir_desired[1], dir_desired[0])

        # -------------------------
        # 3) imu yaw
        # -------------------------
        yaw_imu = quat_to_yaw(q)

        # -------------------------
        # 4) error
        # -------------------------
        # yaw_err = yaw_imu - yaw_plane # Positive, too right; negative : too left.
        yaw_err = yaw_plane - yaw_imu
        yaw_err = np.arctan2(np.sin(yaw_err), np.cos(yaw_err))

        return yaw_err

    def compute_yaw_error_3(self, n, q):
        # Assuming already in ENU / mavros / world frame
        # x forward, y left, z up
        # -------------------------
        # 1) plane yaw
        # -------------------------
        n2 = n.copy()
        n2[2] = 0 # remove horizontal

        if np.linalg.norm(n2) < 1e-6:
            return 0.0

        n2 /= np.linalg.norm(n2)
        # yaw_plane = np.arctan2(n2[1], n2[0])
        # Wall normal points OUT of wall; robot should face INTO wall. fix:
        yaw_plane = np.arctan2(-n2[1], -n2[0])

        # -------------------------
        # 2) imu yaw
        # -------------------------
        yaw_imu = quat_to_yaw(q)

        # -------------------------
        # 3) error (wrapped)
        # -------------------------
        # yaw_err = yaw_imu - yaw_plane # Positive, too right; negative : too left.
        yaw_err = yaw_plane - yaw_imu 

        # wrap to [-pi, pi]
        yaw_err = np.arctan2(np.sin(yaw_err), np.cos(yaw_err))

        return yaw_err

    def compute_distance(self):
        return self.plane_d

    def publish_state(self, yaw_error, dist, plane_normal):
        # Publish distance marker
        self.get_logger().info(f"Distance type: {type(dist)}")
        self.publish_distance_marker(dist)

        self.get_logger().info(f'Dist : {dist:.2f}, Yaw_err : {math.degrees(yaw_error):.2f}')

        nx, ny = plane_normal[0], plane_normal[1]

        msg = Float32MultiArray()
        # 5th element: 1.0 if the distance is valid, 0.0 if not (no beam for > timeout, or none yet)
        msg.data = [float(dist), float(yaw_error), float(nx), float(ny), 1.0 if self.distance_valid else 0.0]
        self.pub.publish(msg)
    
    # ==============================
    # VISUALIZATION
    # ==============================

    def publish_vision_pose(self, pos, q):

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]

        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.vision_pub.publish(msg)
    
    def publish_tf(self, pos, q):

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
    
    def publish_pose(self, pos, q):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.pose_viz_pub.publish(pose)

    def publish_pointcloud(self, points, current_points=None):

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        all_points = []

        # history = white
        for p in points:
            all_points.append((p[0], p[1], p[2], 255, 255, 255))

        # current = red
        if current_points is not None:
            for p in current_points:
                all_points.append((p[0], p[1], p[2], 255, 0, 0))

        msg.height = 1
        msg.width = len(all_points)

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
            PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
            PointField(name='b', offset=14, datatype=PointField.UINT8, count=1),
        ]

        msg.point_step = 15
        msg.row_step = msg.point_step * len(all_points)
        msg.is_dense = True

        buffer = []

        for p in all_points:
            buffer.append(struct.pack('fffBBB', *p))

        msg.data = b''.join(buffer)

        self.pc_pub.publish(msg)
    
    def publish_distance_marker(self, dist):

        if self.stable_plane_normal is None:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "distance"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.02  # line thickness

        # color (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # robot position
        p0 = self.position

        # closest point on plane
        p1 = p0 - dist * self.stable_plane_normal

        # geometry_msgs/Point

        pt0 = Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2]))
        pt1 = Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2]))

        marker.points = [pt0, pt1]

        self.distance_marker_pub.publish(marker)


def main():
    rclpy.init()
    node = DVLHullEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()