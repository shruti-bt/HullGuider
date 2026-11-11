#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import Imu, FluidPressure
from mavros_msgs.msg import VFR_HUD
from sensor_msgs.msg import PointCloud2, PointField

from std_msgs.msg import Float32MultiArray

from geometry_msgs.msg import PoseStamped, Point
from geometry_msgs.msg import TransformStamped

from visualization_msgs.msg import Marker

from tf2_ros import TransformBroadcaster

from message_filters import Subscriber, ApproximateTimeSynchronizer

from dvl_msgs.msg import DVLDR, DVLBeam, DVL

from hull_guider.utils.transformations import dvl_to_ros_transform, dvl_to_ros_transform_beams

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
        self.max_point_buffer = 15 # used for plane normal estimation
        self.max_pc_buffer = 15

        self.yaw_ref = None
        self.plane_normal = None
        self.plane_centroid = None
        self.plane_d = None

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.pc_buffer = [] # your accumulated world point cloud buffer


        # -------------------- TF ---------------------
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- SUBSCRIBERS ----------------
        # self.create_subscription(
        #     DVL,
        #     "/dvl/data",
        #     self.dvl_data_callback,
        #     10
        # )

        # self.create_subscription(
        #     DVLDR,  # replace with actual DVLDR msg later
        #     "/dvl/position",
        #     self.dvl_position_callback,
        #     10
        # )

        ## ------------ SUBSCRIPTIONS -------------------

        # Define the Deep & Reliable Profile
        mavros_qos = QoSProfile(
            # Reliability: BEST_EFFORT
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # History: KEEP_ALL (Results in "Unknown" depth/No fixed limit)
            history=HistoryPolicy.KEEP_LAST,
            depth=2,                        # TODO note 1.1 dramatically reduce queue sizes
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
            1  # TODO note 1.1 dramatically reduce queue sizes
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
            VFR_HUD,
            # "/mavros/imu/static_pressure",
            "/mavros/vfr_hud",
            self.depth_fast_callback,
            mavros_qos
        )

        # self.sync = ApproximateTimeSynchronizer(
        #     [self.dvl_sub, self.pos_sub],
        #     queue_size=20,
        #     slop=0.1   # seconds tolerance (tune this)
        # )

        # self.sync = ApproximateTimeSynchronizer(
        #     [
        #         self.dvl_sub,
        #         # self.pos_sub,      # optional (can remove later)
        #         self.imu_sub,
        #         self.depth_sub
        #     ],
        #     queue_size=2, # TODO note 1.1 dramatically reduce queue sizes
        #     slop=0.1
        # )

        # self.sync.registerCallback(self.synced_callback)

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

        # self.publish_rate = 0.1  # 20 Hz

        # self.timer = self.create_timer(
        #     self.publish_rate,
        #     self.timer_callback
        # )

        self.surface_pressure = None
        self.latest_depth = None
        self.latest_depth_msg = None
        self.latest_imu_msg = None
        # self.freshwater = 997.0
        # self.seawater = 1025.0

        self.position = np.array([0, 0, 0])

        self.prev_dist = 0.0

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
            s.settimeout(2.0)  # ⏱️ timeout in seconds

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

        dist, yaw_error, points = self.dvl_data_callback(dvl_msg)

        # =========================
        # 5) CORRECT POSITION PERPENDICULAR TO WALL ; OBTAIN WORLD POINTS
        # =========================

        if not (dist and yaw_error and np.any(points)):
            # NOthing . however, position has been updated somewhat.
            return

        # Add delta translation along plane normal
        delta_d = dist - self.prev_dist # Change in distance in the normal direction
        p_pred = position + delta_d * self.plane_normal

        # Set the latest position and dist variables
        self.position = p_pred
        self.prev_dist = dist
        self.prev_yaw_err = yaw_error

        # =========================
        # 6) Obtain WORLD POINTS after getting final position p_robot / self.p / self.position
        # =========================

        world_points = points + p_pred

        # Update world points buffer
        self.pc_buffer.extend(world_points.tolist())
        if len(self.pc_buffer) > self.max_pc_buffer:
            self.pc_buffer = self.pc_buffer[-self.max_pc_buffer:]

        # Publish state
        self.publish_state(yaw_error, dist)

        # Publish point cloud
        self.publish_pointcloud(self.pc_buffer, current_points=world_points)
        
        # Publish pose for visualization
        self.publish_pose(pos=self.position.tolist(), q=self.q.tolist())
        self.get_logger().info("Published pose")

        self.publish_tf(pos=self.position.tolist(), q=self.q.tolist())
        self.get_logger().info("Published tf")

        # Publish pose for MAVLink guided mode via mavros # TODO : fix ... this should be updated more often
        # sdjflaskjdfalkdfj
        self.publish_vision_pose(pos=self.position.tolist(), q=self.q.tolist())
        self.get_logger().info("Published mavros vision pose")
       
        # self.dvl_data_callback(dvl_msg)

    def dvl_position_callback(self, msg):
        """
        Replace with actual message fields later
        """
        
        self.get_logger().info("Not used!!!")

        return

        self.get_logger().info("Got DVL POSITION")

        position = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z,
        ])

        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw

        # Convert DVL original to mount

        q_xyzw = euler_to_quat(roll, pitch, yaw)     
        q_ros, t_ros = dvl_to_ros_transform(q_xyzw, position)
        self.q = q_ros
        self.p = t_ros
        self.pose_time =  self.get_clock().now()
        self.have_q = True

        # Publish pose for visualization
        # self.publish_pose(pos=t_ros.tolist(), q=q_ros.tolist())
        self.publish_tf(self, pos=t_ros, q=q_ros)

        # Publish pose for MAVLink guided mode via mavros
        self.publish_vision_pose(t_ros.tolist(), q_ros.tolist())

        

    def dvl_data_callback(self, msg):
        """
        msg.data = beam distances (you adapt this)
        """

        # # no blocking
        # if not hasattr(self, "q") or not hasattr(self, "p"):
        #     return  # no pose yet

        # # optional: check freshness
        # now = self.get_clock().now()
        # if (now - self.pose_time).nanoseconds > 200_000_000:  # 200 ms
        #     self.get_logger().warn("Pose too old, skipping")
        #     return

        nothing = None, None, None
        
        beams = msg.beams

        points, world_points = self.compute_points(beams)
        
        if not np.any(points):
            self.get_logger().info("NO POINTS NOTHING")
            return nothing
        
        # Add world points to buffer for visualization
        # self.pc_buffer.extend(world_points.tolist())
        # if len(self.pc_buffer) > self.max_pc_buffer:
        #     self.pc_buffer = self.pc_buffer[-self.max_pc_buffer:]

        # pts_w = self.to_world(points) # ALREADY world points 
        pts_w = points.copy()

        # INIT
        if self.plane_normal is None:
            if len(pts_w) < 3:
                return nothing
            self.point_buffer.extend(pts_w)
            if len(self.point_buffer) > self.min_plane_points:
                self.plane_normal, self.plane_centroid = self.fit_plane(np.array(self.point_buffer))
            return nothing

        # < 3 beams
        if len(pts_w) < 2: # because with less than this, the alignment strategy after this code block doesn't work
        # if False:

            # ------------------------
            # Compute yaw error without updating it first
            # ------------------------
            yaw_err = self.compute_yaw_error_2(self.plane_normal, self.q)

            # ------------------------
            # PROJECT ONTO EXISTING PLANE
            # ------------------------
            pts_w = np.array(pts_w)

            # distance along normal
            d = self.plane_normal @ (pts_w.mean(axis=0) - self.plane_centroid)

            # project onto plane
            pts_proj = pts_w - d * self.plane_normal
            # match centroid
            # pts_proj += (self.plane_centroid - pts_proj.mean(axis=0))

            # ------------------------
            # ADD TO BUFFER
            # ------------------------
            self.point_buffer.extend(pts_proj.tolist())

            if len(self.point_buffer) > self.max_point_buffer:
                self.point_buffer = self.point_buffer[-self.max_point_buffer:]

            # ------------------------
            # REFIT GLOBAL PLANE
            # ------------------------
            self.plane_normal, self.plane_centroid = self.fit_plane(np.array(self.point_buffer))

            # Compute distance
            # dist = d
            dist = self.compute_distance_2(beams, yaw_err)

            # publish state
            # self.publish_state(yaw_err, dist)

            # # publish point cloud
            # self.publish_pointcloud(self.pc_buffer, current_points=world_points)
            
            return dist, yaw_err, points

        # =========================
        # FULL ALIGNMENT
        # =========================

        # =========================
        # CENTROID MATCH strategy
        # =========================
        c_new = pts_w.mean(axis=0)
        c_old = self.plane_centroid

        delta = c_old - c_new

        # FULL centroid match (no projection)
        pts_w = pts_w + delta

        # =========================
        # UPDATE BUFFER
        # =========================
        # self.point_buffer.extend(pts_w.tolist())

        # if len(self.point_buffer) > self.max_point_buffer:
        #     self.point_buffer = self.point_buffer[-self.max_point_buffer:]

        # # =========================
        # # REFIT PLANE (LOCAL MANIFOLD)
        # # =========================
        # points_np = np.array(self.point_buffer)

        # self.plane_normal, self.plane_centroid = self.fit_plane(points_np)

        # # =========================
        # # YAW ERROR
        # # =========================
        # # yaw_err = self.compute_yaw_error_2(self.plane_normal, self.q)

        # BUFFER UPDATE
        self.point_buffer.extend(pts_w.tolist())
        if len(self.point_buffer) > self.max_point_buffer:
            self.point_buffer = self.point_buffer[-self.max_point_buffer:]

        self.plane_normal, self.plane_centroid = self.fit_plane(np.array(self.point_buffer))

        # what is the point of projecting the points onto the plane?
        # in order to update : according to the latest IMU data, where is the plane?

        # For more stable, derive the yaw error from DVL AHRS' difference with latest moving window of hull point plane rather than current frame wall points (may be unstable)
        yaw_err = self.compute_yaw_error_2(self.plane_normal, self.q)
        dist = self.compute_distance_2(beams, yaw_err)

        # self.publish_state(yaw_err, dist)

        # Publish point cloud
        # self.publish_pointcloud(self.pc_buffer, current_points=world_points)

        # self.ongoing = False

        return dist, yaw_err, points

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
        if self.is_stationary and speed < MIN_SPEED: # TODO tune
            reject = True

        # (B) velocity clamp
        MAX_SPEED = 1.0  # # TODO tune
        if speed > MAX_SPEED:
            v_body = v_body * (MAX_SPEED / speed)

        # (C) beam consistency (variance) # turn off for now
        # if valid_beams >= 3:
        #     if np.std(b) > 0.2:  # tune
        #         reject = True

        # Too low, reject.
        if speed < 0.04: # < 4 cm/s # TODO tune . ideally already captured by self.is_stationary
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
        if self.plane_normal is not None:
            n = self.plane_normal / np.linalg.norm(self.plane_normal)
            v_world = v_world - np.dot(v_world, n) * n

        # =========================
        # 7. INTEGRATE POSITION
        # =========================
        if not hasattr(self, "prev_time"):
            self.prev_time = self.get_clock().now()
            return self.position, np.array([0, 0, 0])

        now = self.get_clock().now()

        MAX_DT = 0.5   # 500 ms (tune: 0.05–1.0) # TODO tune . prevent large jumps in position due to lag time in dvl data but high velocity.

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
        self.position = p_pred
        self.prev_velocity = v_world

        return p_pred

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
        THRESH = 0.04   # tune (0.03–0.1 typical) # TODO tune

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
        # The alt in the VFR_HUD message in the machine is not the correct depth information. The depth information is in NAV_CONTROLLER_OUTPUT, but there is no. The alt_error field information in NAV_CONTROLLER_OUTPUT is also displayed in QGC, but the ALT_HUD mode can run normally and maintain depth :joy:. Why is this? Any impact?

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
        Only keep valid beams
        """
        valid_points = []
        world_points = []

        beam_dirs = self.get_beam_dirs()
        beam_dirs = dvl_to_ros_transform_beams(beam_dirs) # Critical fix : beam dirs need to be in ROS frame first

        for i, d_beam in enumerate(beams):
            d = d_beam.distance
            if d <= 0:
                continue
            if not d_beam.valid:
                self.get_logger().info(f"BEAM INVALID {i}")
                continue
            # else:
                # self.get_logger().info(f"VALID {i}")
            # if True:
                # self.get_logger().info(f"RSSI {d_beam.rssi} \t NSD {d_beam.nsd}")
            # p = d * beam_dirs[i] # wrong, before rotation
            # R_mount = self.get_mount_rotation()
            # R_mount = self.get_mount_rotation()
            
            R_imu = self.quat_to_rot(self.q)

            p_dvl = d * beam_dirs[i]

            # p_robot = R_ros_dvl @ p_dvl # Already transformed above in dvl_to_ros
            p_wall = R_imu @ p_dvl # not p_robot
            p_world = p_wall + self.position


            valid_points.append(p_wall)
            world_points.append(p_world)

        # if len(valid_points) < 3: # keep imperfect beam sets, let downstream decided
            # return None

        return np.array(valid_points), np.array(world_points)

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
    # INITIAL ALIGNMENT
    # =========================
    def initialize_reference(self):
        """
        Find best yaw that minimizes plane error
        """
        best_yaw = 0
        best_error = 1e9

        for yaw in np.linspace(-np.pi, np.pi, 60):
            all_pts = []

            for pts in self.init_buffer:
                # R = self.rotation_matrix(self.roll, self.pitch, yaw)
                R = self.rotation_matrix(
                    self.roll,    # 0.0
                    self.pitch,   # from DVL pitch
                    self.yaw      # from DVL roll
                )
                for p in pts:
                    all_pts.append(R @ p)

            all_pts = np.array(all_pts)

            normal, d, err = self.fit_plane(all_pts)

            if err < best_error:
                best_error = err
                best_yaw = yaw
                self.plane_normal = normal
                self.plane_d = d

        self.yaw_ref = best_yaw
        self.get_logger().info(f"Yaw reference initialized: {best_yaw:.3f}")

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

            d = beam.distance
            valid = beam.valid


            if d <= 0 or not valid:
                continue

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
        self.plane_normal = n
        self.plane_d = d
    
    def fit_plane(self, pts):

        if len(pts) < 3:
            return self.plane_normal, self.plane_centroid

        prev_normal = self.plane_normal

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
            return self.plane_normal, self.plane_centroid

        prev_normal = self.plane_normal

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
        if hasattr(self, "plane_normal") and self.plane_normal is not None:
            if np.dot(normal, self.plane_normal) < 0:
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

    def publish_state(self, yaw_error, dist):
        # Publish distance marker
        self.publish_distance_marker(dist)

        self.get_logger().info(f'Dist : {dist:.2f}, Yaw_err : {math.degrees(yaw_error):.2f}')

        msg = Float32MultiArray()
        msg.data = [dist, yaw_error]
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

        if self.plane_normal is None:
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
        p1 = p0 - dist * self.plane_normal

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