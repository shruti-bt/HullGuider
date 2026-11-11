#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import time

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import PositionTarget, State
from std_msgs.msg import Float32MultiArray
from dvl_msgs.msg import DVLDR

from hull_guider.utils.transformations import dvl_to_ros_transform

def euler_to_quat(roll, pitch, yaw):
    cr = np.cos(roll/2)
    sr = np.sin(roll/2)
    cp = np.cos(pitch/2)
    sp = np.sin(pitch/2)
    cy = np.cos(yaw/2)
    sy = np.sin(yaw/2)

    return np.array([
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy
    ])

class DVLGuidedController(Node):

    def __init__(self):
        super().__init__('dvl_guided_controller')

        # ---------------- STATE ----------------
        self.current_mode = ""
        self.last_velocity = np.zeros(3)
        self.last_yaw_rate = 0.0

        # Example placeholders (replace with your estimator outputs)
        self.yaw_error = 0.0
        self.distance = 0.0
        self.distance_valid = False      # 5th element of /hull/state
        self.have_distance = False       # filtered_distance initialised from a real reading


        # R3.2 and R3.3
        self.filtered_normal = np.array([-1.0, 0.0, 0.0])
        self.filtered_distance = 0.0

        self.alpha_normal = 0.15 # Since the update is 20 Hz targeted, this does not need to be high. This is used for smooth motion interpolation # TODO Tune this
        self.alpha_dist = 0.15

        self.distance_threshold = 0.8 # TODO replace with dynamic parameter or ros parameter. The target distance. (there should be a range)

        self.distance_deadband = 0.15 # meter

        self.tangent_dampen_factor = 0.3 # 30% of planned speed
        self.too_far_distance = 0.8 # meters beyond target 

        self.yaw_slow_threshold = np.deg2rad(45) # TODO tune / parametrize. keep at 45 degrees just like in hull node

        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(
            DVLDR,
            "/dvl/position",
            self.dvl_position_callback,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            "/hull/state",
            self.hull_state_callback,
            10
        )

        self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            10
        )

        # ---------------- PUBLISHERS ----------------
        # self.vision_pub = self.create_publisher(
        #     PoseStamped,
        #     "/mavros/vision_pose/pose",
        #     10
        # )

        self.vel_pub = self.create_publisher(
            TwistStamped,
            "/mavros/setpoint_velocity/cmd_vel",
            10
        )

        self.sp_pub = self.create_publisher(
            PositionTarget,
            "/mavros/setpoint_raw/local",
            10
        )

        # ---------------- SERVICES ----------------
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # ---------------- TIMER ----------------
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("DVL Guided Controller Initialized")

    # =========================
    # CALLBACKS
    # =========================
    def state_callback(self, msg):
        self.current_mode = msg.mode

    def dvl_position_callback(self, msg):
        """
        Replace with actual message fields later
        """
        position = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z,
        ])

        # Example:
        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw

        q_xyzw = euler_to_quat(roll, pitch, yaw)
        q_ros, t_ros = dvl_to_ros_transform(q_xyzw, position)
        # ---------------- PUBLISH ----------------
        # self.publish_vision_pose(t_ros, q_ros)
        

    def dvl_position_callback_old(self, msg):

        # ---------------- RAW POSITION ----------------
        p_dvl = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z
        ])

        # ---------------- ORIENTATION ----------------
        # DVL rotation
        R_dvl = self.rotation_matrix(msg.roll, msg.pitch, msg.yaw)

        R_mount = self.get_mount_correction()

        # ---------------- TRANSFORM ----------------
        # Body frame
        R_body = R_dvl @ R_mount

        # Position
        p_body = R_mount @ p_dvl

        # ---------------- CONVERT TO MAVROS ENU ----------------
        # Convert YOUR frame → MAVROS ENU
        R_conv = np.array([
            [0, 0, 1],
            [0, -1, 0],
            [1, 0, 0]
        ])

        R_mav = R_conv @ R_body

        # Quaternion for MAVROS
        q = self.rot_to_quat(R_mav)

        p_mav = np.array([
            p_body[2],      # forward
            -p_body[1],     # right → left
            p_body[0]       # up
        ])

        # ---------------- PUBLISH ----------------
        self.publish_vision_pose(p_mav, q)

    def hull_state_callback(self, msg):
        """
        Expect:
        msg.data = [distance, yaw_error]
        """
        self.distance = msg.data[0]
        self.yaw_error = msg.data[1]
        # Older hull nodes publish 4 values (no validity flag): treat as valid
        self.distance_valid = len(msg.data) < 5 or msg.data[4] > 0.5

        # R2 addition: stable plane normal (global frame)
        self.plane_normal = np.array([
            msg.data[2],
            msg.data[3],
            0.0
        ])

    # ======================
    # TRANSFORMS
    # ======================

    def get_mount_correction(self):
        theta = -np.pi / 2  # -90 degrees

        c = np.cos(theta)
        s = np.sin(theta)

        return np.array([
            [ c, 0,  s],
            [ 0, 1,  0],
            [-s, 0,  c]
        ])
    
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

    # =========================
    # MAVROS PIPELINE
    # =========================
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

    def set_guided_mode(self):
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            return

        req = SetMode.Request()
        req.custom_mode = "GUIDED"

        self.set_mode_client.call_async(req)

    def publish_setpoint(self, v_world, yaw):

        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()

        # =========================
        # GLOBAL FRAME
        # =========================
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # =========================
        # USE velocity + yaw
        # =========================
        msg.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        # No conversion (Use ENU, automatically converted to NED)
        vx, vy, vz = self.decompose(v_world)

        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz

        msg.yaw = yaw

        self.sp_pub.publish(msg)

    def decompose(self, v):
        return np.array([
            v[0],
            v[1],
            v[2]
        ])

    def enu_to_ned(self, v):
        return np.array([
            v[1],   # x_ned
            v[0],   # y_ned
            -v[2]   # z_ned
        ])

    # =========================
    # CONTROL LOOP
    # =========================

    def control_loop(self):
        # =========================
        # VALIDATE NORMAL
        # =========================
        if np.linalg.norm(self.plane_normal[:2]) < 1e-3:
            self.get_logger().warn("PLANE NORMAL IS INVALID!!")
            return

        n = self.plane_normal / np.linalg.norm(self.plane_normal)

        # =========================
        # SMOOTHING (controller responsibility)
        # =========================

        # R2.2 filter plane normal
        self.filtered_normal = (
            self.alpha_normal * n + (1 - self.alpha_normal) * self.filtered_normal
        )
        self.filtered_normal /= np.linalg.norm(self.filtered_normal)

        # No valid distance (no beam on the hull for a while, or none yet): hold position
        if not self.distance_valid:
            self.get_logger().warn("Hull distance invalid: holding", throttle_duration_sec=1.0)
            self.last_velocity = np.zeros(3)
            yaw_hold = np.arctan2(-self.filtered_normal[1], -self.filtered_normal[0])
            self.publish_setpoint(self.last_velocity, yaw_hold)
            return

        # R2.3 filter distance to plane (start from the first real reading, not from 0)
        if not self.have_distance:
            self.filtered_distance = self.distance
            self.have_distance = True
        self.filtered_distance = (
            self.alpha_dist * self.distance +
            (1 - self.alpha_dist) * self.filtered_distance
        )

        n = self.filtered_normal
        d = self.filtered_distance

        # =========================
        # TANGENT DIRECTION
        # =========================
        z_axis = np.array([0, 0, 1])
        t = np.cross(z_axis, n)
        t /= np.linalg.norm(t)

        # Prevent abrupt shifts in tangent direction # TODO note : this shouldn't happen if hull state plane normal is always TOWARDS robot. positive thrust in the tangent direction should lead the t to the right
        # if hasattr(self, "prev_t"):
        #     if np.dot(t, self.prev_t) < 0:
        #         t = -t

        self.prev_t = t

        # =========================
        # YAW TARGET
        # =========================
        yaw_target = np.arctan2(-n[1], -n[0])  # ArduSub handles yaw correction smoothing internally, no issue

        # =========================
        # CONTROL GAINS
        # =========================
        k_dist = 0.5 # proportional gain of normal component # TODO tune
        k_tangent = 0.3 # tangential speed # TODO tune 

        # =========================
        # NORMAL CONTROL
        # =========================

        # Simple version
        # v_normal = -k_dist * (d - self.distance_threshold) * n

        # =========================
        # NORMAL CONTROL (WITH DEADBAND)
        # =========================
        # error = d - self.distance_threshold

        # # deadband = 0.15  # meters 
        # deadband = self.distance_deadband

        # if abs(error) < deadband:
        #     scaled_error = 0.0
        # else:
        #     scaled_error = np.sign(error) * (abs(error) - deadband)

        # v_normal = -k_dist * scaled_error * n

        # =========================
        # NORMAL CONTROL (DEADBAND + DAMPING)
        # =========================
        error = d - self.distance_threshold

        deadband = self.distance_deadband

        # --- soft deadband ---
        if abs(error) < deadband:
            scaled_error = 0.0
        else:
            scaled_error = np.sign(error) * (abs(error) - deadband)

        # --- previous velocity along normal (projection) ---
        v_prev_n = np.dot(self.last_velocity, n)

        # --- gains ---
        k_p = k_dist          # keep your existing gain
        k_d = 0.8             # damping gain (tune: 0.3–1.2)

        # --- PD control ---
        v_normal_mag = -k_p * scaled_error - k_d * v_prev_n

        v_normal = v_normal_mag * n
        
        # =========================
        # LATERAL CONTROL (WITH YAW GATING)
        # =========================

        if abs(self.yaw_error) > self.yaw_slow_threshold:
            # R3.1
            # stop lateral movement if yaw unstable
            v_tangent = np.zeros(3)
        else:
            v_tangent = k_tangent * t

        # =========================
        # DEPTH CONTROL
        # =========================

        # TODO add depth control to keep the same depth . this shouldn't apply smoothing or use fast smoothing.

        # =========================
        # ADDITIONAL SAFETY (distance too high)
        # =========================
        if abs(d - self.distance_threshold) > self.too_far_distance:
            # R3.3
            # slow down lateral if distance too far from target
            v_tangent *= self.tangent_dampen_factor # 0.3 # TODO . tune. or stop completely

        # =========================
        # FINAL VELOCITY
        # =========================
        v_world = v_normal + v_tangent

        # =========================
        # SMOOTH OUTPUT
        # =========================
        
        # limits actuator jerk, stabilizes system response # particularly for example distance too high safety (jerky)
        alpha = 0.3
        self.last_velocity = alpha * v_world + (1 - alpha) * self.last_velocity

        # =========================
        # PUBLISH
        # =========================
        self.publish_setpoint(self.last_velocity, yaw_target)

    def control_loop2(self):


        # distance_threshold = 0.8


        # # Ensure GUIDED mode
        # if self.current_mode != "GUIDED":
        #     self.set_guided_mode()
        #     return

        # # ---------------- SIMPLE CONTROL ----------------
        # # Replace with your real control logic

        # # Distance control → forward/back
        # k_dist = 0.5
        # surge = -k_dist * (self.distance - distance_threshold)

        # # Yaw control
        # k_yaw = 0.8
        # yaw_rate = -1.0 * k_yaw * self.yaw_error # Negatived. 

        # lateral_speed = 0.3  # slow scan (tune: 0.05–0.15)

        # v_yours = np.array([
        #     0.0,             # x (up)
        #     lateral_speed,   # y (right)
        #     surge            # z (forward)
        # ])

        # # ---------------- SCALING ----------------
        # v_yours = np.clip(v_yours, -0.3, 0.3)
        # yaw_rate = np.clip(yaw_rate, -0.3, 0.3)

        # # Smooth (important for 5 Hz DVL)
        # alpha = 0.3
        # self.last_velocity = alpha * v_yours + (1 - alpha) * self.last_velocity
        # self.last_yaw_rate = alpha * yaw_rate + (1 - alpha) * self.last_yaw_rate

        # # ---------------- PUBLISH ----------------
        # # self.publish_velocity(self.last_velocity, self.last_yaw_rate)

        # # =========================
        # # GLOBAL VELOCITY (KEY CHANGE)
        # # =========================

        # # plane_normal should already be in WORLD frame
        # n = self.plane_normal / np.linalg.norm(self.plane_normal)

        # # Tangential direction (wall-following)
        # # choose a consistent direction (e.g. cross with Z)
        # z_axis = np.array([0, 0, 1])
        # t = np.cross(z_axis, n)
        # t /= np.linalg.norm(t)

        # # -------------------------
        # # CONTROL
        # # -------------------------

        # k_dist = 0.5
        # k_tangent = 0.3

        # # normal correction (distance control)
        # v_normal = -k_dist * (self.distance - distance_threshold) * n

        # # tangential motion (scan)
        # v_tangent = k_tangent * t

        # v_world = v_normal + v_tangent

        # # -------------------------
        # # YAW TARGET (IMPORTANT)
        # # -------------------------

        # # face INTO wall (normal direction)
        # yaw_target = np.arctan2(-n[1], -n[0])

        # # -------------------------
        # # SMOOTHING
        # # -------------------------

        # alpha = 0.3
        # self.last_velocity = alpha * v_world + (1 - alpha) * self.last_velocity

        # # -------------------------
        # # PUBLISH
        # # -------------------------

        # self.publish_setpoint(self.last_velocity, yaw_target)

        # self.get_logger().info(
        #     f"vel={self.last_velocity} yaw_rate={self.last_yaw_rate:.2f}"
        # )
        return


def main():
    rclpy.init()
    node = DVLGuidedController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()