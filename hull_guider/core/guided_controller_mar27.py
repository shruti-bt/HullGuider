#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import time

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
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

    def publish_velocity(self, v_yours, yaw_rate):

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        # ALL frames are already in ROS / MAVROS frame

        msg.twist.linear.x = v_yours[0]
        msg.twist.linear.y = v_yours[1]
        msg.twist.linear.z = v_yours[2]

        msg.twist.angular.z = yaw_rate # Yaw rate is already negatived.

        self.vel_pub.publish(msg)

    # =========================
    # CONTROL LOOP
    # =========================
    def control_loop(self):


        distance_threshold = 0.8


        # Ensure GUIDED mode
        if self.current_mode != "GUIDED":
            self.set_guided_mode()
            return

        # ---------------- SIMPLE CONTROL ----------------
        # Replace with your real control logic

        # Distance control → forward/back
        k_dist = 0.5
        surge = -k_dist * (self.distance - distance_threshold)

        # Yaw control
        k_yaw = 0.8
        yaw_rate = -1.0 * k_yaw * self.yaw_error # Negatived. 

        lateral_speed = 0.3  # slow scan (tune: 0.05–0.15)

        v_yours = np.array([
            0.0,             # x (up)
            lateral_speed,   # y (right)  ✅
            surge            # z (forward)
        ])

        # ---------------- SCALING ----------------
        v_yours = np.clip(v_yours, -0.3, 0.3)
        yaw_rate = np.clip(yaw_rate, -0.3, 0.3)

        # Smooth (important for 5 Hz DVL)
        alpha = 0.3
        self.last_velocity = alpha * v_yours + (1 - alpha) * self.last_velocity
        self.last_yaw_rate = alpha * yaw_rate + (1 - alpha) * self.last_yaw_rate

        # ---------------- PUBLISH ----------------
        self.publish_velocity(self.last_velocity, self.last_yaw_rate)

        self.get_logger().info(
            f"vel={self.last_velocity} yaw_rate={self.last_yaw_rate:.2f}"
        )


def main():
    rclpy.init()
    node = DVLGuidedController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()