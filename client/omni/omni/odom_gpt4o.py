#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from geometry_msgs.msg import Twist, Pose2D, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import serial
import time
import math
from tf2_ros import TransformBroadcaster, TransformStamped

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value): #rpm
        error = setpoint - measured_value
        er = round((abs(error) / 4) / 4)
        if error < 0: er *= -1
        return er

    def firsttime_set(self, setpoint, wheelnumber):
        output_spd = abs(round(setpoint / 4))
        if setpoint < 0: output_spd *= -1
        print(f'fi {output_spd}')
        return output_spd

class KinModel(Node):
    def __init__(self):
        super().__init__('kinematic_model')
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.rpm_sub = self.create_subscription(Float32MultiArray, '/rpm_all', self.rpm_callback, 10)
        self.control_pub = self.create_publisher(Int32, '/wheel_control', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Parameters: adjust these to match your robot's dimensions
        self.r = 0.04  # wheel radius
        self.l = 0.31 / 2 # distance from the robot center to wheel along the length
        self.w = 0.22 / 2  # distance from the robot center to wheel along the width
        
        # Initial signals
        self.control_singnals_initial = np.zeros(4, dtype=np.int32)
        # Error signals
        self.error_signals = np.zeros(4, dtype=np.int32)
        
        # Kinematic matrix
        self.M = np.array([[-self.l - self.w, 1, -1],
                           [self.l + self.w, 1, 1],
                        #    [self.l + self.w, 1, -1],
                           [-self.l - self.w, 1, 1],
                        #    [-self.l - self.w, 1, 1]], dtype=np.float32)
                           [self.l + self.w, 1, -1]], dtype=np.float32)
        # self.M = np.array([[-self.l - self.w, 1, -1],
        #                    [self.l + self.w, 1, 1],
        #                    [self.l + self.w, 1, -1],
        #                 #    [-self.l - self.w, 1, 1],
        #                    [-self.l - self.w, 1, 1]], dtype=np.float32)
        #                 #    [self.l + self.w, 1, -1]], dtype=np.float32)
                        
        
        self.twist = np.zeros(3, dtype=np.float32)
        self.desired_rpm = np.zeros(4, dtype=np.float32)
        self.measured_rpm = np.zeros(4, dtype=np.float32)
        
        # PID controllers for each wheel
        self.pids = [PIDController(1.5, 0.05, 0.001) for _ in range(4)]
        
        # Control signals
        self.control_signal = Int32()
        
        # Odometry
        self.tick_per_rev = 330
        self.robot_pose = Pose2D()  # x, y, yaw
        self.prev_time = self.get_clock().now()
        
        # TF2 broadcaster for odometry frame
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Serial port
        self.s = serial.Serial('/dev/ttyUSB2', 9600, timeout=1)
        self.ss = serial.Serial('/dev/ttyUSB3', 9600, timeout=1)

        # Timer for publishing odometry
        self.create_timer(0.1, self.publish_odometry)

    def cmd_vel_callback(self, msg):
        self.twist[0] = msg.angular.z
        self.twist[1] = msg.linear.x
        self.twist[2] = msg.linear.y
        self.compute_desired_rpm()

    def rpm_callback(self, msg):
        self.measured_rpm = np.array(msg.data, dtype=np.float32)
        print(f'rpm_cal: {self.measured_rpm}')
        self.apply_pid_control()

    def compute_desired_rpm(self):
        wheel_speeds = np.dot(self.M, self.twist) / self.r  # rad/s
        
        self.desired_rpm = wheel_speeds * 60 / (2 * np.pi)  # convert rad/s to RPM
        self.control_singnals_initial = [pid.firsttime_set(self.desired_rpm[i], i + 1) for i, pid in enumerate(self.pids)]

    def apply_pid_control(self):
        error_signals = [pid.compute(self.desired_rpm[i], self.measured_rpm[i]) for i, pid in enumerate(self.pids)]
        control_signals = self.convert_rpm_to_control(error_signals)
        self.publish_and_control_signals(control_signals)

    def convert_rpm_to_control(self, error_signals):
        control_signals = self.control_singnals_initial.copy()
        print(f'initial {control_signals}')
        print(f'err {error_signals}')
        
        for i, spd in enumerate(control_signals):
            spd += error_signals[i]
            if i in [0, 2]:  # Left wheels (Front and Rear)
                if spd > 0:
                    control_signals[i] = max((min(int(spd), 63)), 0)
                else:
                    control_signals[i] = max((min(int(-spd) + 64, 127)), 64)
            elif i in [1, 3]:  # Right wheels (Front and Rear)
                if spd > 0:
                    control_signals[i] = max((min(int(spd) + 128, 191)), 128)
                else:
                    control_signals[i] = max((min(int(-spd) + 192, 255)), 192)
        
        return control_signals

    def publish_and_control_signals(self, control_signals):
        packetFL = bytearray([control_signals[0]])  # FL
        packetFR = bytearray([control_signals[1]])  # FR
        packetBL = bytearray([control_signals[2]])  # RL
        packetBR = bytearray([control_signals[3]])  # RR
        self.ss.write(packetFL)
        self.ss.write(packetFR)
        self.s.write(packetBL)
        self.s.write(packetBR)
            
        print(f'fi {self.control_singnals_initial}')
        print(f'desired: {self.desired_rpm}')
        print(f"Control Signals: {control_signals}")
        print()

    def compute_odometry(self):
        # Calculate the time elapsed
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time

        # Calculate distances travelled by each wheel (in meters)
        # wheel_circumference = (2 * np.pi * self.r) / self.tick_per_rev
        # wheel_distances = (self.measured_rpm / 60) * wheel_circumference * dt
        # print(wheel_distances)
        wheel_distances = (self.measured_rpm / 60) * (2 * np.pi * self.r) * dt
        
        # Using kinematic model to update the pose
        delta_x = 0
        delta_y = 0
        delta_theta = 0

        for i in range(4):
            x_dot = wheel_distances[i] * (self.M[i][1] * math.cos(self.robot_pose.theta) - self.M[i][2] * math.sin(self.robot_pose.theta))
            y_dot = wheel_distances[i] * (self.M[i][1] * math.sin(self.robot_pose.theta) + self.M[i][2] * math.cos(self.robot_pose.theta))
            delta_x += x_dot
            delta_y += y_dot
            delta_theta += wheel_distances[i] * self.M[i][0] / (self.l + self.w)

        self.robot_pose.x += delta_x / 4
        self.robot_pose.y += delta_y / 4
        # self.robot_pose.theta += delta_theta / 4
        self.robot_pose.theta += delta_theta / 1.2
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    def publish_odometry(self):
        self.compute_odometry()

        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set the position
        odom_msg.pose.pose.position.x = self.robot_pose.x
        odom_msg.pose.pose.position.y = self.robot_pose.y
        odom_msg.pose.pose.position.z = 0.0
        quaternion = self.euler_to_quaternion(0, 0, self.robot_pose.theta)
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        # Set the velocity
        odom_msg.twist.twist.linear.x = float(self.twist[1])
        odom_msg.twist.twist.linear.y = float(self.twist[2])
        odom_msg.twist.twist.angular.z = float(self.twist[0])

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)
        
        # Broadcast TF transform
        tfs = TransformStamped()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id = 'odom'
        tfs.child_frame_id = 'base_link'
        tfs.transform.translation.x = self.robot_pose.x
        tfs.transform.translation.y = self.robot_pose.y
        tfs.transform.translation.z = 0.0
        tfs.transform.rotation.x = quaternion[0]
        tfs.transform.rotation.y = quaternion[1]
        tfs.transform.rotation.z = quaternion[2]
        tfs.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(tfs)
        
def main(args=None):
    print('starting')
    rclpy.init(args=args)
    kin_model = KinModel()
    rclpy.spin(kin_model)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
