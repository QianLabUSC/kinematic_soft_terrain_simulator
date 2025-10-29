#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


class FullyActuated2DDriveSim(Node):
    def __init__(self):
        super().__init__('fully_actuated_2d_drive_sim')

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 20.0 * math.pi / 180.0  # Heading angle (rad)

        # Velocity commands (local frame)
        self.linear_x = 0.0  # Forward/backward velocity
        self.linear_y = 0.0  # Left/right velocity (sideways)
        self.angular_velocity = 0.0  # Rotational velocity

        # Subscriber to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'spirit/ghost_trot_control',
            self.cmd_vel_callback,
            10
        )

        # Publisher for pose (geometry_msgs/Pose)
        self.pose_pub = self.create_publisher(Pose, 'spirit/current_pose', 10)

        # TF broadcaster for map -> base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for simulation updates (10 Hz)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.update_pose)

        self.get_logger().info('Drive sim started')

    def cmd_vel_callback(self, msg: Twist):
        """Callback to receive velocity commands in robot's local frame."""
        self.linear_x = msg.linear.x      # Forward/backward
        self.linear_y = msg.linear.y      # Left/right (sideways)
        self.angular_velocity = msg.angular.z  # Rotation

    def update_pose(self):
        """Update the robot pose using fully actuated 2D kinematics."""
        dt = self.timer_period

        # Transform local velocities to global coordinate frame
        # Robot can move in any direction relative to its current orientation
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)
        
        # Global velocities from local velocities
        global_vx = self.linear_x * cos_theta - self.linear_y * sin_theta
        global_vy = self.linear_x * sin_theta + self.linear_y * cos_theta
        
        # Update position and orientation
        self.x += global_vx * dt
        self.y += global_vy * dt
        self.theta += self.angular_velocity * dt

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish the pose
        pose_msg = Pose()
        pose_msg.position = Point(x=self.x, y=self.y, z=0.0)

        # Quaternion from yaw (theta)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        pose_msg.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f'Pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')



def main(args=None):
    rclpy.init(args=args)
    node = FullyActuated2DDriveSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Fully Actuated 2D Drive Simulator')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

