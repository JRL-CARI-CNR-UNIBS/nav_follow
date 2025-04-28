#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
from tf2_ros import TransformBroadcaster, TransformListener, TransformException
from geometry_msgs.msg import TransformStamped
from time import time
import math
import tf_transformations

class VelocityBasedTFPublisher(Node):
    def __init__(self):
        super().__init__('velocity_based_tf_publisher')
        
        # Create a broadcaster for the transformation
        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to the cmd_vel topic
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/leader_simulation/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Set timeout (in seconds)
        self.timeout = 1.0
        self.last_received_time = time()

        # Initial velocity parameters (x, y, angular)
        self.linear_velocity_x = 0.0  # Forward/backward (x-axis)
        self.linear_velocity_y = 0.0  # Lateral (y-axis)
        self.angular_velocity_z = 0.0  # Rotation (z-axis)

        self.amplification_factor = 0.5  # Amplification factor for velocities
        
        # Initial position and orientation (0, 0, 0)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientation in radians
        
        # Timer to update the transformation
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Node for publishing TF based on cmd_vel started!")

        self.leader_frame = 'azrael/base_footprint'
        self.follower_frame = 'omron/base_footprint'
        self.base_frame = 'azrael/odom'


        self.initial_trasformation = None
        self.initialization_timer = self.create_timer(1, self.initialize)

           
    def initialize(self):
        try:
            # Ottieni la trasformazione tra i frame
            self.initial_transform = self.tf_buffer.lookup_transform(
                self.base_frame,       # from
                self.leader_frame,     # to
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Estrai la traslazione (x, y, z) e la rotazione (quaternion)
            translation = self.initial_transform.transform.translation
            rotation = self.initial_transform.transform.rotation

            self.initial_trasformation = tf_transformations.compose_matrix(
                translate=[translation.x, translation.y, translation.z],  # Passa i valori di traslazione separatamente
                angles=tf_transformations.euler_from_quaternion(
                    [rotation.x, rotation.y, rotation.z, rotation.w]  # Passa il quaternion per la rotazione
                )
            )
            self.destroy_timer(self.initialization_timer)
        
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.leader_frame} to {self.base_frame}: {ex}')

    def cmd_vel_callback(self, msg: Twist):
        # Update the linear velocities (x, y) and angular velocity (z)
        self.linear_velocity_x = msg.linear.x * self.amplification_factor
        self.linear_velocity_y = msg.linear.y * self.amplification_factor
        self.angular_velocity_z = msg.angular.z * self.amplification_factor
        
        # Update the timestamp of the last received message
        self.last_received_time = time()

        self.get_logger().info(f"Received cmd_vel: linear_x={self.linear_velocity_x},linear_y={self.linear_velocity_y}, angular_z={self.angular_velocity_z}")  

    def timer_callback(self) -> None:
        # Calculate the time elapsed since the last cmd_vel message
        elapsed_time = time() - self.last_received_time
        
        # If the timeout has passed, set velocity to 0
        if elapsed_time > self.timeout:
            self.linear_velocity_x = 0.0
            self.linear_velocity_y = 0.0
            self.angular_velocity_z = 0.0
        
        # Integrate the velocities to calculate the new position and orientation
        self.integrate_velocity()

        # Publish the transformation
        self.publish_transform()

    def integrate_velocity(self):
        # Simple integration (Euler method) for linear and angular velocity
        dt = 0.1  # Time step for each iteration (0.1s)
        
        # Update the orientation (theta) by integrating the angular velocity
        self.theta += self.angular_velocity_z * dt
        
        # Normalize theta to the range [-pi, pi] to avoid large angles
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Update the position (x, y) by integrating the linear velocities
        self.x += self.linear_velocity_x * math.cos(self.theta) * dt - self.linear_velocity_y * math.sin(self.theta) * dt
        self.y += self.linear_velocity_x * math.sin(self.theta) * dt + self.linear_velocity_y * math.cos(self.theta) * dt

    def publish_transform(self):
        if self.initial_trasformation is None:
            return

        # Create the transformation between base_footprint and base_link
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.follower_frame
        
        # Translation based on integrated position (x, y)
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0  # No movement along z-axis
        
        # Rotation based on the integrated orientation (theta)
        q = self.quaternion_from_euler(0.0, 0.0, self.theta)
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        relative_mat = tf_transformations.compose_matrix(
                translate=[self.x, self.y, 0.0],  # Passa i valori di traslazione separatamente
                angles=tf_transformations.euler_from_quaternion(
                    q  # Passa il quaternion per la rotazione
                )
            )
        
        mat_to_send = relative_mat + self.initial_trasformation 

        quat = tf_transformations.quaternion_from_matrix(mat_to_send)
        tras = tf_transformations.translation_from_matrix(mat_to_send)


        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        t.transform.translation.x = tras[0]
        t.transform.translation.y = tras[1]
        t.transform.translation.z = tras[2]

        # Send the transformation
        self.br.sendTransform(t)
        self.get_logger().info(f"Published transform: x={self.x}, y={self.y}, theta={self.theta}")

    def quaternion_from_euler(self, roll, pitch, yaw):
        """Converts Euler angles (roll, pitch, yaw) to a quaternion."""
        qx = math.sin(roll / 2.0) * math.cos(pitch / 2.0) * math.cos(yaw / 2.0) - math.cos(roll / 2.0) * math.sin(pitch / 2.0) * math.sin(yaw / 2.0)
        qy = math.cos(roll / 2.0) * math.sin(pitch / 2.0) * math.cos(yaw / 2.0) + math.sin(roll / 2.0) * math.cos(pitch / 2.0) * math.sin(yaw / 2.0)
        qz = math.cos(roll / 2.0) * math.cos(pitch / 2.0) * math.sin(yaw / 2.0) - math.sin(roll / 2.0) * math.sin(pitch / 2.0) * math.cos(yaw / 2.0)
        qw = math.cos(roll / 2.0) * math.cos(pitch / 2.0) * math.cos(yaw / 2.0) + math.sin(roll / 2.0) * math.sin(pitch / 2.0) * math.sin(yaw / 2.0)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = VelocityBasedTFPublisher()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
