#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
import tf2_ros

# QoS for best‐effort on high‐rate topics
qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.BEST_EFFORT

class OdometryBridge(Node):
    def __init__(self):
        super().__init__('vehicle_odometry_bridge')

        # 1) Subscribe to PX4 VehicleOdometry
        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos
        )

        # 2) Publisher for nav_msgs/Odometry on /odom
        self.pub = self.create_publisher(Odometry, '/odom', 10)

        # 3) TF2 broadcaster for odom → base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def odom_callback(self, msg: VehicleOdometry):
        # Grab the current sim-time once
        now = self.get_clock().now().to_msg()

        # --- Build & publish Odometry ---
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position.x = float(msg.position[0])
        odom_msg.pose.pose.position.y = float(msg.position[1])
        odom_msg.pose.pose.position.z = float(msg.position[2])

        # Orientation
        q = msg.q
        odom_msg.pose.pose.orientation = Quaternion(
            x=float(q[0]),
            y=float(q[1]),
            z=float(q[2]),
            w=float(q[3])
        )

        # Linear velocity
        odom_msg.twist.twist.linear.x = float(msg.velocity[0])
        odom_msg.twist.twist.linear.y = float(msg.velocity[1])
        odom_msg.twist.twist.linear.z = float(msg.velocity[2])

        # Angular velocity (if present)
        if hasattr(msg, 'angular_velocity') and len(msg.angular_velocity) >= 3:
            odom_msg.twist.twist.angular.x = float(msg.angular_velocity[0])
            odom_msg.twist.twist.angular.y = float(msg.angular_velocity[1])
            odom_msg.twist.twist.angular.z = float(msg.angular_velocity[2])

        self.pub.publish(odom_msg)

        # --- Broadcast TF with the same timestamp ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation        = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
