#!/usr/bin/env python3
"""Republish the bridged /odom message as the odom -> base_link transform.

Why this node exists at all:
    Nav2 needs a complete TF chain  map -> odom -> base_link.
      * map -> odom      is published by AMCL (localisation correction)
      * odom -> base_link is the raw wheel odometry - this node
      * base_link -> lidar_link etc. come from robot_state_publisher
    Gazebo's DiffDrive plugin can publish that middle edge itself, but doing
    it here keeps exactly ONE publisher on that edge and makes the data flow
    visible in the ROS graph, which is easier to explain and to debug.

Two details that matter:
    1. The transform copies msg.header.stamp rather than calling now().
       Odometry is stamped with simulation time; stamping the TF with wall
       time would make every lookup fail with an extrapolation error.
    2. use_sim_time must be true (the launch file sets it), otherwise the
       node's own clock disagrees with the stamps it is forwarding.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class OdomTFBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        topic = self.get_parameter('odom_topic').value

        self.tf_broadcaster = TransformBroadcaster(self)

        # BEST_EFFORT matches how the ros_gz bridge publishes sensor-ish data
        # and avoids a silent QoS mismatch where the subscription connects to
        # nothing.  If /odom is flowing but this node is quiet, this line is
        # the first thing to check.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Odometry, topic,
                                            self.odom_callback, qos)

        self.count = 0
        self.get_logger().info(
            f'publishing {self.odom_frame} -> {self.base_frame} from {topic}')

    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp          # simulation time, not now()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        self.count += 1
        if self.count == 1:
            self.get_logger().info('first odometry received - TF chain is live')


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
