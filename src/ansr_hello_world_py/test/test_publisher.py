import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from adk_node.msg import WaypointPath

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from math import sin,cos,pi
from tf_transformations import quaternion_from_euler 
from tf_transformations import euler_from_quaternion 
from tf_transformations import euler_from_matrix

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('sparse_waypoint_publisher')
        qos_profile = QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(WaypointPath, 'adk_node/input/waypoints', qos_profile)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        waypoint_msg = WaypointPath()
        
        pose_msg_array = []


        x = 128.0
        y = 100.0
        z = -10.0

        roll = 0
        pitch = 0
        yaw = 0

        q=quaternion_from_euler(roll*pi/180, pitch*pi/180, yaw*pi/180)


        pose_msg = PoseStamped()

        h = Header()
        h.stamp = self.get_clock().now().to_msg()

        pose_msg.header = h
        pose_msg.header.frame_id = "world_ned"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        pose_msg_array.append(pose_msg)

        waypoint_msg.path = pose_msg_array
        waypoint_msg.velocity = 1.0
        waypoint_msg.lookahead = -1.0
        waypoint_msg.adaptive_lookahead = 0.0
        # waypoint_msg.drive_train_type = "ForwardOnly"
        waypoint_msg.wait_on_last_task = False

        self.publisher_.publish(waypoint_msg)
        self.get_logger().info('Publishing Waypoints...')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
