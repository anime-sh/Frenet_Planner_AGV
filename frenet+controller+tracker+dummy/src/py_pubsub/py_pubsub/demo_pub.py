import rclpy
import sys
from rclpy import publisher
from rclpy.node import Node
from std_msgs.msg import String 
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def main(args=None):
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('demo_publisher')
    vel_publisher = node.create_publisher(Twist,'cmd_vel',10)
    delta_publisher = node.create_publisher(Twist,'cmd_delta',10)
    ground_truth_publisher = node.create_publisher(Odometry, 'base_pose_ground_truth' ,10)

    msg_1 = Twist()
    msg_2 = Twist()
    msg_3 = Odometry()

    #just dummy values 

    msg_1.linear.x = 1.0
    msg_1.angular.z = 1.0

    msg_2.linear.x = 1.2
    msg_2.angular.y = 1.2
    
    msg_3.header.frame_id = "odom"
    current_time = node.get_clock().now().to_msg()

    msg_3.pose.pose.position.x = 2.0
    msg_3.pose.pose.position.y = 3.0
    msg_3.pose.pose.position.z = 0.0

    msg_3.pose.pose.orientation.x = 0.0

    def timer_callback():
        vel_publisher.publish(msg_1)
        delta_publisher.publish(msg_2)
        ground_truth_publisher.publish(msg_3)

    timer_period = 0.5  # seconds
    timer = node.create_timer(timer_period, timer_callback)

    rclpy.spin(node)

    # Destroy the timer attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




