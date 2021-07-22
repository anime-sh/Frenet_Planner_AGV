import sys
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import _thread
from prius_msgs.msg import Control


# Node name        - controls
# Published topic  - pid_output (Twist)
# Subscriber topic - cmd_vel, cmd_delta, base_pose_ground_truth

gear_stat = "F"
tar_vel = 0.0  # target velocity
tar_omega = 0.0  # target omega
active_vel = 0.0  # current velocity
error_sum = 0.0
prev_error = 0.0
error_diff = 0.0
output = 0.0
wheelbase = 1.958  # in meters
radius = 0.0  # radius of curvature of path
steering_angle = 0.0  # steering angle
default_depth = 10
tar_delta = 0.0

kp = 8.0  # proprtional gain
ki = 2.0  # integral gain
kd = 0.2  # derivative gain

yp = 20.0  # kp gain
yi = 0.5  # ki gain
yd = 0.1  # kd gain

acc_thershold = 0.0  # threshold for acceleration
brake_threshold = 20.0  # threshold for braking

tar_vel = 0.0
tar_omega = 0.0


def prius_pub(data):
    '''
    publishes the velocity and steering angle
    published on topic : ackermann_cmd_topic
    '''
    global prius_vel
    prius_vel = Control()

    if(data.linear.x > 0.0):
        prius_vel.throttle = data.linear.x / 100.0
        prius_vel.brake = 0.0
        print("acc")
        print(prius_vel.throttle)

    if(data.linear.x < 0.0):
        prius_vel.brake = -data.linear.x / 100.0
        prius_vel.throttle = 0.0
        print("brake")
        print(prius_vel.brake)

    prius_vel.steer = data.angular.z / 30.0

    pub.publish(prius_vel)


def callback_feedback(data):
    '''
    Applies adaptive PID to velcity input from odom readings and publishes.
    :params data [Odometry]
    :params output [Twist]
    :params plot [Twist]
    '''
    global active_vel
    global tar_vel
    global tar_omega
    global wheelbase
    global error_sum
    global error
    global error_diff
    global output
    global i
    global flag
    global kp
    global ki
    global kd
    global pub
    global prev_error
    global gear_stat
    global acc_thershold
    global brake_threshold
    global act_velocity
    global yp
    global yi
    global yd

    siny = 2.0 * (data.pose.pose.orientation.w * data.pose.pose.orientation.z
                + data.pose.pose.orientation.x * data.pose.pose.orientation.y)
    cosy = 1.0 - 2.0 * (data.pose.pose.orientation.y *
        data.pose.pose.orientation.y +
        data.pose.pose.orientation.z *
        data.pose.pose.orientation.z)
    yaw = math.atan2(siny, cosy)

    last_recorded_vel = (data.twist.twist.linear.x * math.cos(yaw) +
        data.twist.twist.linear.y * math.sin(yaw))

    active_vel = last_recorded_vel

    plot = Twist()
    output = Twist()

    error = tar_vel - active_vel
    error_sum += error
    error_diff = error - prev_error
    prev_error = error
    if error == 0.0:
        if tar_vel == 0.0:
            output.linear.x = 0.0
        else:
            output.linear.x = output.linear.x - 5.0
    # updating kp, ki, kd using MIT rule
    kp = kp + yp * error * error
    ki = ki + yi * error * error_sum
    kd = kd + yd * error * error_diff

    node.get_logger().info("kp is : {f}".format(f=kp))
    node.get_logger().info("ki is : {f}".format(f=ki))
    node.get_logger().info("kd is : {f}".format(f=kd))

    # PID on velocity with updated parameters
    if error > 0.01:
        output.linear.x = (kp * error + ki * error_sum + kd * error_diff)
    if error < -0.01:
        output.linear.x = ((kp * error + ki * error_sum + kd * error_diff) -
            brake_threshold)

    plot.linear.x = tar_vel
    plot.linear.y = active_vel
    plot.linear.z = tar_vel - active_vel  # error term
    # thresholding the forward velocity
    if output.linear.x > 100:
        output.linear.x = 100.0
    if output.linear.x < -100:
        output.linear.x = -100.0
    # thresholding the angle
    output.angular.z = min(30.0, max(-30.0, tar_delta))

    #rospy.loginfo("linear velocity : %f", plot.linear.y)
    #rospy.loginfo("target linear velocity : %f", plot.linear.x)
    #rospy.loginfo("delta : %f", output.angular.z)

    node.get_logger().info("linear velocity : {f}".format(f=plot.linear.y))
    node.get_logger().info("target linear velocity : {f}".format(f=plot.linear.x))
    node.get_logger().info("delta : {f}".format(f=output.angular.z))
    prius_pub(output)
    pub1.publish(plot)


def callback_cmd_vel(data):
    """
    Assigns the value of velocity from topic cmd_vel to tar_vel
    :param tar_vel: (float)
    :param data: (twist)
    """
    global tar_vel
    tar_vel = data.linear.x


def callback_delta(data):
    """
    Assigns the value from subscribed topic to the variable tar_delta
    :param tar_delta: (float)
    :param data: (twist)
    """
    global tar_delta
    tar_delta = data.angular.z


def start():
    global pub
    global pub1
    global node
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('controls')

    ackermann_cmd_topic = node.declare_parameter('~ackermann_cmd_topic', '/prius').value
    pub = node.create_publisher(Control,ackermann_cmd_topic,default_depth)
    pub1 = node.create_publisher(Twist,'plot',default_depth)

    node.create_subscription(Twist,'cmd_vel',callback_cmd_vel,default_depth)
    node.create_subscription(Twist, 'cmd_delta',callback_delta,default_depth)
    node.create_subscription(Odometry, 'base_pose_ground_truth',callback_feedback,default_depth)
    rclpy.spin(node)



if __name__ == '__main__':
    start()

