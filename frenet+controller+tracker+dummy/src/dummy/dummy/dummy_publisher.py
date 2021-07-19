#a few of the extra imports need to be cleaned up later

import rclpy
import sys
from rclpy import publisher
from rclpy.node import Node
from std_msgs.msg import String 
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Int8
from std_msgs.msg import UInt32
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32


def main(args=None):
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('demo_publisher')
    ground_truth_publisher = node.create_publisher(Odometry, 'base_pose_ground_truth' ,10)
    occupancy_grid_publisher = node.create_publisher(OccupancyGrid,'costmap',10)
    footprint_publisher = node.create_publisher(PolygonStamped,'footprint',10)

    msg_3 = Odometry()
    grid = OccupancyGrid()
    footprint = PolygonStamped()

    #just dummy values for base pose ground truth
    
    msg_3.header.frame_id = "odom"
    current_time = node.get_clock().now().to_msg()

    msg_3.pose.pose.position.x = 2.0
    msg_3.pose.pose.position.y = 3.0
    msg_3.pose.pose.position.z = 0.0

    msg_3.pose.pose.orientation.x = 0.0
    msg_3.pose.pose.orientation.y = 0.0
    msg_3.pose.pose.orientation.z = 0.0

    #just dummy values for occupancy grid

    grid._header._stamp = current_time
    grid._header._frame_id = 'demo'

    grid._info._map_load_time = current_time
    grid._info._resolution = 1.0
    grid._info._width = 100
    grid._info._height = 100
    grid._info._origin.position._x = 0.0
    grid._info._origin.position._y = 0.0
    grid._info._origin.position._z = 0.0
    grid._info._origin.orientation._x = 0.0
    grid._info._origin.orientation._y = 0.0
    grid._info._origin.orientation._z = 0.0
    grid._data = [0 for i in range(100*100)]
    grid._data[0] = 100
    grid._data[1010] = 100
    grid._data[2020] = 100
    grid._data[3030] = 100
    grid._data[4040] = 100
    grid._data[5050] = 100
    grid._data[6060] = 100
    grid._data[7070] = 100
    grid._data[9092] = 100

    #just dummy values for footprint

    triangle1 = Point32()
    triangle2 = Point32()
    triangle3 = Point32()

    triangle1._x = 0.0
    triangle1._y = 0.0 
    triangle1._z = 0.0

    triangle2._x = 2.0
    triangle2._y = 2.0 
    triangle2._z = 0.0

    triangle3._x = -2.0
    triangle3._y = -2.0
    triangle3._z = 0.0

    footprint._polygon._points = [triangle3,triangle1,triangle1]

    footprint._header._stamp = current_time
    footprint._header._frame_id = 'demo'
    

    def timer_callback():
        ground_truth_publisher.publish(msg_3)
        occupancy_grid_publisher.publish(grid)
        footprint_publisher.publish(footprint)

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