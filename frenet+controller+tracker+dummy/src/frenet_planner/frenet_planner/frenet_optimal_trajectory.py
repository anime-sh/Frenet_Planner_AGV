"""

Frenet optimal trajectory generator

original author: Atsushi Sakai (@Atsushi_twi)
contributors: AGV MPC Team

Ref:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
(https://www.youtube.com/watch?v=Cj6tAQe7UCY)

"""
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
import rclpy.parameter 

# import messages
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32

# import via reference to ros2 package
from frenet_planner import params
from frenet_planner.frenet_helpers import *

show_animation = True
cost_count = 0
footprint_count = 0
odom_count = 0

# functionality imitated from cpp codebase
def initial_conditions_new(csp, global_s, global_x, global_y,global_R, global_yaw, path):
    global s0
    global c_speed
    global c_d
    global c_d_d
    global c_d_dd

    vx = params.odom.twist.twist.linear.x
    vy = params.odom.twist.twist.linear.y
    v = np.sqrt(vx * vx + vy * vy)

    find_nearest_in_global_path(global_x, global_y, 0, path)

    vec1=[0,0]
    vec2=[0,0]
    vec1[0] = params.odom.pose.pose.position.x - global_x[params.min_id]
    vec1[1] = params.odom.pose.pose.position.y - global_y[params.min_id]
    vec2[0] = global_x[params.min_id] - global_x[params.min_id + 1]
    vec2[1] = global_y[params.min_id] - global_y[params.min_id + 1]
    curl2D = vec1[0] * vec2[1] - vec2[0] * vec1[1]
    if (curl2D < 0):
        c_d = c_d * -1
    s0 = global_s[params.min_id]
    _,_,bot_yaw = euler_from_quaternion(params.odom.pose.pose.orientation.x,params.odom.pose.pose.orientation.y,params.odom.pose.pose.orientation.z,params.odom.pose.pose.orientation.w)
    g_path_yaw = global_yaw[params.min_id]
    delta_theta = bot_yaw - g_path_yaw
    c_d_d = v * np.sin(delta_theta)
    k_r = global_R[params.min_id]
    c_speed = v * np.cos(delta_theta) / (1 - k_r * c_d)
    c_d_dd = 0
    return params.min_id

# nearest in global path
# functionality imitated from cpp codebase
def find_nearest_in_global_path(global_x, global_y, flag,path):
    if (flag == 0):
        bot_x = params.odom.pose.pose.position.x
        bot_y = params.odom.pose.pose.position.y
    else:
        bot_x = path.get_x()[1]
        bot_y = path.get_y()[1]
    min_dis = params.FLT_MAX
    for i in range(len(global_x)):
        dis = calc_dis(global_x[i], global_y[i], bot_x, bot_y)
        if (dis < min_dis):
            params.min_id = i

# define publishers, subscribers and respective callback functions
class Frenet_Planner(Node):

    def __init__(self):
        super().__init__('frenet_planner')

        self.frenet_path = self.create_publisher(Path, '/frenet_path', 1)
        self.global_path = self.create_publisher(Path, '/global_path', 1)
        self.target_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(Odometry,'base_pose_ground_truth',self.odom_callback,10)
        self.odom_sub # prevent unused variable warning
        self.footprint_sub = self.create_subscription(PolygonStamped,'footprint',self.footprint_callback,10)
        self.footprint_sub # prevent unused variable warning
        self.costmap_sub = self.create_subscription(OccupancyGrid,'costmap',self.costmap_callback,1000)
        self.costmap_sub # prevent unused variable warning

        self.declare_parameters(
        namespace='',
        parameters=[
            ('K_J', 0.0),
            ('K_T', 0.0),
            ('K_D', 0.0),
            ('K_LAT', 0.0),
            ('K_LON', 0.0),
            ('MAX_SPEED', 0.0),
            ('MAX_ACCEL', 0.0),
            ('MAX_CURVATURE', 0.0),
            ('MAX_ROAD_WIDTH', 0.0),
            ('D_ROAD_W', 0.0),
            ('DT', 0.0),
            ('MAX_T', 0.0),
            ('MIN_T', 0.0),
            ('TARGET_SPEED', 0.0),
            ('D_T_S', 0.0),
            ('N_S_SAMPLE', 0.0),
            ('OBSTACLE_RADIUS', 0.0),
            ('MIN_LAT_VEL', 0.0),
            ('MAX_LAT_VEL', 0.0),
            ('D_D_NS', 0.0),
            ('W_X', [0.0,1.0]),
            ('W_Y', [0.0,1.0]),
            ('LOOKAHEAD_DIST', 0.0)
        ])
        params.K_J = self.get_parameter("K_J").get_parameter_value().double_value
        params.K_T = self.get_parameter("K_T").get_parameter_value().double_value
        params.K_D = self.get_parameter("K_D").get_parameter_value().double_value
        params.K_LAT = self.get_parameter("K_LAT").get_parameter_value().double_value
        params.K_LON = self.get_parameter("K_LON").get_parameter_value().double_value
        params.MAX_SPEED = self.get_parameter("MAX_SPEED").get_parameter_value().double_value
        params.MAX_ACCEL = self.get_parameter("MAX_ACCEL").get_parameter_value().double_value
        params.MAX_CURVATURE = self.get_parameter("MAX_CURVATURE").get_parameter_value().double_value
        params.MAX_ROAD_WIDTH = self.get_parameter("MAX_ROAD_WIDTH").get_parameter_value().double_value
        params.D_ROAD_W = self.get_parameter("D_ROAD_W").get_parameter_value().double_value
        params.DT = self.get_parameter("DT").get_parameter_value().double_value
        params.MAX_T = self.get_parameter("MAX_T").get_parameter_value().double_value
        params.MIN_T = self.get_parameter("MIN_T").get_parameter_value().double_value
        params.TARGET_SPEED = self.get_parameter("TARGET_SPEED").get_parameter_value().double_value
        params.D_T_S = self.get_parameter("D_T_S").get_parameter_value().double_value
        params.N_S_SAMPLE = self.get_parameter("N_S_SAMPLE").get_parameter_value().double_value
        params.OBSTACLE_RADIUS = self.get_parameter("OBSTACLE_RADIUS").get_parameter_value().double_value
        params.MIN_LAT_VEL=self.get_parameter("MIN_LAT_VEL").get_parameter_value().double_value
        params.MAX_LAT_VEL=self.get_parameter("MAX_LAT_VEL").get_parameter_value().double_value
        params.D_D_NS=self.get_parameter("D_D_NS").get_parameter_value().double_value
        params.W_X = self.get_parameter("W_X").get_parameter_value().double_array_value
        params.W_Y = self.get_parameter("W_Y").get_parameter_value().double_array_value
        params.LOOKAHEAD_DIST=self.get_parameter("LOOKAHEAD_DIST").get_parameter_value().double_value

    def odom_callback(self, msg):
        params.odom = msg

    def footprint_callback(self, msg):
        params.footprint = msg

    # convert occupancy grid to obstacle list
    def costmap_callback(self, msg):
        global cost_count
        cost_count+=1
        ob1=[]
        params.cmap=msg
        origin=Pose()
        origin=msg.info.origin
        for width in range(0,msg.info.width):
            for height in range(0,msg.info.height):
                if msg.data[height * msg.info.width + width] > 0 :
                    ob1.append([width * msg.info.resolution + msg.info.resolution / 2 + origin.position.x, height * msg.info.resolution + msg.info.resolution / 2 + origin.position.y])

        ob1.sort()
        ob_x= [ ob1[i][0] for i in range(len(ob1)) ]
        ob_y= [ ob1[i][1] for i in range(len(ob1)) ]
        params.ob = np.column_stack((np.array(ob_x), np.array(ob_y)))

def main():
    gotOdom = False
    print(__file__ + " start!!")

    rclpy.init()
    frenet_planner = Frenet_Planner()
    tx, ty, tyaw, tc, csp = generate_target_course(params.W_X, params.W_Y)

    # initial state
    c_speed = 10.0 / 3.6  # current speed [m/s]
    c_d = 2.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    global_s=[]
    global_s.append(0)
    s=0
    for i in range (1,len(tx)):
        dis = calc_dis(tx[i], ty[i], tx[i - 1], ty[i - 1])
        s = s + dis
        global_s.append(s)

    params.s_dest=global_s[len(tx)-1]
    area = 20.0  # animation area length [m]
    path= FrenetPath()
    path_msg= Path()
    global_path_msg= Path()
    while (rclpy.ok()):
        params.min_id=0
        if (False): # set to False since we dont have actual pose data durin testing
            params.min_id = initial_conditions_new(csp, global_s, tx, ty, tc, tyaw, path)
        if (params.s_dest - s0 <= params.LOOKAHEAD_DIST or s0 - params.s_dest >= 1):
            params.STOP_CAR = True
            params.TARGET_SPEED = 0.0
        path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd)

        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]

        if (np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0)  :
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(tx, ty)
            if(params.ob != []):
                plt.plot(params.ob[:, 0], params.ob[:, 1], "xk")
            plt.plot(path.x[1:], path.y[1:], "-or")
            plt.plot(path.x[1], path.y[1], "vc")
            plt.xlim(path.x[1] - area, path.x[1] + area)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
            plt.grid(True)
            plt.pause(0.0001)
        

        global_path_msg.poses=[]
        for i in range(len(tx)):
            loc= PoseStamped()
            loc.pose.position.x = tx[i]
            loc.pose.position.y = ty[i]
            global_path_msg.poses.append(loc)
        
        if (len(path.d) <= 1 or len(path.s_d) <= 1 or len(path.d_d) <= 1) :
            bot_v = np.sqrt((1 - tc[params.min_id] * c_d)** 2 * (c_speed)** 2 + (c_d_d)** 2)
        else :
            if (params.STOP_CAR):
                bot_v = calc_bot_v(path.d, path.s_d, path.d_d,tc)
            else :
                bot_v = np.sqrt(pow(1 - tc[params.min_id] * path.d[1], 2) * pow(path.s_d[1], 2) +
							 pow(path.d_d[1], 2))
        if (params.STOP_CAR):
            print(bot_v)
        
        vel=Twist()
        vel.linear.x = bot_v
        vel.linear.y = 0.0
        vel.linear.z = 0.0

        path_msg=path_to_msg(path,tc,tyaw,c_speed,c_d,c_d_d)
        path_msg.header.frame_id = "map"
        global_path_msg.header.frame_id = "map"
        frenet_planner.frenet_path.publish(path_msg)
        frenet_planner.global_path.publish(global_path_msg)
        frenet_planner.target_vel.publish(vel)
        rclpy.spin_once(frenet_planner)

    print("Finish")
    if show_animation:  # pragma: no cover
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()

    frenet_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()