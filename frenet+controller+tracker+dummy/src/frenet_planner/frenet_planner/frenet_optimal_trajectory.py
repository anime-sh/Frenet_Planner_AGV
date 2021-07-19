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
import copy
import math
import sys
import os

import rclpy
from rclpy.node import Node
# import tf2_ros
# from tf2_ros.transformations import quaternion_from_euler
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PolygonStamped
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
import rclpy.parameter 

# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/../QuinticPolynomialsPlanner/")
# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/../CubicSpline/")
# from quintic_polynomials_planner import QuinticPolynomial
# import cubic_spline_planner
try:
    from frenet_planner.quintic_polynomials_planner import QuinticPolynomial
    from frenet_planner import cubic_spline_planner
except ImportError:
    raise

cost_count = 0
footprint_count = 0
odom_count = 0
ob=np.array([])
wx=[]
wy=[]
odom = Odometry()
footprint = PolygonStamped()
cmap = OccupancyGrid()
# SIM_LOOP = 500

# Parameter
MAX_SPEED = 50.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 2.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 1.0  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 7.0  # maximum road width [m]
D_ROAD_W = 1.0  # road width sampling length [m]
DT = 0.2  # time tick [s]
MAX_T = 5.0  # max prediction time [m]
MIN_T = 4.0  # min prediction time [m]
TARGET_SPEED = 30.0 / 3.6  # target speed [m/s]
D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
ROBOT_RADIUS = 2.0  # robot radius [m]
STOP_CAR = False
# cost weights
K_J = 0.1
K_T = 0.1
K_D = 1.0
K_LAT = 1.0
K_LON = 1.0
FLT_MAX=sys.float_info.max
show_animation = True


class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        for Ti in np.arange(MIN_T, MAX_T, DT):
            fp = FrenetPath()

            # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Velocity keeping)
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE,
                                TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = QuarticPolynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2

                tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):
    for fp in fplist:

        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            i_yaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
            fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.hypot(dx, dy))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ob):
    for i in range(ob.shape[0]):
        d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= ROBOT_RADIUS ** 2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob):
    ok_ind = []
    for i, _ in enumerate(fplist):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            continue
        elif any([abs(a) > MAX_ACCEL for a in
                  fplist[i].s_dd]):  # Max accel check
            continue
        elif any([abs(c) > MAX_CURVATURE for c in
                  fplist[i].c]):  # Max curvature check
            continue
        elif not check_collision(fplist[i], ob):
            continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)
    fplist = check_paths(fplist, ob)

    # find minimum cost path
    min_cost = float("inf")
    best_path = None
    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp

    return best_path


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp

def calc_dis( x1, y1, x2, y2):
	return np.sqrt((x1 - x2)** 2 + (y1 - y2)** 2)

def get_bot_yaw():
    q = odom.pose.pose.orientation
    yaw = np.arctan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
    return yaw

def quaternion_from_euler(yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

def path_to_msg(path,rk,ryaw,c_speed,c_d,c_d_d):
    path_msg = Path()
    loc = PoseStamped()
    x_vec = path.x
    y_vec = path.y
    for i in range(len(path.x)):
        loc.pose.position.x = x_vec[i]
        loc.pose.position.y = y_vec[i]
        delta_theta = np.arctan(c_d_d / ((1 - rk[i] * c_d) * c_speed))
        yaw = delta_theta + ryaw[i]
        q = quaternion_from_euler(0, 0, yaw)
        q=np.array(q)
        denom= np.sqrt(np.sum(q*q))
        q_norm = [float(i)/denom for i in q]
        # tf.quaternionTFToMsg(q, loc.pose.orientation)
        loc.pose.orientation.x= q_norm[0] 
        loc.pose.orientation.y= q_norm[1]
        loc.pose.orientation.z= q_norm[2]
        loc.pose.orientation.w= q_norm[3]
        path_msg.poses.append(loc)
    return path_msg

def find_nearest_in_global_path(global_x, global_y, flag,path):
    global min_id
    global min_x
    global min_y
    global min_dis
    if (flag == 0):
        bot_x = odom.pose.pose.position.x
        bot_y = odom.pose.pose.position.y
    else:
        bot_x = path.get_x()[1];
        bot_y = path.get_y()[1];
    min_dis = FLT_MAX;
    for i in range(len(global_x)):
        dis = calc_dis(global_x[i], global_y[i], bot_x, bot_y)
        if (dis < min_dis):
            min_dis = dis;
            min_x = global_x[i];
            min_y = global_y[i];
            min_id = i;

def initial_conditions_new(csp, global_s, global_x, global_y,global_R, global_yaw, s0, c_speed, c_d,c_d_d,c_d_dd, path):
    global bot_yaw
    vx = odom.twist.twist.linear.x
    vy = odom.twist.twist.linear.y
    v = np.sqrt(vx * vx + vy * vy)

    find_nearest_in_global_path(global_x, global_y, 0, path)

    vec1=[0,0]
    vec2=[0,0]
    vec1[0] = odom.pose.pose.position.x - global_x[min_id]
    vec1[1] = odom.pose.pose.position.y - global_y[min_id]
    vec2[0] = global_x[min_id] - global_x[min_id + 1]
    vec2[1] = global_y[min_id] - global_y[min_id + 1]
    curl2D = vec1[0] * vec2[1] - vec2[0] * vec1[1]
    if (curl2D < 0):
        c_d =c_d*-1
    s0 = global_s[min_id]
    bot_yaw = get_bot_yaw()
    g_path_yaw = global_yaw[min_id]
    delta_theta = bot_yaw - g_path_yaw
    c_d_d = v * np.sin(delta_theta)
    k_r = global_R[min_id]
    c_speed = v * np.cos(delta_theta) / (1 - k_r * c_d)
    c_d_dd = 0
    return min_id

def calc_bot_v ( d, s_d, d_d,rk):
    return np.sqrt(pow(1 - rk[min_id] * d[len(d) / 2], 2) * pow(s_d[len(s_d) / 2], 2) +pow(d_d[len(d_d) / 2], 2));


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
        self.costmap_sub = self.create_subscription(OccupancyGrid,'costmap',self.costmap_callback,100)
        self.costmap_sub # prevent unused variable warning
        global wx
        global wy

        #dummy stuff

        self.declare_parameter("W_X",[])
        self.declare_parameter("W_Y",[])

        dummy_param_0 = rclpy.parameter.Parameter("W_X",rclpy.Parameter.Type.DOUBLE_ARRAY,[0.0 , 10.0 , 20.0 , 30.0 , 40.0 , 50.0 , 60.0 , 70.0 , 80.0])
        dummy_param_1 = rclpy.parameter.Parameter("W_Y",rclpy.Parameter.Type.DOUBLE_ARRAY,[0.0 , 10.0 , 20.0 , 30.0 , 40.0 , 50.0 , 60.0 , 70.0 , 80.0])

        self.set_parameters([dummy_param_0])
        self.set_parameters([dummy_param_1])

        wx = self.get_parameter("W_X").get_parameter_value().double_array_value
        wy = self.get_parameter("W_Y").get_parameter_value().double_array_value
        
    def odom_callback(self, msg):
        global odom
        odom = msg

    def footprint_callback(self, msg):
        global footprint
        footprint = msg

    def costmap_callback(self, msg):
        global cmap
        global ob
        global cost_count
        cost_count+=1
        ob1=[]
        cmap=msg
        origin=Pose()
        origin=msg.info.origin
        for width in range(0,msg.info.width):
            for height in range(0,msg.info.height):
                if msg.data[height * msg.info.width + width] > 0 :
                    ob1.append([width * msg.info.resolution + msg.info.resolution / 2 + origin.position.x, height * msg.info.resolution + msg.info.resolution / 2 + origin.position.y])

        ob1.sort()
        ob_x= [ ob1[i][0] for i in range(len(ob1)) ]
        ob_y= [ ob1[i][1] for i in range(len(ob1)) ]
        ob = np.column_stack((np.array(ob_x), np.array(ob_y)))


def main():
    global min_id
    gotOdom = False
    print(__file__ + " start!!")

    rclpy.init()
    frenet_planner = Frenet_Planner()
    # # # way points
    # wx = [0.0, 10.0, 20.5, 35.0, 70.5]
    # wy = [0.0, -6.0, 5.0, 6.5, 0.0]
    # # # obstacle lists
    # ob = np.array([[20.0, 10.0],
    #                [30.0, 6.0],
    #                [30.0, 8.0],
    #                [35.0, 8.0],
    #                 [50.0, 3.0]
    #                 ])

    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)

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

    s_dest=global_s[len(tx)-1]
    run_frenet = True


    area = 20.0  # animation area length [m]
    path_msg= Path()
    global_path_msg= Path()
    # for i in range(SIM_LOOP):
    while (rclpy.ok()):
        path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)
        min_id=0
        if (True):
            min_id = initial_conditions_new(csp, global_s, tx, ty, tc, tyaw, s0, c_speed, c_d, c_d_d,c_d_dd, path)


        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]

        if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(tx, ty)
            if(ob != []):
                plt.plot(ob[:, 0], ob[:, 1], "xk")
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
            bot_v = np.sqrt((1 - tc[min_id] * c_d)** 2 * (c_speed)** 2 + (c_d_d)** 2)
        else :
            if (STOP_CAR):
                bot_v = calc_bot_v(path.get_d(), path.get_s_d(), path.get_d_d(),tc)
            else :
                bot_v = np.sqrt(pow(1 - tc[min_id] * path.d[1], 2) * pow(path.s_d[1], 2) +
							 pow(path.d_d[1], 2))
        if (STOP_CAR):
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


        
