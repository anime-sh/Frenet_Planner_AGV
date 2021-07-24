
from frenet_planner import params
import numpy as np
import copy
import math

# import messages
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32

from frenet_planner.quintic_polynomials_planner import QuinticPolynomial
from frenet_planner import cubic_spline_planner
from frenet_planner.tf2 import quaternion_from_euler,euler_from_quaternion

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

# calculate list of frenet paths given start and end states
def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []
    if (params.STOP_CAR):
        for di in np.arange(-params.MAX_ROAD_WIDTH, params.MAX_ROAD_WIDTH, params.D_ROAD_W):
            for Ti in np.arange(params.DT, params.MAX_T, params.DT):
                fp = FrenetPath()
                lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
                fp.t = [t for t in np.arange(0.0, Ti, params.DT)]
                fp.d = [lat_qp.calc_point(t) for t in fp.t]
                fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
                fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
                fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

                tfp = copy.deepcopy(fp)
                lon_qp = QuinticPolynomial(s0, c_speed, 0.0, min(s0+params.LOOKAHEAD_DIST,params.s_dest),params.TARGET_SPEED, 0.0, Ti)
                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (params.TARGET_SPEED - tfp.s_d[-1]) ** 2

                tfp.cd = params.K_J * Jp + params.K_T * Ti + params.K_D * tfp.d[-1] ** 2
                tfp.cv = params.K_J * Js + params.K_T * Ti + params.K_D * ds
                tfp.cf = params.K_LAT * tfp.cd + params.K_LON * tfp.cv

                frenet_paths.append(tfp)
        return frenet_paths


    # generate path to each offset goal
    for di in np.arange(-params.MAX_ROAD_WIDTH, params.MAX_ROAD_WIDTH, params.D_ROAD_W):
        # Lateral motion planning
        for Ti in np.arange(params.MIN_T, params.MAX_T, params.DT):
            for Di_d in np.arange(params.MIN_LAT_VEL,params.MAX_LAT_VEL,params.D_D_NS):
                fp = FrenetPath()

                lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, Di_d, 0.0, Ti)

                fp.t = [t for t in np.arange(0.0, Ti, params.DT)]
                fp.d = [lat_qp.calc_point(t) for t in fp.t]
                fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
                fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
                fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

                # Longitudinal motion planning (Velocity keeping)
                for tv in np.arange(params.TARGET_SPEED - params.D_T_S * params.N_S_SAMPLE,
                                    params.TARGET_SPEED + params.D_T_S * params.N_S_SAMPLE, params.D_T_S):
                    tfp = copy.deepcopy(fp)
                    lon_qp = QuinticPolynomial(s0, c_speed, 0.0, s0+params.LOOKAHEAD_DIST,tv, 0.0, Ti)

                    tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                    tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                    tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                    tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                    Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                    Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                    # square of diff from target speed
                    ds = (params.TARGET_SPEED - tfp.s_d[-1]) ** 2

                    tfp.cd = params.K_J * Jp + params.K_T * Ti + params.K_D * tfp.d[-1] ** 2
                    tfp.cv = params.K_J * Js + params.K_T * Ti + params.K_D * ds
                    tfp.cf = params.K_LAT * tfp.cd + params.K_LON * tfp.cv

                    frenet_paths.append(tfp)
    return frenet_paths

# convert frenet paths to global frame
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
        if(len(fp.x)>1):
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

# calculate footprint polygon at given position on path via transformation of bot footprint 
# simple rotation and translation via math referenced here -> https://drive.google.com/file/d/1_xbz9cf8KGwfHLOJU2muVP7nrP_SInJE/view?usp=sharing
def transformation(foot_p,cp,px,py,pyaw):
    _,_,byaw = euler_from_quaternion(cp.orientation.x, cp.orientation.y, cp.orientation.z, cp.orientation.w)

    theta = pyaw - byaw 
    
    x=[]
    y=[]
    for i in foot_p:
        x.append(i.x)
        y.append(i.y)

    orig = np.column_stack((np.array(x), np.array(y)))
    final = np.zeros(orig.shape)

    xc=np.sum(x)/len(foot_p)
    yc=np.sum(y)/len(foot_p)
    
    centre = np.array([xc,yc])
    rot_matrix = np.array([[np.cos(theta),np.sin(theta)],[-np.sin(theta),np.cos(theta)]])
    
    final = np.dot((orig - centre),rot_matrix) + np.array([px,py])
    
    new_foot_p=[]
    for i in range(len(final)):
        temp=Point32()
        x=final[i][0]
        y=final[i][1]
        temp.x=x
        temp.y=y
        new_foot_p.append(temp)
           
    return new_foot_p

# check for obstacles in a radius given by OBSTACLE_RADIUS around each vertice of transformed polygon
def check_collision(fp):
    if(params.ob==[]):
        return True
    for i in range(min(len(fp.x),len(fp.yaw))):
        trans_footprint = transformation(params.footprint.polygon.points,params.odom.pose.pose,fp.x[i],fp.y[i],fp.yaw[i])
        for j in trans_footprint:
                for i in range(params.ob.shape[0]): #currently checks with every obstacle 
                    ix = j.x
                    iy = j.y
                    d = ((ix - params.ob[i, 0]) ** 2) + ((iy - params.ob[i, 1]) ** 2)
                    collision = (d <= params.OBSTACLE_RADIUS ** 2 )
                    if collision:
                        return False

    return True

# run collision checks and constraint-checking on all initially generated paths
def check_paths(fplist):
    ok_ind = []
    # speed=0
    # acc=0
    # curvature=0
    # coll=0

    for i, _ in enumerate(fplist):
        if any([v > params.MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            # speed+=1
            continue
        elif any([abs(a) > params.MAX_ACCEL for a in
                  fplist[i].s_dd]):  # Max accel check
            # acc+=1
            continue
        elif any([abs(c) > params.MAX_CURVATURE for c in
                  fplist[i].c]):  # Max curvature check
            # curvature+=1
            continue
        elif not check_collision(fplist[i]):
            # coll+=1
            continue

        ok_ind.append(i)
    # print("s="+str(speed))
    # print("a="+str(acc))
    # print("c="+str(curvature))
    # print("coll="+str(coll))
    # print("remaining="+str(len(ok_ind)))
    return [fplist[i] for i in ok_ind]

# generate central spline from given waypoint coordinates
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

# # nearest in global path
# # functionality imitated from cpp codebase
# def find_nearest_in_global_path(global_x, global_y, flag,path):
#     global min_x
#     global min_y
#     global min_dis
#     if (flag == 0):
#         bot_x = params.odom.pose.pose.position.x
#         bot_y = params.odom.pose.pose.position.y
#     else:
#         bot_x = path.get_x()[1]
#         bot_y = path.get_y()[1]
#     min_dis = params.FLT_MAX
#     for i in range(len(global_x)):
#         dis = calc_dis(global_x[i], global_y[i], bot_x, bot_y)
#         if (dis < min_dis):
#             min_dis = dis
#             min_x = global_x[i]
#             min_y = global_y[i]
#             params.min_id = i

# # functionality imitated from cpp codebase
# def initial_conditions_new(csp, global_s, global_x, global_y,global_R, global_yaw, s0, c_speed, c_d,c_d_d,c_d_dd, path):
#     vx = params.odom.twist.twist.linear.x
#     vy = params.odom.twist.twist.linear.y
#     v = np.sqrt(vx * vx + vy * vy)

#     find_nearest_in_global_path(global_x, global_y, 0, path)

#     vec1=[0,0]
#     vec2=[0,0]
#     vec1[0] = params.odom.pose.pose.position.x - global_x[params.min_id]
#     vec1[1] = params.odom.pose.pose.position.y - global_y[params.min_id]
#     vec2[0] = global_x[params.min_id] - global_x[params.min_id + 1]
#     vec2[1] = global_y[params.min_id] - global_y[params.min_id + 1]
#     curl2D = vec1[0] * vec2[1] - vec2[0] * vec1[1]
#     if (curl2D < 0):
#         c_d =c_d*-1
#     s0 = global_s[params.min_id]
#     _,_,bot_yaw = euler_from_quaternion(params.odom.pose.pose.orientation.x,params.odom.pose.pose.orientation.y,params.odom.pose.pose.orientation.z,params.odom.pose.pose.orientation.w)
#     g_path_yaw = global_yaw[params.min_id]
#     delta_theta = bot_yaw - g_path_yaw
#     c_d_d = v * np.sin(delta_theta)
#     k_r = global_R[params.min_id]
#     c_speed = v * np.cos(delta_theta) / (1 - k_r * c_d)
#     c_d_dd = 0
#     return params.min_id

# calculate best_path between start and end states
def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd):
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)
    fplist = check_paths(fplist)

    # find minimum cost path
    min_cost = params.FLT_MAX
    best_path = None
    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp

    return best_path


# calculate target velocity
def calc_bot_v( d, s_d, d_d,rk):
    return np.sqrt(pow(1 - rk[int(params.min_id)] * d[int(len(d) / 2)], 2) * pow(s_d[int(len(s_d) / 2)], 2) +pow(d_d[int(len(d_d) / 2)], 2))

# calculate euclidean distance between two points
def calc_dis( x1, y1, x2, y2):
	return np.sqrt((x1 - x2)** 2 + (y1 - y2)** 2)

# generate Path() message to publish to /frenet_path
def path_to_msg(path,rk,ryaw,c_speed,c_d,c_d_d):
    path_msg = Path()
    loc = PoseStamped()
    x_vec = path.x
    y_vec = path.y
    for i in range(len(path.x)):
        loc.pose.position.x = x_vec[i]
        loc.pose.position.y = y_vec[i]
        delta_theta = math.atan(c_d_d / ((1 - rk[i] * c_d) * c_speed))
        yaw = delta_theta + ryaw[i]
        q = quaternion_from_euler(0, 0, yaw)
        q=np.array(q)
        denom= np.sqrt(np.sum(q*q))
        q_norm = [float(i)/denom for i in q]
        loc.pose.orientation.x= q_norm[0] 
        loc.pose.orientation.y= q_norm[1]
        loc.pose.orientation.z= q_norm[2]
        loc.pose.orientation.w= q_norm[3]
        path_msg.poses.append(loc)
    return path_msg

