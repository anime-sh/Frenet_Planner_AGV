#!/usr/bin/env python
from frenet_planner import params
import numpy as np
import copy
import math
import bisect
import time
import multiprocessing as mp
from joblib import Parallel, delayed

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

# # calculate list of frenet paths given start and end states
def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []
    if (params.STOP_CAR): 
        frenet_paths=Parallel(n_jobs=1)(delayed(calc_frenet_path_stop)(di,Ti,c_speed, c_d, c_d_d, c_d_dd, s0) for di in np.arange(-params.MAX_ROAD_WIDTH, params.MAX_ROAD_WIDTH, params.D_ROAD_W) for Ti in np.arange(params.MIN_T, params.MAX_T, params.DT) )
        return frenet_paths

    # generate path to each offset goal
    frenet_paths=Parallel(n_jobs=mp.cpu_count())(delayed(calc_frenet_path_non_stop)(di,Ti,Di_d,tv,c_speed, c_d, c_d_d, c_d_dd, s0) for di in np.arange(-params.MAX_ROAD_WIDTH, params.MAX_ROAD_WIDTH, params.D_ROAD_W) for Ti in np.arange(params.MIN_T, params.MAX_T, params.DT) for Di_d in np.arange(-params.MAX_LAT_VEL,params.MAX_LAT_VEL+0.001,params.D_D_NS) for tv in np.arange(params.TARGET_SPEED - params.D_T_S * params.N_S_SAMPLE,params.TARGET_SPEED + params.D_T_S * params.N_S_SAMPLE, params.D_T_S)) 
    return frenet_paths

def calc_frenet_path_stop(di,Ti,c_speed, c_d, c_d_d, c_d_dd, s0):
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
    # print(tfp.s)
    return tfp

def calc_frenet_path_non_stop(di,Ti,Di_d,tv,c_speed, c_d, c_d_d, c_d_dd, s0):
    fp = FrenetPath()

    lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, Di_d, 0.0, Ti)

    fp.t = [t for t in np.arange(0.0, Ti, params.DT)]
    fp.d = [lat_qp.calc_point(t) for t in fp.t]
    fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
    fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
    fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

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

    return tfp


# convert frenet paths to global frame
def calc_global_paths(fp, csp):

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

    return fp

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

def nearest_obs(point32):
    p_x=point32.x
    p_y=point32.y
    ob_x = params.ob[:,0]
    ob_y = params.ob[:,1]
    
    it = bisect.bisect_left(ob_x,p_x,lo = 0,hi = len(ob_x))

    if it==0:
        if(calc_dis(p_x,p_y,ob_x[it],ob_y[it])<= params.OBSTACLE_RADIUS):
            return False
    elif it==len(ob_x):
        it = it-1
        if(calc_dis(p_x,p_y,ob_x[it],ob_y[it])<= params.OBSTACLE_RADIUS):
            return False
    else:
        if(calc_dis(p_x,p_y,ob_x[it],ob_y[it])<= params.OBSTACLE_RADIUS) and (calc_dis(p_x,p_y,ob_x[it-1],ob_y[it-1])> params.OBSTACLE_RADIUS):
            return False

    it = bisect.bisect_left(ob_y,p_y,lo = 0,hi = len(ob_y))

    if it==0:
        if(calc_dis(p_x,p_y,ob_x[it],ob_y[it])<= params.OBSTACLE_RADIUS):
            return False
    elif it==len(ob_x):
        it = it-1
        if(calc_dis(p_x,p_y,ob_x[it],ob_y[it])<= params.OBSTACLE_RADIUS):
            return False
    else:
        if(calc_dis(p_x,p_y,ob_x[it],ob_y[it])<= params.OBSTACLE_RADIUS) and (calc_dis(p_x,p_y,ob_x[it-1],ob_y[it-1])> params.OBSTACLE_RADIUS):
            return False

    return True

# faster check for obstacles in a radius given by OBSTACLE_RADIUS around 3 bounding circle centres
def check_collision(fp):
    if(params.ob==[]):
        return True
    triangle1 = Point32()
    triangle2 = Point32()
    triangle3 = Point32()
    triangle3._x = 0.0
    triangle3._y = 0.0 
    triangle3._z = 0.0
    triangle2._x = 1.0
    triangle2._y = 0.0 
    triangle2._z = 0.0
    triangle1._x = 2.0
    triangle1._y = 0.0
    triangle1._z = 0.0

    params.footprint.polygon.points = [triangle1,triangle2,triangle3]
    for i in range(min(len(fp.x),len(fp.yaw))):
        trans_footprint = transformation(params.footprint.polygon.points,params.odom.pose.pose,fp.x[i],fp.y[i],fp.yaw[i])
        for j in trans_footprint:
                collision = nearest_obs(j)
                if not collision:
                    return False

    return True

# run collision checks and constraint-checking on all initially generated paths
def check_paths(fp):
    if any([v > params.MAX_SPEED for v in fp.s_d]):  # Max speed check
        return False
    elif any([abs(a) > params.MAX_ACCEL for a in
                fp.s_dd]):  # Max accel check
        return False
    elif any([abs(c) > params.MAX_CURVATURE for c in
                fp.c]):  # Max curvature check
        return False

    return check_collision(fp)


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

# calculate best_path between start and end states
def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd):
    start_calc_frenet=time.time()
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    end_calc_frenet=time.time()
    print("calc_frenet_paths time elapsed: "+str(end_calc_frenet -start_calc_frenet))

    try: import operator
    except ImportError: keyfun= lambda x: x.cf # use a lambda if no operator module
    else: keyfun= operator.attrgetter("cf") # use operator since it's faster than lambda

    fplist.sort(key=keyfun, reverse=False) # sort in-place

    for fp in fplist:
        start_calc_global=time.time()
        fp = calc_global_paths(fp, csp)
        if check_paths(fp):
            end_calc_global=time.time()
            print("calc_global_paths time elapsed: "+str(end_calc_global -start_calc_global))
            return fp
    else:
        print("No path!")

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

