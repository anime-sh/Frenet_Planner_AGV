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
        self.Jp=0.0
        self.Js=0.0
        self.dss=0.0
        self.Ti=0.0

class Fplist:
    def __init__(self,c_speed, c_d, c_d_d, c_d_dd, s0):
        self.c_speed= c_speed
        self.c_d= c_d
        self.c_d_d= c_d_d
        self.c_d_dd= c_d_dd
        self.s0= s0
        self.fplist_lat =[]
        self.fplist_lon =[]
        self.samples_tv = len(np.arange(params.TARGET_SPEED - params.D_T_S * params.N_S_SAMPLE,params.TARGET_SPEED + params.D_T_S * params.N_S_SAMPLE, params.D_T_S))

        if (params.STOP_CAR): 
            self.fplist_lat=Parallel(n_jobs=1)(delayed(self.calc_lat)(di,Ti,0.0) for di in np.arange(-params.MAX_ROAD_WIDTH, params.MAX_ROAD_WIDTH, params.D_ROAD_W) for Ti in np.arange(params.MIN_T, params.MAX_T, params.DT))
        else :
            self.fplist_lat=Parallel(n_jobs=1)(delayed(self.calc_lat)(di,Ti,Di_d) for di in np.arange(-params.MAX_ROAD_WIDTH, params.MAX_ROAD_WIDTH, params.D_ROAD_W) for Ti in np.arange(params.MIN_T, params.MAX_T, params.DT) for Di_d in np.arange(-params.MAX_LAT_VEL,params.MAX_LAT_VEL+0.001,params.D_D_NS))
            
        if (params.STOP_CAR): 
            self.fplist_lon=Parallel(n_jobs=1)(delayed(self.calc_lon)(params.TARGET_SPEED,Ti) for Ti in np.arange(params.MIN_T, params.MAX_T, params.DT))
        else:
            self.fplist_lon=Parallel(n_jobs=1)(delayed(self.calc_lon)(tv,Ti) for tv in np.arange(params.TARGET_SPEED - params.D_T_S * params.N_S_SAMPLE,params.TARGET_SPEED + params.D_T_S * params.N_S_SAMPLE, params.D_T_S) for Ti in np.arange(params.MIN_T, params.MAX_T, params.DT))
        
        self.fplist_lat = list(np.repeat(self.fplist_lat, self.samples_tv))
        
        try: import operator
        except ImportError: keyfun= lambda x: x.Ti # use a lambda if no operator module
        else: keyfun= operator.attrgetter("Ti") # use operator since it's faster than lambda
        self.fplist_lon.sort(key=keyfun, reverse=False) # sort in-place
        
        Parallel(n_jobs=1)(delayed(self.copy)(i) for i in np.arange(0,len(self.fplist_lat),self.samples_tv))
        Parallel(n_jobs=1)(delayed(self.calc_cost)(i) for i in range(len(self.fplist_lat)))

    def calc_lat(self,di,Ti,Di_d):
        fp = FrenetPath()
        lat_qp = QuinticPolynomial(self.c_d, self.c_d_d, self.c_d_dd, di, Di_d, 0.0, Ti)

        fp.Ti=Ti
        fp.t = np.arange(0.0, Ti, params.DT)
        fp.d = lat_qp.calc_point(fp.t)
        fp.d_d = lat_qp.calc_first_derivative(fp.t) 
        fp.d_dd = lat_qp.calc_second_derivative(fp.t) 
        fp.d_ddd = lat_qp.calc_third_derivative(fp.t) 
        fp.Jp =  np.sum(np.power(fp.d_ddd, 2))
        fp.cd = params.K_J * fp.Jp + params.K_T * fp.Ti + params.K_D * fp.d[-1] ** 2
        return fp

    def calc_lon(self,tv,Ti):
        fp = FrenetPath() 
        lon_qp = QuinticPolynomial(self.s0, self.c_speed, 0.0, min(self.s0+params.LOOKAHEAD_DIST,params.s_dest),tv, 0.0, Ti)
        
        fp.Ti=Ti
        fp.t = np.arange(0.0, Ti, params.DT)
        fp.s = lon_qp.calc_point(fp.t) 
        fp.s_d = lon_qp.calc_first_derivative(fp.t) 
        fp.s_dd = lon_qp.calc_second_derivative(fp.t) 
        fp.s_ddd = lon_qp.calc_third_derivative(fp.t) 
        fp.Js =  np.sum(np.power(fp.s_ddd, 2))
        fp.dss= (params.TARGET_SPEED - fp.s_d[-1]) ** 2
        fp.cv = params.K_J * fp.Js + params.K_T * fp.Ti + params.K_D * fp.dss
        return fp

    def copy(self,i):
        index_start = int(((self.fplist_lat[i]).Ti - params.MIN_T) * self.samples_tv)
        for j in range(self.samples_tv):
            self.fplist_lat[i+j].s = self.fplist_lon[index_start+j].s
            self.fplist_lat[i+j].s_d = self.fplist_lon[index_start+j].s_d
            self.fplist_lat[i+j].s_dd = self.fplist_lon[index_start+j].s_dd
            self.fplist_lat[i+j].s_ddd = self.fplist_lon[index_start+j].s_ddd
            self.fplist_lat[i+j].Js = self.fplist_lon[index_start+j].Js
            self.fplist_lat[i+j].dss = self.fplist_lon[index_start+j].dss
            self.fplist_lat[i+j].cv = self.fplist_lon[index_start+j].cv

    def calc_cost(self,i):
        self.fplist_lat[i].cf = params.K_LAT * self.fplist_lat[i].cd + params.K_LON * self.fplist_lat[i].cv

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
    fplist = Fplist(c_speed, c_d, c_d_d, c_d_dd, s0).fplist_lat
    end_calc_frenet=time.time()
    print("calc_frenet_paths time elapsed: "+str(end_calc_frenet -start_calc_frenet))

    try: import operator
    except ImportError: keyfun= lambda x: x.cf # use a lambda if no operator module
    else: keyfun= operator.attrgetter("cf") # use operator since it's faster than lambda

    fplist.sort(key=keyfun, reverse=False) # sort in-place
    start_calc_global=time.time()
    for fp in fplist:
        
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

