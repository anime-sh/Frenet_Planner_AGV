import numpy as np
# import messages
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32

# Initialise Parameters
K_J = 0.1
K_T = 0.1
K_D = 1.0
K_LAT = 1.0
K_LON = 1.0
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
OBSTACLE_RADIUS = 0.75  # obstacle radius [m]
MIN_LAT_VEL=-5.0
MAX_LAT_VEL=5.0
D_D_NS=1.0
W_X=[0.0,10.0,20.0]
W_Y=[0.0,15.0,12.0]
LOOKAHEAD_DIST=15

# global variables
STOP_CAR = False
FLT_MAX=float("inf")
s_dest=5.0
min_id=0.0
ob = np.array([[ 0.5 , 0.5]])

footprint = PolygonStamped()
triangle1 = Point32()
triangle2 = Point32()
triangle3 = Point32()
triangle4 = Point32()
triangle4._x = 0.0
triangle4._y = 0.0 
triangle4._z = 0.0
triangle2._x = 2.0
triangle2._y = 2.0 
triangle2._z = 0.0
triangle3._x = 0.0
triangle3._y = 2.0
triangle3._z = 0.0
triangle1._x = 2.0
triangle1._y = 0.0
triangle1._z = 0.0

footprint.polygon.points = [triangle1,triangle2,triangle3,triangle4]
odom = Odometry()
cmap = OccupancyGrid()
