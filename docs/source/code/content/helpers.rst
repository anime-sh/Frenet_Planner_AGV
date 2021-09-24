================
Helper Functions
================

*****************************
frenet_optimal_trajectory.cpp
*****************************

dist 
^^^^
    - Argument : double, double, double, double
    - Return Type : double
    - Function : returns distance between two points
check_path
^^^^^^^^^^
    - Argument : FrenetPath object, double, double, double
    - Return Type : bool
    - Function : check for specified velocity, acceleration, curvature constraints and collisions
calc_global_path
^^^^^^^^^^^^^^^^
    - Argument : FrenetPath object, Spline2D object
    - Return Type : FrenetPath
    - Function : convert the frenet path to global frame
frenet_optimal_planning
^^^^^^^^^^^^^^^^^^^^^^^
    - Argument : Spline2D, double, double, double, double, double, FrenetPath, double
    - Return Type : FrenetPath
    - Function : samples paths and returns the bestpath
transformation
^^^^^^^^^^^^^^ 
    - Argument : vector<geometry_msgs::Point32>, geometry_msgs::Pose, double, double, double
    - Return Type : vector<geometry_msgs::Point32>
    - Function : transforms robot's footprint according to yaw at particular location on the path
point_obcheck 
^^^^^^^^^^^^^
    - Argument : geometry_msgs::Point32, double
    - Return Type : bool
    - Function : check collision with the two bounding nearest obstacle on x and y axes 
sortByCost (inline)
^^^^^^^^^^^^^^^^^^^ 
    - Argument : FrenetPath &, FrenetPath &
    - Return Type : bool
    - Function : comparator function for sorting FrenetPaths on cost 

******************
frenetROS_obst.cpp
******************

costmap_callback
^^^^^^^^^^^^^^^^ 
    - Argument : const nav_msgs::OccupancyGrid::ConstPtr &
    - Return Type : void 
    - Function : accesses the costmap and updates the obstacle coordinates
footprint_callback
^^^^^^^^^^^^^^^^^^
    - Argument : const geometry_msgs::PolygonStampedConstPtr &
    - Return Type : void
    - Function : accesses the robot footprint
odom_callback
^^^^^^^^^^^^^
    - Argument : const nav_msgs::Odometry::ConstPtr &
    - Return Type : void 
    - Function: accesses the odometry data
find_nearest_in_global_path
^^^^^^^^^^^^^^^^^^^^^^^^^^^
    - Argument : vector<double>, vector<double>, double, double,double,  int, int, FrenetPath &
    - Return Type : void
    - Function : finds the point in the global path which is nearest to the bot
get_bot_yaw
^^^^^^^^^^^
    - Argument : nil 
    - Return Type : double
    - Function : calculates yaw of ego vehicle using odom
initial_conditions_new
^^^^^^^^^^^^^^^^^^^^^^  
    - Argument : Spline2D object, vector<double>, vector<double>, vector<double>, vector<double>, vector<double>, double, double, double, double
    - Return Type : int
    - Function : provides initial conditions for sampling of paths
publishPath
^^^^^^^^^^^
    - Argument : nav_msgs::Path &, FrenetPath object, vector<double>,vector<double>,double, double, double
    - Return Type : void
    - Function :  publishes path as ros messages

