==========
Parameters
==========
/frenet_planner/path/max_speed : MAX_SPEED 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    maximum speed [m/s]
/frenet_planner/path/max_accel : MAX_ACCEL 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    maximum acceleration [m/ss]
/frenet_planner/path/max_curvature : MAX_CURVATURE
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    maximum curvature [1/m]
/frenet_planner/path/max_road_width : MAX_ROAD_WIDTH
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    maximum road width [m]
/frenet_planner/path/d_road_w : D_ROAD_W
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    road width sampling length [m]
/frenet_planner/path/dt : DT
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    time tick [s]
/frenet_planner/path/maxt : MAXT
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    max prediction time [m]
/frenet_planner/path/mint : MINT
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    min prediction time [m]
/frenet_planner/path/target_speed : TARGET_SPEED
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    target speed [m/s]
/frenet_planner/path/d_t_s : D_T_S
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    target speed sampling length [m/s]
/frenet_planner/path/n_s_sample : N_S_SAMPLE
    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    sampling number of target speed
/frenet_planner/path/robot_radius : ROBOT_RADIUS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    robot radius [m]
/frenet_planner/path/max_lat_vel : MAX_LAT_VEL
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    minimum lateral speed sampling for d_d
/frenet_planner/path/min_lat_vel : MIN_LAT_VEL
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    maximum lateral speed sampling for d_d
/frenet_planner/path/d_d_ns : D_D_NS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Step size for sampling of d_d
/frenet_planner/path/max_shift_d : MAX_SHIFT_D
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Sampling width for sampling of d
/frenet_planner/cost/kj : KJ
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    cost weight
/frenet_planner/cost/kt : KT
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    cost weight
/frenet_planner/cost/kd : KD
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    cost weight
/frenet_planner/cost/kd_v : KD_V
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    cost weight
/frenet_planner/cost/klon : KLON
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    cost weight
/frenet_planner/cost/klat : KLAT
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    cost weight
/frenet_planner/waypoints/W_X : W_X
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    waypoint x-coordinates
/frenet_planner/waypoints/W_Y :W_Y
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    waypoint y-coordinates

