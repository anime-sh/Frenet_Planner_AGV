Class FrenetPath
================
Member Variables
****************
t
^^^
    t is the vector comprising of the discrete points in time at which other translational variables are calculated
d
^
    d is the vector comprising of the lateral offset(distance) from the expected global path at the discrete points in time
d_d
^^^^
    d_d is the vector comprising of the first derivative of d with respect to t
d_dd
^^^^
    d_dd is the vector comprising of the second derivative of d with respect to t
d_ddd
^^^^^^
    d_ddd is the vector comprising of the third derivative of d with respect to t
s
^
    s is the vector comprising of the longitudinal arc length covered at the discrete points in time
s_d
---
    s_d is the vector comprising of the first derivative of s with respect to t
s_dd
^^^^
    s_dd is the vector comprising of the second derivative of s with respect to t
s_ddd
^^^^^
    s_ddd is the vector comprising of the third derivative of s with respect to t
x
^
    x stores positions of sampled points on path along x axis
y
^
    y stores positions of sampled points on path along y axis
yaw
^^^
    yaw is the yaw of the path at points
ds
^^
    ds stores distance between sampled points on the path
c
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    c is the curvature of the path
Js
^^
    Js is the summation comprising of the square of all values of d_ddd (latitudinal jerk)
Jp
^^
    Jp is the summation comprising of the square of all values of s_ddd (longitudinal jerk)
cd
^^
    cd is the cost of the lateral trajectory in the frenet path
cv
^^
    cv is the cost of the longitudinal trajectory in the frenet path
cf
^^
    cf is the total cost of the frenet path(comprises of weighted values of cd and cv)
Ti
^^    
    Ti is the time difference after which end state is achieved from the start state for each sampling
dss
^^^
    dss stores the square values of difference in speeds; needed for cost calculations- - - 
Member Functions
****************
get functions
^^^^^^^^^^^^^
set functions
^^^^^^^^^^^^^
adding_global_path
^^^^^^^^^^^^^^^^^^
    - Argument : Spline2D object
    - Return Type : void
    - Function :  convert the frenet path to global frame
check_collision
^^^^^^^^^^^^^^^
    - Argument : double
    - Return Type : bool
    - Function :  checks for collision of the bot
plot_path
^^^^^^^^^
    - Argument : void
    - Return Type : void
    - Function :  helper function to plot path
plot_velocity_profile
^^^^^^^^^^^^^^^^^^^^^^^
    - Argument : void
    - Return Type : void
    - Function :  helper function to plot velocity profile
Friend Class
^^^^^^^^^^^^
    - Class Fplist

