*****************************
Class Spline2D
*****************************
calc_A
^^^^^^^
    - Argument : vector<double>
    - Return Type : matrix
    - Function : calculates and returns matrix A
calc_B
^^^^^^^^^^
    - Argument : vector<double>
    - Return Type : matrix
    - Function : calculates and returns matrix B
search_index
^^^^^^^^^^^^^^ 
    - Argument : double
    - Return Type : v=int
    - Function : returns index of the just greater element
init
^^^^^
    - Argument : vector<double>, vector<double>
    - Return Type : void
    - Function : calculates the coeffs(a[i], b[i], c[i], d[i]) for splines by making calls to calc_A, calc_B and performing appropriate mathematical operations 
calc
^^^^^
    - Argument : double
    - Return Type : double
    - Function : find y at given x
    
calcd
^^^^^^
    - Argument : double
    - Return Type : double
    - Function : find y_dot at given x
    
calcdd
^^^^^^^^^^^^^^^^^^
    - Argument : double
    - Return Type : double
    - Function : find y_doubleDot at given x

calc_s
^^^^^^^^
    - Argument : vector<double>, vector<double>
    - Return Type : vector<double>
    - Function: approximately calculates s along the spline

calc_position
^^^^^^^^^^^^^^^^^^^^^^^^^^^
    - Argument : double, double, double
    - Return Type : void
    - Function : calculates x and y at particular point on the spline
calc_curvature
^^^^^^^^^^^
    - Argument : double
    - Return Type : double
    - Function : calculates curvature at particular point on the spline 
calc_yaw
^^^^^^^^^^^^^^^^^^^^^^  
    - Argument : double
    - Return Type : double
    - Function : calculates curvature at particular point on the spline 
get_s_last
^^^^^^^^^^^
    - Argument : nil
    - Return Type : double
    - Function :  returns last s 
calc_spline_course
^^^^^^^^^^^^^^^^^^^
    - Argument : vec<double> , vec<double> , vec<double> , vec<double> , vec<double> , vec<double> , double
    - Return Type : Spline2D
    - Function :  generates the Spline2D with points along the spline at distance = ds, also returns yaw and curvature
