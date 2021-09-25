============
Class Fplist
============
****************
Member Variables
****************

c_speed
^^^^^^^
    current longitudinal speed of the ego vehicle
c_d
^^^
    current latitudinal position of the ego vehicle
c_d_d
^^^^^
    current latitudinal velocity of the ego vehicle
c_d_dd
^^^^^^
    current latitudinal acceleration of the ego vehicle
s0
^^
    current longitudinal position of ego vehicle
fplist_lat
^^^^^^^^^^
    latitudinal part of the FrenetPath object is sampled and stored in fplist_lat
fplist_lon
^^^^^^^^^^
    longitudinal part of the FrenetPath object is sampled and stored in fplist_lon
    samples_tv
^^^^^^^^^^
    number of samples at each time value
    
****************
Member Functions
****************

Fplist (Constructor)
^^^^^^^^^^^^^^^^^^^^
    - Argument : double, double, double, double, double
    - Return Type : void
    - Function :  class constructor which generates paths
calc_lat
^^^^^^^^
    - Argument : double, double, double
    - Return Type : FrenetPath object
    - Function :  calculates latitudinal part of FrenetPath object using the sampling parameters passed
calc_lon
^^^^^^^^
    - Argument : double, double 
    - Return Type : FrenetPath object
    - Function :  calculates longitudinal part of FrenetPath object using the sampling parameters passed
copy
^^^^
    - Argument : int
    - Return Type : void
    - Function :  cross product of sampled latitudinal and longitudinal parts
calc_cost
^^^^^^^^^
    - Argument : int
    - Return Type : void
    - Function :  calculate final cost of path as weighted sum of latitudinal and longitudinal path costs

