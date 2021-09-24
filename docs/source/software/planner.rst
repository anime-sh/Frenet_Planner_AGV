========================
The Frenet Frame Planner
========================
Research Paper
==============
Werling, Moritz, Julius Ziegler, Sören Kammel, and Sebastian Thrun. "Optimal trajectory generation for dynamic street scenarios in a frenet frame." In 2010 IEEE International Conference on Robotics and Automation, pp. 987-993. IEEE, 2010.
`(view the paper) <https://ieeexplore.ieee.org/abstract/document/5509799/>`_


Folder Structure
==================
The repository is composed of different components. Each of them is contained inside the src folder.

+--------------------------------+-----------------------------------------------------------------------------+
| File                           | Description                                                                 |
+================================+=============================================================================+
| cubic_spline_planner.cpp       | This file holds Spline2D class definition.                                  |
+--------------------------------+-----------------------------------------------------------------------------+
| polynomials.cpp                | This file holds quintic and quartic class definitions.                      |
+--------------------------------+-----------------------------------------------------------------------------+
| frenet_optimal_trajectory.cpp  | This folder holds all helper functions needed for best path sampling.       |
+--------------------------------+-----------------------------------------------------------------------------+
| frenetROS_obst.cpp             | This is the main file and publishes best path iteratively to the publishers |
+--------------------------------+-----------------------------------------------------------------------------+

Codeflow Structure
==================
    1. The launch file shifts control to the main function which resides under frenetROS_obst.cpp .
    2. Nodes are initialised and parameters are loaded from yaml file. 
    3. Publishers and subscribers are created and registered with the ROS Master.
    4. Control now enters the iterative portion. It remains here until the ego vehicle has reached its destination.
    5. Inside the iterative loop, initialising functions are called to give the current state and end state for the ego vehicle. These details are passed to the frenet_optimal_planning function.
    6. frenet_optimal_planning function in turn calls the Fplist class constructor.
    7. The constructor sequentially samples all latitudinal paths and longitudinal paths. The copy function is then called to merge these two sampled parts and after a cross product, all sampled paths are populated in fplist_lat.
    8. Costs are calculated for sampled paths and used to sort the paths.
    9. Sorted paths are traversed in ascending order, converted into global frame and checked for collision. Detection of collision free path results in a return to the main function.
    10. Once the destination waypoint is reached, loop is exited and program ends.
