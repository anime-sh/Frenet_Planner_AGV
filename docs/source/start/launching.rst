=============================
Launching 
=============================
The following commands are to be run from your terminal after the workspace has been built:

.. code-block:: bash

    roscore
    roslaunch car_demo demo.launch
    roslaunch frenet_planner frenet_planner.launch
    python ~ catkin_ws/src/tracking_control/src/scripts/tracking_Methods/pure_pursuit.py
    python ~ catkin_ws/src/tracking_control/src/scripts/controllers/PID_MIT_velocity_controller.py
