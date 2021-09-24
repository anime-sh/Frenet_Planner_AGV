==============================
ROS Publisher/Subscriber Setup
==============================
**************
ROS publishers
**************
/frenet_path
^^^^^^^^^^^^ 
    publishes the frenet path
/global_path
^^^^^^^^^^^^
    publishes the global path
/cmd_vel
^^^^^^^^
    publishes the target velocity
***************
ROS subscribers
***************
/base_pose_ground_truth
^^^^^^^^^^^^^^^^^^^^^^^
    subscribes to the pose of the bot
/move_base/local_costmap/footprint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    subscribes to the footprint of the bot
/move_base/local_costmap/costmap
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    subscribes to the costmap 

