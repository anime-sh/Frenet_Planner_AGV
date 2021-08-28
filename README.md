# Frenet_Planner_AGV
This repository improves upon the work done previously on the sampling based planners in frenet frame. The improvements were done by [Parv Maheshwari](https://github.com/Parv-Maheshwari), [Animesh Jha](https://github.com/anime-sh), [Rohit Ranjan](https://github.com/thefurorjuror), [Himadri Bhatka](https://github.com/sirmisscriesalot) and [Aayush Jain](https://github.com/Aayush-Jain01).

## Changes and improvements
### Version 2.0 
#### Contributors
[Rohit Ranjan](https://github.com/thefurorjuror), [Himadri Bhatka](https://github.com/sirmisscriesalot) and [Aayush Jain](https://github.com/Aayush-Jain01)
#### Changes and Results
Here is a brief summary of the changes and improvements made in the planner:
* Sorting based on path cost introduced. Sampled paths are now checked for collision in ascending order of path cost.
* New class Fplist introduced.
* Fplist class reduces redundancy of repetitive polynomial sampling and provides efficient memory transactions.   
* Average frequency of over 25 Hz achieved.

A short [video](https://youtu.be/8txINQdnBJU) of a demo run.
### Version 1.0 
#### Contributors
[Parv Maheshwari](https://github.com/Parv-Maheshwari) and [Animesh Jha](https://github.com/anime-sh)
#### Changes and Results
Here is a brief summary of the changes and improvements made in the planner:
* ROS Melodic and Gazebo 9 compatibility added.
* Provisions for stopping the car have been added.
* Reduced the time spent on generating a path (by 5-6 times) through parallel processing and other optimisations.
* Average frequency of over 5 Hz achieved.
* Fixed the issue of ghost obstacles caused due to faulty lidar data processing by car_demo.

A short [video](https://youtu.be/BPBGQtXIR2Q) of a demo run.  
# Frenet Planner ROS (Readme of the original repo)

## Introduction
This is an implementation for a sampling based planner in frenet frame. 
The [**original paper**](https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame) discusses about converting normal coordinate frame to frenet frame & sampling of paths, selecting the them based on cost associated with each path. 

### Use Case
The planner has been tested in various environments. But is formed & found to be useful for high speed scenarios with minimal curves in path. This converts to highway scenario in the the physical world.

There are a lot of cases that require dynamic consideration of other traffic participants for complex maneuvers like merging into traffic flow, passing with on-
coming traffic, changing lanes, or avoiding other vehicles. 

Heuristics based planners sort all these quite easily. But in cases of time sparsity these planners don't perform well. So there in comes the concept of taking time **'t'** into consideration at planning & execution level. This is what the planner handles through taking maneuver time into account in sampling.  

## Requirements
### Dependencies 
 Based on whether you have Ubuntu 16 or 18, the packages you require changes. The first options are for Ubuntu 16 while the 2nd ones are for Ubuntu 18.
- ROS [Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)/[Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
 - Gazebo [8/9](https://medium.com/@abhiksingla10/setting-up-ros-kinetic-and-gazebo-8-or-9-70f2231af21a)
 - car_demo [8](https://drive.google.com/open?id=1c7gM1AfW6i5L6ZNBFWT8nnvKydPoz3C-)/[9](https://github.com/osrf/car_demo)

### Running Instructions
The following commands are to be run from your terminal:
    $ roscore
    $ roslaunch car_demo demo.launch
    $ roslaunch frenet_planner frenet_planner.launch   
    $ python ~ catkin_ws/src/tracking_control/src/scripts/tracking_Methods/pure_pursuit.py  
    $ python ~ catkin_ws/src/tracking_control/src/scripts/controllers/PID_MIT_velocity_controller.py

# Further Research Aspects
### Incorporation of vehicle dynamics 
The original paper doesn't take the vehicle dynamics(Max. Acceleration & Turning Radius) into account. We have implemented a check function to prevent generation of paths with yaw greater than turning radius of our car. But a mathematical approach to incorporate this in the cost is still to be thought of.
### Robust & Optimized Obstacle Checking
Currently we are iterating through the entire cost map for checking presence of obstacles. But this approach, though foolproof, isn't optimized & requires a lot of computation, specially in the urban environments.
### Training of Sampling Parameters
Our main task for the planner is to find the best suited path. In the original paper this was done through sampling. But on close observation & after checking it's mathematical validity, we are trying to make an Reinforced Learning based model learn the same on it's own. This would thus reduce the computation by a lot while maintaining the stability of the planner.  
# Resources
- [Original Paper](https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame) 
- [Notes on the Paper](https://drive.google.com/file/d/1ZkUTlOpAZ7df4IcZ8Ny91uF-eHg1mh4t/view?usp=sharing)
- [Simple Python Implementation](https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/FrenetOptimalTrajectory)
- [Video](https://youtu.be/sv6ST721SI4) of simulation

Contributors:
- [Sohan Rudra](https://github.com/rudrasohan)
- [Siddhant Agarwal](https://github.com/agarwalsiddhant10)
- [Arnesh Kumar Issar](https://github.com/thefatbandit)
- [Rutav Shah](https://github.com/ShahRutav)
- [Atharva Naik](https://github.com/atharva-naik)
