=============================
Installation 
=============================
This is a brief tutorial how to prepare your workspace to execute the source code of the local planner.

All the code was tested with **Python 3.7**, newer versions should also work. The code was tested for compatibility with
**Linux (Ubuntu 16.04 and Ubuntu 18.04)**

Open a terminal inside the folder in which you have git cloned the repo. Source your ROS installation and then run the following ROS command to build the entire workspace. 

.. code-block:: bash

    catkin build

Dependencies
================

The following dependencies are required for Ubuntu 18.04 .

* ROS Melodic
* Gazebo 9
* car_demo 9

Installation tutorials for ROS

* `http://wiki.ros.org/melodic/Installation/Ubuntu <http://http://wiki.ros.org/melodic/Installation/Ubuntu>`_

After ROS has been set up run the following commands (The given commands are for ROS Melodic) -

.. code-block:: bash

    sudo apt install ros-melodic-catkin python-catkin-tools
    sudo apt install ros-melodic-costmap-converter
    sudo apt install ros-melodic-base-local-planner
    curl -sSL http://get.gazebosim.org | sh
    sudo apt upgrade libignition-math2
    sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
    sudo apt-get install ros-melodic-move-base
    sudo apt-get install ros-melodic-fake-localization
    sudo apt install pip
    pip install pyyaml
    pip install rospkg



