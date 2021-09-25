.. Agent Simulation documentation master file, created by
   sphinx-quickstart on Thu May 28 14:28:26 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

AGV's Frenet Frame Planner Documentation
==================================================

.. image:: /figures/agv.png
  :alt: AGV Title Image
.. image:: /figures/Title.png
  :alt: Trajectory Planner Title Image

Introduction
============

This is an implementation for a sampling based planner in frenet frame. 
The `original paper <https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame>`_ discusses about converting normal coordinate frame to frenet frame & sampling of paths, selecting the them based on cost associated with each path.

Use Case
========

The planner has been tested in various environments. But is formed & found to be useful for high speed scenarios with minimal curves in path. This converts to highway scenario in the the physical world.

The planner runs at 25 Hz under single-threaded operation on a modern pc.

There are a lot of cases that require dynamic consideration of other traffic participants for complex maneuvers like merging into traffic flow, passing with on-coming traffic, changing lanes, or avoiding other vehicles. 

Heuristics based planners sort all these quite easily. But in cases of time sparsity these planners don't perform well. So there in comes the concept of taking time **'t'** into consideration at planning & execution level. This is what the planner handles through taking maneuver time into account in sampling.  

A video of the test run can be found `here <https://youtu.be/UnL6ZROvW4s>`_.

.. image:: /figures/Title_2.png
  :alt: Simulator Run Image
  
.. warning::
   This software is provided *as-is* and has not been subject to a certified safety validation. Autonomous Driving is a
   highly complex and dangerous task. In case you plan to use this software on a vehicle, it is by all means required
   that you assess the overall safety of your project as a whole. By no means is this software a replacement for a valid
   safety-concept. See the license for more details.

Contents
========

.. toctree::
   :maxdepth: 2
   :caption: Getting Started:

   start/installation.rst
   start/launching.rst

.. toctree::
   :maxdepth: 2
   :caption: Software Explanation:
   :glob:

   software/config.rst     
   software/polynomials.rst
   software/splines.rst
   software/planner.rst

.. toctree::
   :maxdepth: 2
   :caption: Code Documentaion:

   code/documentation.rst


Contributions
=============
[1]AGV Planning Module
[2]Werling, Moritz, Julius Ziegler, Sören Kammel, and Sebastian Thrun. "Optimal trajectory generation for dynamic street scenarios in a frenet frame." In 2010 IEEE International Conference on Robotics and Automation, pp. 987-993. IEEE, 2010.
`(view the paper) <https://ieeexplore.ieee.org/abstract/document/5509799/>`_



Contact Information
===================

:Website: `agv.iitkgp.ac.in <http://www.agv.iitkgp.ac.in>`_
