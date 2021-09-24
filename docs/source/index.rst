.. Agent Simulation documentation master file, created by
   sphinx-quickstart on Thu May 28 14:28:26 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

AGV's Frenet Frame Planner Documentation
==================================================

.. image:: /figures/Title.png
  :alt: Trajectory Planner Title Image


Introduction
============

This is an implementation for a sampling based planner in frenet frame. 
The `original paper <https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame>`_ discusses about converting normal coordinate frame to frenet frame & sampling of paths, selecting the them based on cost associated with each path. 

Use Case
========

The planner has been tested in various environments. But is formed & found to be useful for high speed scenarios with minimal curves in path. This converts to highway scenario in the the physical world.

There are a lot of cases that require dynamic consideration of other traffic participants for complex maneuvers like merging into traffic flow, passing with on-coming traffic, changing lanes, or avoiding other vehicles. 

Heuristics based planners sort all these quite easily. But in cases of time sparsity these planners don't perform well. So there in comes the concept of taking time **'t'** into consideration at planning & execution level. This is what the planner handles through taking maneuver time into account in sampling.  

A video of the test run can be found `here <https://youtu.be/UnL6ZROvW4s>`_.

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

   start/overview.rst
   start/installation.rst
   start/launching.rst

.. toctree::
   :maxdepth: 2
   :caption: Software Explanation:
   :glob:

   software/config.rst   
   software/logs.rst   
   software/polynomials.rst
   software/splines.rst
   software/planner.rst

.. note:: Further details about the actual implementation and the purpose of individual functions can be found in the
    :doc:`graph_ltpl/modules`.


.. toctree::
   :maxdepth: 2
   :caption: Code Documentaion:

   graph_ltpl/modules.rst


Contributions
=============
[1] T. Stahl, A. Wischnewski, J. Betz, and M. Lienkamp,
“Multilayer Graph-Based Trajectory Planning for Race Vehicles in Dynamic Scenarios,”
in 2019 IEEE Intelligent Transportation Systems Conference (ITSC), Oct. 2019, pp. 3149–3154.
`(view pre-print) <https://arxiv.org/pdf/2005.08664>`_

If you find our work useful in your research, please consider citing:

.. code-block:: latex

   @inproceedings{stahl2019,
     title = {Multilayer Graph-Based Trajectory Planning for Race Vehicles in Dynamic Scenarios},
     booktitle = {2019 IEEE Intelligent Transportation Systems Conference (ITSC)},
     author = {Stahl, Tim and Wischnewski, Alexander and Betz, Johannes and Lienkamp, Markus},
     year = {2019},
     pages = {3149--3154}
   }

Contact Information
===================

:Email: `tim.stahl@tum.de <tim.stahl@tum.de>`_
