Flexible Behavior Trees
=======================

Flexible Behavior Trees provides the integration of Behavior Trees (BTs) into the HFSM behavior engine FlexBE.
The packages are derived from the [ROS2 Navigation2] nav2_behavior_tree package.
Flexible Behavior Trees uses the BehaviorTree.CPP (v4.6+) framework for developing and loading
BTs as XML files and interfaces with the [FlexBE Behavior Engine] with state
implementations. These HFSM states allow a user to specify the loading and execution of BTs by
sending a BT server the XML file names to load and execute. Moreover, these HFSM BT states
can be chained together to allow for executing multiple BTs in a series while providing for
user supervision and adjustable autonomy to better control contingencies and recovery behaviors.

About
-----

This system provides specific states to load and execute various BTs from robot navigation
to arm manipulation. This system has two main components a BT server and FlexBE state
implementations. The FlexBE state implementations send goals to the BT server to load and
execute a BT with and without a user defined goal. FlexBE orchestrates the execution of
different BTs while the BehaviorTree.CPP framework executes the BTs and passes data between
nodes and BTs.

Install
-------

The Flexible Behavior Trees system has been tested using the latest version of ROS2 Iron. You
should first follow the [ROS2 Install Guide] and get that set up before proceeding.

After installing ROS2, `git clone` this package into a ROS2 workspace and build the packages using
the automated build tool `colcon`.

## Publications

Please use the following publication for reference when using Flexible Behavior Trees:

- Joshua M. Zutell, David C. Conner, and Philipp Schillinger, ["Flexible Behavior Trees: In search of the mythical HFSMBTH for Collaborative Autonomy in Robotics"](https://doi.org/10.48550/arXiv.2203.05389), March 2022.


### Further Publications for FlexBE

- Joshua Zutell, David C. Conner and Philipp Schillinger, ["ROS 2-Based Flexible Behavior Engine for Flexible Navigation ,"](http://dx.doi.org/10.1109/SoutheastCon48659.2022.9764047), IEEE SouthEastCon, April 2022.

- Philipp Schillinger, Stefan Kohlbrecher, and Oskar von Stryk, ["Human-Robot Collaborative High-Level Control with Application to Rescue Robotics"](http://dx.doi.org/10.1109/ICRA.2016.7487442), IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, May 2016.

- Stefan Kohlbrecher et al. ["A Comprehensive Software Framework for Complex Locomotion and Manipulation Tasks Applicable to Different Types of Humanoid Robots."](http://dx.doi.org/10.3389/frobt.2016.00031) Frontiers in Robotics and AI 3 (2016): 31.

- Alberto Romay et al., [“Collaborative autonomy between high-level behaviors and human operators for remote manipulation tasks using different humanoid robots,”](http://dx.doi.org/10.1002/rob.21671) Journal of Field Robotics, September 2016.


[FlexBE Behavior Engine]: https://github.com/CNURobotics/flexbe_behavior_engine.git
[ROS2 Install Guide]: https://docs.ros.org/en/iron/Installation.html
[ROS2 Navigation2]: https://github.com/ros-planning/navigation2
