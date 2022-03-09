Flexible Behavior Trees
=======================

Flexible Behavior Trees provides the integration of Behavior Trees (BTs) into the HFSM behavior engine FlexBE.  The packages is derived from the [ROS2 Navigation2] nav2_behavior_tree package.
Flexible Behavior Trees uses the BehaviorTree.CPP framework for developing and loading
BTs as XML files and interfaces with the [FlexBE App] and [FlexBE Behavior Engine] with state
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

The Flexible Behavior Trees system has been tested using the latest version of ROS2 Foxy. You
should first follow the [ROS2 Install Guide] and get that set up before proceeding.

After installing ROS2, git clone this package into a ROS2 workspace and build the packages using
the automated builder colcon.

## Publications

Please use the following publication for reference when using Flexible Behavior Trees:

- Currently under review


### Further Publications for FlexBE

- Philipp Schillinger, Stefan Kohlbrecher, and Oskar von Stryk, ["Human-Robot Collaborative High-Level Control with Application to Rescue Robotics"](http://dx.doi.org/10.1109/ICRA.2016.7487442), IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, May 2016.

- Stefan Kohlbrecher et al. ["A Comprehensive Software Framework for Complex Locomotion and Manipulation Tasks Applicable to Different Types of Humanoid Robots."](http://dx.doi.org/10.3389/frobt.2016.00031) Frontiers in Robotics and AI 3 (2016): 31.

- Alberto Romay et al., [“Collaborative autonomy between high-level behaviors and human operators for remote manipulation tasks using different humanoid robots,”](http://dx.doi.org/10.1002/rob.21671) Journal of Field Robotics, September 2016.


License
-------

	Copyright (c) 2022
	Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLab)
	Christopher Newport University

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	  1. Redistributions of source code must retain the above copyright notice,
	     this list of conditions and the following disclaimer.

	  2. Redistributions in binary form must reproduce the above copyright
	     notice, this list of conditions and the following disclaimer in the
	     documentation and/or other materials provided with the distribution.

	  3. Neither the name of the copyright holder nor the names of its
	     contributors may be used to endorse or promote products derived from
	     this software without specific prior written permission.

	     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	     COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
	     WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	     POSSIBILITY OF SUCH DAMAGE.

[FlexBE App]: https://github.com/CNURobotics/flexbe_app.git
[FlexBE Behavior Engine]: https://github.com/CNURobotics/flexbe_behavior_engine.git
[ROS2 Install Guide]: https://docs.ros.org/en/foxy/Installation.html
[ROS2 Navigation2]: https://github.com/ros-planning/navigation2
