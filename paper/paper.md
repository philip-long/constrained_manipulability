---
title: 'constrained_manipulability: A ROS 2 library to Compute and Visualize Constrained Capacities for Robotic Manipulators'
tags:
  - ros
  - robotics
  - kinematics
  - manipulability polytopes
authors:
  - name: Mark Zolotas
    orcid: 0000-0002-7672-940X
    corresponding: true # (This is how to denote the corresponding author)
    equal-contrib: true # (This is how you can denote equal contributions between multiple authors)
    affiliation: 1
  - name: Philip Long
    orcid: 0000-0002-0784-8720
    equal-contrib: true # (This is how you can denote equal contributions between multiple authors)
    affiliation: 2
  - name: Keerthi Sagar
    affiliation: 3
  - name: Taskin Padir
    orcid: 0000-0001-5123-5801
    affiliation: "1, 4"
affiliations:
 - name: Northeastern University, USA (at the time of this work)
   index: 1
 - name: Atlantic Technological University, Ireland
   index: 2
 - name: Irish Manufacturing Research Limited, Mullingar, Ireland
   index: 3
 - name: Amazon Robotics, USA
   index: 4
date: 14 August 2024
bibliography: paper.bib
---

# Summary

This paper presents `constrained_manipulability`, a C++ library to compute and visualize a robot manipulator's constrained motion capacities. Manipulability polytopes provide a geometrical tool to evaluate the volume of free space surrounding the robotic arm when considering both environmental and intrinsic robot constraints, such as collisions and joint limits, respectively. Moreover, the polytopes defined by these convex constraints represent feasible configurations for the whole robot, that can be utilized in inverse kinematics (IK) optimization algorithms to produce collision-free motion trajectories.

The library is encapsulated in a Robot Operating System (ROS) package [@Quigley2009ROS]. We include ROS 1 and ROS 2 [@Macenski2022ROS2] implementations of the core C++ library. The `constrained_manipulability` package also heavily depends on our `robot_collision_checking` package, a ROS interface for collision checking via the Flexible Collision Library (FCL) [@Pan2012FCL].

The main program of the `constrained_manipulability` package reads a kinematic chain from a robot manipulator's Unified Robot Description Format (URDF), from a user defined root of the robot (e.g., the robot's base frame) to a tip (e.g., the end-effector or tool). Joint position and velocity limits are also read from the URDF, while a collision world is maintained to express environmental constraints on the robot's motion. A variety of collision objects can be added or removed using common ROS message types, such as Octomaps [@Hornung2013Octomap], mesh files or solid primitives. These joint limit and obstacle constraints are then stacked together to define different polytopes, such as the allowable motion polytope [@Long2019Optimization] and the constrained velocity polytope [@Long2018Humanoids].

# Statement of Need

Generating complex constraint geometries efficiently is a significant challenge for real-time robot manipulation. For applications, like motion planning or remote teleoperation, being able to represent the space of obstacle-free allowable motion around an end-effector is vital. Given such a representation a collision-free IK solution can be obtained to maximize the free space of a robot moving in a constrained environment. Manipulability polytopes represent manipulability capacities, e.g., the capacity for the robot to transmit velocities and forces from the configuration space to the task space. The benefit of using convex polytopes is that they capture exact velocity bounds, rather than the approximation provided by for example ellipsoids [@Yoshikawa1984Analysis]. While the aforementionned ellipsoids have been the dominant paradigm due to historic computational constraints, more efficient polytope generation methods [@Skuric2023; @Sagar2023] coupled with more computation availability has led to an increased usage.   
Furthermore, since a polytope is defined by a set of inequality constraints, additional constraints can be easily incorporated into existing polytopes, e.g., mobile robot toppling constraints [@rasheed2018tension], friction cones [@caron2016zmp], and maximum danger values. 

The `constrained_manipulability` ROS 2 package aims to fulfil this need for constrained robot capacity calculation by supplying the robotics community with a fast C++ implementation that computes various types of polytopes. These geometrical constructs can also be visualized for workspace analysis or to guide an operator through the Cartesian motions available due to joint limits, kinematic constraints, and obstacles in the workspace, as illustrated in \autoref{fig:traj_planning}. The utility of these visualizations has proven advantageous in remote teleoperation scenarios involving virtual reality headsets [@Zolotas2021Motion]. Moreover, the package provides an interface to the `robot_collision_checking` package and a set of convenience functions for checking the distance/collision between a robot manipulator and the environment.

![Collision-free path for a robot manipulator visualized as a trajectory of polytopes generated by maximizing the volume of allowable motion around the robot's end-effector.\label{fig:traj_planning}](figures/trajplanning.png)

## Polytope and Manipulator Model
A polytope, $\mathcal{P}$, can be represented as the convex hull of its vertex set, or as a volume bounded by a finite number of half-spaces,known as the $\mathcal{V}$-representation and $\mathcal{H}$-representation and denoted as $\mathcal{P}^{V}$ and $\mathcal{P}^{H}$, respectively. Converting between the $\mathcal{V}$ and $\mathcal{H}$ representations can be carried out in several ways, however the `constrained_manipulability` package uses the double description method [@Fukuda1996Double]. 

The `constrained_manipulability` provides two types of polytope representation, Constrained Motion Polytopes which modify the classical manipulability velocity polytope [@kokkinis1989kinetostatic] to include joint limits and capacities reductions due to nearby obstacles and the Allowable Motion Polytope a linearisation of the above to generate a measure of free space or allowable motions in which a robot can move while satisfying all constraints. 

In both cases, the polytope is contructed as follows. First a system of linear inequalities is constructed in joint space, for instance based on the joint velocities limits: \begin{equation} \label{eq:qh_repr}
\begin{split} 
\mathcal{Q}^{H}=
\left[\begin{array}{ccc} 
\mathbb{I}_{n}\\ -\mathbb{I}_{n}\\
\end{array} \right]   \dot{\mathbf{q}} \leq 
\left[\begin{array}{ccc} 
\dot{\mathbf{q}}^{max}\\ -\dot{\mathbf{q}}^{min}\\
\end{array} \right], 
\end{split} 
\end{equation}
where $\dot{\mathbf{q}}^{max}$ and $\dot{\mathbf{q}}^{min}$ are the robot's maximum and minimum joint velocities. Using the double description method, an equivalent polytope in the $\mathcal{V}$-representation (i.e., defined by its vertices) is written as:
\begin{equation} \label{eq:qv_repr}
\mathcal{Q}^{V}=
\{ \begin{array}{cccc} 
\dot{\mathbf{q}}^{v}_{1}, \ \dot{\mathbf{q}}^{v}_{2}, \ \ldots, \ \dot{\mathbf{q}}^{v}_{q} 
\end{array}  \},
\end{equation}
where $\dot{\mathbf{q}}^{v}_{i}$ denotes the $i^{th}$ vertex of the polytope $\mathcal{Q}$, given $q$ vertices in $n$-dimensional space.  A Cartesian polytope, denoted as $\mathcal{MP}$,  can be obtained by transforming the vertices of \autoref{eq:qv_repr} to Cartesian space using the forward kinematics. $\mathcal{MP}$'s vertex set representation of $p$ vertices in 3-dimensional space is given as:
\begin{equation} \label{eq:mpv_task_repr}
\mathcal{MP}^{V}=
\{ \begin{array}{ccc} 
\boldsymbol{\nu}^{v}_{1}, \ \ldots, \ \boldsymbol{\nu}^{v}_{p} \\
\end{array} \}
=
\{ \begin{array}{ccc} 
\mathbf{J}_{n} \dot{\mathbf{q}}^{v}_{1}, \ \ldots, \ \mathbf{J}_{n}\dot{\mathbf{q}}^{v}_{p}
\end{array} \},
\end{equation}
with $\boldsymbol{\nu}^{v}_{j} = \mathbf{J}_{n} \dot{\mathbf{q}}^{v}_{j}$ and $p \leq q$. The convexity of a polytope is preserved under affine transformation, thus a bounded volume of $\mathcal{MP}$ that represents the system's manipulability can easily be obtained to serve as an exact indicator of robot performance.





## Block Diagram

Any joint position and velocity limits are extracted from the robot's URDF description via ROS 2 parameter operations. Polytopes are calculated as described above using these limits and by obtaining the minimum distance from each link on the robot to objects in the collision world. FCL [@Pan2012FCL] is required to compute these distances and is accessible via the interface package [robot_collision_checking](https://github.com/philip-long/ros_collision_checking). Polytopes in Cartesian space can then be returned from getter functions, like:
```
Polytope getConstrainedAllowableMotionPolytope(
  const sensor_msgs::msg::JointState& joint_state,
  bool show_polytope,
  Eigen::MatrixXd& AHrep,
  Eigen::VectorXd& bHrep,
  Eigen::Vector3d& offset_position,
  const std::vector<double>& color_pts,
  const std::vector<double>& color_line);
```
where `AHrep` and `bHrep` represent the joint space polytope constraints defined in \autoref{eq:joint_space_h_repr}. 

A diagram summarizing the `constrained_manipulability` package architecture and ROS 2 communication flow is displayed in \autoref{fig:block_diagram}.

![Block diagram for the constrained_manipulability ROS 2 package.\label{fig:block_diagram}](figures/cm_block_diagram.jpg)

# Acknowledgements

Mark Zolotas is currently at Toyota Research Institute (TRI), Cambridge, MA, USA. This paper describes work performed at Northeastern University and is not associated with TRI.

Taskin Padir holds concurrent appointments as a Professor of Electrical and Computer Engineering at Northeastern University and as an Amazon Scholar. This paper describes work performed at Northeastern University and is not associated with Amazon.

# References
