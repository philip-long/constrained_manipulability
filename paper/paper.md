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
date: 20 January 2025
bibliography: paper.bib
---

# Summary

This paper presents `constrained_manipulability`, a C++ library to compute and visualize a robot manipulator's constrained motion capacities. Manipulability polytopes provide a geometrical tool to evaluate the volume of free space surrounding the robotic arm when considering both environmental and intrinsic robot constraints, such as collisions and joint limits, respectively. Moreover, the polytopes defined by these convex constraints represent feasible configurations for the whole robot, that can be utilized in inverse kinematics (IK) optimization algorithms to produce collision-free motion trajectories.

The library is encapsulated in a Robot Operating System (ROS) package [@Quigley2009ROS]. We include ROS 1 and ROS 2 [@Macenski2022ROS2] implementations of the core C++ library. The `constrained_manipulability` package also heavily depends on our `robot_collision_checking` package [@Zolotas2025ROSFCL], a ROS interface for collision checking via the Flexible Collision Library (FCL) [@Pan2012FCL].

The main program of the `constrained_manipulability` package reads a Unified Robot Description Format (URDF) kinematic chain from a user-defined root of the robot (e.g., the robot's base frame) to a tip (e.g., the end-effector or tool). Joint position and velocity limits are also read from the URDF, while a collision world is maintained to express environmental constraints on the robot's motion. A variety of collision objects can be added or removed using common ROS message types, such as OctoMaps [@Hornung2013Octomap], mesh files, or solid primitives. These joint limit and obstacle constraints are then stacked together to define different polytopes, such as the allowable motion polytope [@Long2019Optimization] and the constrained velocity polytope [@Long2018Humanoids].

Polytopes computed by `constrained_manipulability` are published as a `visualization_msgs::msg::MarkerArray` ROS message. A `visualization_msgs::msg::Marker::TRIANGLE_LIST` represents the polytope facets and a `visualization_msgs::msg::Marker::SPHERE_LIST` represents the vertices. As a result, the manipulability polytopes  calculated by our package for a given robot manipulator can also be visualized using standard RViz [@kam2015rviz] tools. Additionally, the generated polytopes are published in vertex and hyperplane form, which is easily interpretable by third-party libraries.

# Statement of Need

Generating complex constraint geometries efficiently is a significant challenge for real-time robot manipulation. For applications, like motion planning or remote teleoperation, being able to represent the space of obstacle-free allowable motion around an end-effector is vital. Given such a representation a collision-free IK solution can be obtained to maximize the free space of a robot moving in a constrained environment. Manipulability polytopes represent manipulability capacities, e.g., the capacity for the robot to transmit velocities and forces from the configuration space to the task space. The benefit of using convex polytopes is that they capture exact velocity bounds, rather than the approximation provided by for example ellipsoids [@Yoshikawa1984Analysis]. While the aforementionned ellipsoids have been the dominant paradigm due to historic computational constraints, more efficient polytope generation methods [@Skuric2023; @Sagar2023] coupled with more computation availability has led to an increased usage. Furthermore, since a polytope is defined by a set of inequality constraints, additional constraints can be easily incorporated into existing polytopes, e.g., mobile robot toppling constraints [@rasheed2018tension], friction cones [@caron2016zmp], and maximum danger values. 

The `constrained_manipulability` ROS 2 package aims to fulfil this need for constrained robot capacity calculation by supplying the robotics community with a fast C++ implementation that computes various types of polytopes. These geometrical constructs can also be visualized for workspace analysis or to guide an operator through the Cartesian motions available due to joint limits, kinematic constraints, and obstacles in the workspace, as illustrated in \autoref{fig:traj_planning}. The utility of these visualizations has proven advantageous in remote teleoperation scenarios involving virtual reality headsets [@Zolotas2021Motion]. Moreover, the package interfaces with the `robot_collision_checking` package [@Zolotas2025ROSFCL] to perform distance and collision checking between a robot manipulator and the environment.

There are currently few software libraries capable of computing robot manipulator capacities as polytopes while seamlessly interfacing with popular robotics middleware, such as ROS. Among them, the one most similar to `constrained_manipulability` is `pycapacity` [@Skuric2023], which is implemented in Python rather than C++. Unlike `pycapacity`, where velocity and force polytopes are solely computed based on the intrinsic constraints of a kinematic chain, our package accounts for additional constraints, such as positional joint limits and environmental obstacles, to represent the constrained motion space.

![Collision-free path for a robot manipulator visualized as a trajectory of polytopes generated by maximizing the volume of allowable motion around the robot's end-effector.\label{fig:traj_planning}](figures/trajplanning.png)

## Polytope and Manipulator Model
A polytope, $\mathcal{P}$, can be represented as the convex hull of its vertex set, or as a volume bounded by a finite number of half-spaces, known as the $\mathcal{V}$-representation and $\mathcal{H}$-representation, which are denoted as $\mathcal{P}^{V}$ and $\mathcal{P}^{H}$, respectively. Converting between the $\mathcal{V}$ and $\mathcal{H}$ representations can be carried out in several ways, however the `constrained_manipulability` package uses the double description method [@Fukuda1996Double]. 

The `constrained_manipulability` provides two types of polytope representations: 1. constrained motion polytopes, which modify the classic manipulability velocity polytope [@kokkinis1989kinetostatic] to include joint limits and capacity reductions due to nearby obstacles; and 2. allowable motion polytopes, which are a linearization of their constrained counterpart used to generate a measure of free space or allowable motion in which a robot can move while satisfying all constraints. 

In both cases, the polytope is constructed as follows. First, a system of linear inequalities is constructed in joint space, for instance based on joint velocity limits: 
\begin{equation} \label{eq:qh_repr}
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
where $\dot{\mathbf{q}}^{v}_{i}$ denotes the $i^{th}$ vertex of the polytope $\mathcal{Q}$, given $q$ vertices in $n$-dimensional space. A Cartesian polytope, denoted as $\mathcal{MP}$, can then be obtained by transforming the vertices of \autoref{eq:qv_repr} to Cartesian space using the forward kinematics. $\mathcal{MP}$'s vertex set representation of $p$ vertices in 3-dimensional space is given as:
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

For constrained motion polytopes, the following constraints are supported: joint velocity limits, joint velocity damping due to positional joint limits, and joint velocity damping due to obstacle proximity to the kinematic chain. For allowable motion polytopes the following constraints are supported: positional joint limits, positional limits due to obstacle proximity to the kinematic chain, and linearization limits. Increasing the values of the linearization expands the free space virtual fixture at a cost of reduced fidelity. Our implementation uses a scalar linearization limit for all joints that can also be altered at run time, e.g., could be increased to enlarge the solution or shrunk to guide the user towards a defined goal configuration, as described in [@Zolotas2021Motion]. Finally, in our implementation, we select at every instant the point along each link nearest to all environmental obstacles. Hence, the system considers the set of instantaneous collision-free joint motion for each link along the robot's kinematic chain, considering all surrounding obstacles. 

## Block Diagram

Any joint position and velocity limits are extracted from the robot's URDF description via ROS 2 parameter operations. Polytopes are calculated as described above using these limits and by obtaining the minimum distance from each link on the robot to objects in the collision world. FCL [@Pan2012FCL] is required to compute these distances and is accessible via the interface package: `robot_collision_checking` [@Zolotas2025ROSFCL]. Polytopes in Cartesian space can then be returned from getter functions:
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
where `AHrep` and `bHrep` represent joint space polytope constraints. 

A diagram summarizing the `constrained_manipulability` package architecture and ROS 2 communication flow is displayed in \autoref{fig:block_diagram}.

![Block diagram for the constrained_manipulability ROS 2 package.\label{fig:block_diagram}](figures/cm_block_diagram.jpg)

# Conflict of Interest
The authors declare that the research was conducted in the absence of any commercial or financial relationships that could be constructed as a potential conflict of interest.

# Acknowledgements
Mark Zolotas is currently at Toyota Research Institute (TRI), Cambridge, MA, USA. This paper describes work performed at Northeastern University and is not associated with TRI.

Taskin Padir holds concurrent appointments as a Professor of Electrical and Computer Engineering at Northeastern University and as an Amazon Scholar. This paper describes work performed at Northeastern University and is not associated with Amazon.

# References
