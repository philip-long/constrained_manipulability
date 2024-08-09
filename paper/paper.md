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
  - name: Taskin Padir
    orcid: 0000-0001-5123-5801
    affiliation: "1, 3"
affiliations:
 - name: Northeastern University, USA (at the time of this work)
   index: 1
 - name: Atlantic Technological University, Ireland
   index: 2
 - name: Amazon
   index: 3
date: 09 August 2024
bibliography: paper.bib
---

# Summary

This paper presents `constrained_manipulability`, a C++ library to compute and visualize a robot manipulator's constrained motion capacities during manipulation tasks. Manipulability polytopes are used to represent these capacities, providing a geometrical tool to evaluate the volume of free space surrounding the robotic arm when considering both environmental constraints and intrinsic robot constraints, such as collision with the environment and a manipulator's joint limits, respectively. Moreover, the polytopes defined by these convex constraints represent feasible configurations for the whole robot, that can be utilized in inverse kinematics (IK) optimization algorithms to produce collision-free motion trajectories.

The library is encapsulated in a Robot Operating System (ROS) package [@Quigley2009ROS], given that the robotics community widely relies on ROS as the standard for software development. We include ROS 1 and ROS 2 [@Macenski2022ROS2] implementations of the core C++ library. The `constrained_manipulability` package also heavily depends on our `robot_collision_checking` package, a ROS interface for collision checking via the Flexible Collision Library (FCL) [@Pan2012FCL].

The main program of the `constrained_manipulability` package reads in a kinematic chain from a robot manipulator's Unified Robot Description Format (URDF), traversing from a user defined root of the robot (e.g. robot base frame) to a tip (e.g. end-effector or tool). Joint position and velocity limits are also read into the program using this URDF input, while a collision world is maintained to express environmental constraints on the robot's motion. A variety of collision objects can be added or removed using common ROS message types, such as Octomaps [@Hornung2013Octomap], mesh files or solid primitives. These joint limit and obstacle constraints are then stacked together to define different polytopes, such the allowable motion polytope [@Long2019Optimization] and the constrained velocity polytope [@Long2018Humanoids].

# Statement of Need

Generating complex constraint geometries in an efficient and effective manner is a significant challenge for real-time robot manipulation. For common applications within manipulation, like motion planning or remote teleoperation, being able to represent the space of allowable motion around a robot's end-effector is a vital capability. Given such a representation of the obstacle-free volume around the robot's end-effector, a collision-free IK solution can be obtained to maximize the free space of a robot moving in a constrained environment. Manipulability polytopes represent manipulability capacities, e.g., for the robot to transmit velocities and forces from the configuration space to the task space. The benefit of using convex polytopes is that they capture exact velocity bounds, rather than the approximation provided by other typical choices for performance indicators, such as ellipsoids [@Yoshikawa1984Analysis]. While the aforementionned ellipsoids have been the dominant paradigm due to historic computational constraints, more efficicent polytope generation methods [@Skuric2023] [@Sagar2023] coupled with more computation availability has led to an increased usage.   
Furthermore, since a polytope is defined by a set of inequality constraints, additional constraints can be easily incorporated into existing polytopes, e.g., mobile robot toppling constraints [@rasheed2018tension], friction cones [@caron2016zmp] or maximum danger values. 

The `constrained_manipulability` ROS 2 package aims to fulfil this need for constrained robot capacity calculation by supplying the robotics community with a fast C++ implementation that computes various types of polytopes. These geometrical constructs can also be visualized for workspace analysis or to guide an operator through the Cartesian motions available due to joint limits, kinematic constraints, and obstacles in the workspace, as illustrated in \autoref{fig:traj_planning}. The utility of these visualizations has proven advantageous in remote teleoperation scenarios involving virtual reality headsets [@Zolotas2021Motion]. Moreover, the package provides an interface to the `robot_collision_checking` package and a set of convenience functions for checking the distance/collision between a robot manipulator and the environment.

![Collision-free path for a robot manipulator visualized as a trajectory of polytopes generated by maximizing the volume of allowable motion around the robot's end-effector.\label{fig:traj_planning}](figures/trajplanning.png)

# Manipulability Polytopes Calculation

A polytope, $\mathcal{P}$, can be represented in two ways: as the convex hull of its vertex set, known as the $\mathcal{V}$-representation and denoted as $\mathcal{P}^{V}$, or as volume bounded by a finite number of half-spaces, known as the $\mathcal{H}$-representation and denoted as $\mathcal{P}^{H}$. These are written respectively as:
\begin{align}
\mathcal{P}^{V} &= \{\mathbf{x}: \mathbf{x}=\sum\limits_{i=1}^{n} \alpha_{i}\mathbf{y}_{i}\Bigg| \alpha_{i}\geq0, \sum\limits_{i=1}^{n}\alpha_{i}=1\}, \\ \mathcal{P}^{H} &= \mathbf{A}\mathbf{x} \leq \mathbf{b},
\label{eq:poly_spaces}
\end{align}
where $\mathbf{y}_{i}$ denotes the $i^{th}$ element of the vertex set, $\mathbf{x}$ is any point inside $\mathcal{P}$, $\mathbf{A}$ contains the half-spaces' normals, and $\mathbf{b}$ is the shifted distance from the origin along the normal. Converting between the $\mathcal{V}$ and $\mathcal{H}$ representations can be carried out in several ways, however the `constrained_manipulability` package uses the double description method [@Fukuda1996Double]. 

Below is a description of how constrained motion polytopes are constructed in `constrained_manipulability` for a given manipulator model.

## Manipulator Model

Consider a serial manipulator with $n$ degrees-of-freedom operating in 6-dimensional space. The end-effector pose is represented by a vector $\mathbf{x}_n \in \mathbb{R}^6$, consisting of the position $\mathbf{x}^p_n \in \mathbb{R}^3$ and unit quaternion orientation $\mathbf{x}^q_n \in \mathbb{R}^3$, all of which can be derived from the forward kinematic chain $\mathit{fk}$:
\begin{equation} \label{eq:fk}
    \mathbf{x}_n = \mathit{fk}(\mathbf{q}),
\end{equation}
where $\mathbf{q}=[q_1, ..., q_n]$ are the joint configuration variables. The twist at the end-effector's $n^{th}$ frame, $\boldsymbol{\nu}_n$, can then be obtained from the differential kinematic model:
\begin{equation} \label{eq:dk}
    \boldsymbol{\nu}_n = \begin{bmatrix} \mathbf{v} \\ \boldsymbol{\omega} \end{bmatrix} = \mathbf{J}_n \dot{\mathbf{q}},
\end{equation}
with $\mathbf{v}$ and $\boldsymbol{\omega}$ denoting translational and angular velocities, respectively. $\mathbf{J}_n$ is the 6$\times n$ Jacobian matrix defined at the end-effector frame $n$ and $\dot{\mathbf{q}} = [\dot{q}_1, ..., \dot{q}_n]^\intercal$ is the joint velocity vector. 

## Allowable Motions

At trajectory step $k$, the end-effector pose and the instantaneous joint velocity are given by $\mathbf{x}^k_n$ and $\dot{\mathbf{q}}^k$, respectively. If the end-effector travels a distance of $\delta \mathbf{x}^k_n$ over a period of $\delta t$ seconds, then the resulting end-effector pose at instant $k+1$ can be obtained by linearizing \autoref{eq:fk} using \autoref{eq:dk}:
 \begin{align}
\mathbf{x}^{k+1}_n &= \mathbf{x}^{k}_n +\delta \mathbf{x}^{k}_n  \approx \mathit{fk}(\mathbf{q}^{k} +  \delta \mathbf{q}^{k}), \label{eq:lin_1} \\
\delta \mathbf{x}^{k}_n &= \mathbf{J}_{n} \delta \mathbf{q}^{k},
\end{align}
where $\delta \mathbf{q}^{k}$ denotes the displacement in joint variables over the timestep and is defined as:
\begin{equation}
\quad \delta \mathbf{q}^{k}&=\dot{\mathbf{q}}^{k} \delta t.
\end{equation}
In a cluttered environment, the end-effector's allowable motion is conditioned on the allowable motion of its attached links. Even if the end-effector is located nominally in free space, certain allowable motions may be restricted by obstacles close to the chain's preceding links.

For a point $\mathcal{I}$ on the robot's kinematic chain whose position with respect to the world frame is denoted by the vector $ \mathbf{r}_{i}$, the displacement due to $\delta \mathbf{q}^{k}$ is written as:
\begin{equation} \label{eq:joint_lin}
\delta \mathbf{x}^{k}_{i}=\mathbf{J}_{i} \delta \mathbf{q}^{k},
\end{equation}
where $\mathbf{J}_{i} \in  \mathbb{R}^{3 \times n} $ denotes the kinematic Jacobian matrix that relates velocities of antecedent joints along the kinematic chain to the translational velocity at point $\mathcal{I}$, with columns of zeros for joints that do not generate velocity at $\mathcal{I}$. 

Using the linearization from \autoref{eq:joint_lin}, limits of translational motion for any point on the manipulator body can be defined based on the location of environmental obstacles. Suppose there is an environmental obstacle $\mathcal{O}$ in the robot's proximity, with a location in the world frame described as $\mathbf{r}_{o}$. Then at instant $k$, the translational motion of point $\mathcal{I}$ towards $\mathbf{r}_{o}$, written as $\delta \mathbf{x}^{k}_{i,o}$, is defined as:
\begin{equation}
\delta \mathbf{x}^{k}_{i,o} = \hat{\mathbf{r}}^\intercal_{io} \mathbf{J}_{i} \delta \mathbf{q}^{k},
\end{equation}
where $\mathbf{r}_{io}= \mathbf{r}_{i} -\mathbf{r}_{o}$ is the relative position between $\mathcal{O}$ and $\mathcal{I}$, $\lVert \mathbf{r}_{io} \rVert$ is the norm of this vector, i.e., the distance, while $\hat{\mathbf{r}}_{io}$ is the corresponding normalized unit vector. Hence, to prevent a collision between $\mathcal{I}$ and $\mathcal{O}$, an allowable motion constraint defining the maximum displacement of $\mathcal{I}$ can be expressed as follows:
\begin{equation} \label{eq:one_obs_one_pt}
\hat{\mathbf{r}}^\intercal_{io} \mathbf{J}_{i} \delta \mathbf{q}^{k} \leq \lVert \mathbf{r}_{io} \rVert.
\end{equation}
This inequality constraint can be repeated for a set of $l$ points discretized along the kinematic chain, leading to the following set of linear inequalities:
\begin{align} \label{eq:one_obs_all_pts}  
\left[\begin{array}{cc} 
\hat{\mathbf{r}}^{T}_{1o} \mathbf{J}_{1}  \\
\vdots   \\
\hat{\mathbf{r}}^{T}_{lo} \mathbf{J}_{l}  \\
\end{array} \right] \delta \mathbf{q}^{k} 
& \leq
\left[\begin{array}{ccc} 
\lVert \mathbf{r}_{1o}\rVert\\
\vdots \\
\lVert \mathbf{r}_{lo}\rVert
\end{array} \right]. 
\end{align}
In our implementation, we select at every instant the point along each link nearest to an environmental obstacle, i.e., $l=n$. Hence, \cref{eq:one_obs_all_pts} represents the set of instantaneous collision-free joint deviations for each link on the robot's kinematic chain, considering obstacle $\mathcal{O}$. We can extend this formulation to $m$ obstacles in the robot's workspace, obtaining $(l \times m)$ constraints to ensure collision-free motions. This can be reduced by neglecting obstacles where $\lVert \mathbf{r}_{io} \rVert$ is greater than a threshold.

Aside from obstacles, a robot's motion is restricted by positional limits of the joints. To integrate these limits into our joint space polytope, the following linear inequalities are included:
 \begin{align} \label{eq:pos_dev_limits}
\left[\begin{array}{ccc} 
\mathbb{I}_{n}\\
-\mathbb{I}_{n}\\
\end{array} \right]   \delta{\mathbf{q}}^{k} \leq
\left[\begin{array}{ccc} 
 \mathbf{q}^{max}-\mathbf{q}^{k}\\
 \mathbf{q}^{k}-\mathbf{q}^{min}\\
\end{array} \right], 
\end{align}
where $\mathbb{I}_{n}$ is the $n \times n$ identity matrix, while $\mathbf{q}^{max}$ and $\mathbf{q}^{min}$ are vectors of upper and lower positional joint limits, respectively.

As joint displacement increases, so does the linearisation error. A maximum linearisation limit is thus imposed:
\begin{align} \label{eq:lin_limits}
\left[\begin{array}{ccc} 
\mathbb{I}_{n}\\
-\mathbb{I}_{n}\\
\end{array} \right]   \delta{\mathbf{q}}^{k} \leq
\left[\begin{array}{ccc} 
 \delta \mathbf{q}^{max}\\
 \delta \mathbf{q}^{max}\\
\end{array} \right],
\end{align}
where $ \delta \mathbf{q}^{max}$ is the vector of linearisation limits for each joint of the kinematic chain. Increasing the values of $\delta \mathbf{q}^{max}$ expands the free space virtual fixture at a cost of reduced fidelity. While not necessary, for simplicity, we let $\delta {q}_{1}=\delta {q}_{2}=\ldots = \delta {q}_{n} = \epsilon_{lin}$ in our implementation. The linearisation limits can also be altered at run time. For example, limits could be increased to enlarge the solution space of feasible configurations. Alternatively, the limits could be shrunk progressively in order to guide the user towards a defined goal configuration.

## Constrained Motion Polytopes

The capacity of a serial manipulator can be obtained by first constructing the joint space polytope in the $\mathcal{H}$-representation according to its velocity limits:
\begin{align} \label{eq:qh_repr}
\mathcal{Q}^{H}=
\left[\begin{array}{ccc} 
\mathbb{I}_{n}\\
-\mathbb{I}_{n}\\
\end{array} \right]   \dot{\mathbf{q}} \leq 
\left[\begin{array}{ccc} 
\dot{\mathbf{q}}^{max}\\
-\dot{\mathbf{q}}^{min}\\
\end{array} \right], 
\end{align}
where $\dot{\mathbf{q}}^{max}$ and $ \dot{\mathbf{q}}^{min}$ are the robot's maximum and minimum joint velocities.

Using the double description method, an equivalent polytope in the $\mathcal{V}$-representation (i.e., defined by its vertices) is written as:
\begin{equation} \label{eq:qv_repr}
\mathcal{Q}^{V}=\{ \begin{array}{cccc} 
\dot{\mathbf{q}}^{v}_{1}, \
\dot{\mathbf{q}}^{v}_{2}, \
\ldots, \
\dot{\mathbf{q}}^{v}_{q} 
\end{array}  \},
\end{equation}
where $\dot{\mathbf{q}}^{v}_{i}$ denotes the $i^{th}$ vertex of the polytope $\mathcal{Q}$, given $q$ vertices in $n$-dimensional space. 

A linear transformation of a polytope produces another polytope. Ergo, a manipulability polytope, denoted as $\mathcal{MP}$, representing the Cartesian-space velocities can then be obtained by transforming the vertices of \autoref{eq:qv_repr} to Cartesian space using \autoref{eq:dk}. $\mathcal{MP}$'s vertex set representation of $p$ vertices in 3-dimensional space is given as:

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

with $\delta \mathbf{x}^v_{j} = \mathbf{J}_{n} \dot{\mathbf{q}}^{v}_{j}$ and $p \leq q$. The convexity of a polytope is preserved under affine transformation, thus a bounded volume of $\mathcal{MP}$ that represents the system's manipulability can easily be obtained to serve as an exact indicator of robot performance.

The above considers a constrained velocity polytope based on the constraints specified in \autoref{eq:qh_repr}. To obtain the constrained motion polytope for a manipulator, a joint space polytope is first defined using position deviations instead of instantaneous velocities, following from its position limits stated in \autoref{eq:pos_dev_limits}. 

Therefore, by stacking \autoref{eq:one_obs_all_pts}, \autoref{eq:pos_dev_limits}, and \autoref{eq:lin_limits}, an $\mathcal{H}$-representation of a joint space polytope that approximates, at any instant $k$, the maximum range of joint displacements with respect to the system's constraints is derived:
\begin{equation}
\mathbf{A}^{k} \delta{\mathbf{q}}^{k} \leq \mathbf{b}^{k}.
\label{eq:joint_space_h_repr}
\end{equation} 
The polytope can then be transformed to a $\mathcal{V}$-representation using the double description method~\cite{Fukuda1996Double}, as in \autoref{eq:qv_repr}, after which the Cartesian task space representation is obtained using the differential kinematic model \autoref{eq:dk}, as in \autoref{eq:mpv_task_repr}. Overall, \autoref{eq:joint_space_h_repr} defines a set of Cartesian displacements for the manipulator's end effector, at an instant $k$, for which an IK solution can be obtained that is collision-free and within the positional joint limits.

## Block Diagram

Any joint position and velocity limits are extracted from the robot's URDF description via a ROS 2 parameter. Polytopes are calculated as described above using these limits and by obtaining the minimum distance from each link on the robot to objects in the collision world. FCL [@Pan2012FCL] is required to compute these distances and is accessible via the interface package [robot_collision_checking](https://github.com/philip-long/ros_collision_checking). Polytopes in <b>Cartesian</b> space can then be returned from getter functions, like:
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

![Block diagram for the constrained_manipulability ROS 2 package.\label{fig:traj_planning}](figures/cm_block_diagram.jpg)

# Acknowledgements

Taskin Padir holds concurrent appointments as a Professor of Electrical and Computer Engineering at Northeastern University and as an Amazon Scholar. This paper describes work performed at Northeastern University and is not associated with Amazon.

# References
