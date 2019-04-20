# Constrained Manipulability
Constrained Manipulability is a library used to compute and vizualize a robot's constrained capacities. 




## Features
 - Compute a robot's constrained allowable Cartesian motions due to collision avoidance constraints and joint limits
 - Compute a robot's constrained manipulability polytope due to dangerfield constraints
 - Static functions that allow the above quantities to be used in optimization algorithms for collision free trajectory optimization

## Installation 
### Dependencies
- [ROS](http://wiki.ros.org/catkin) 
    - [eigen_conversions](http://wiki.ros.org/eigen_conversions) 
    - [geometric_shapes](http://wiki.ros.org/geometric_shapes)
    - [pcl_ros](http://wiki.ros.org/pcl_ros)
    - [kdl_parser](https://wiki.ros.org/kdl_parser)
- [Eigen 3](https://eigen.tuxfamily.org/dox/GettingStarted.html)
- [robot_collision_checking](https://github.com/philip-long/ros_collision_checking)
- [eigen-cddlib](https://github.com/philip-long/eigen-cddlib)


### Install instructions
1. Clone repo into your current workspace

    ```
    cd catkin_ws/src
    git clone https://github.com/philip-long/constrained_manipulability.git
    cd ..
    catkin build
    ```


### Examples
Demos can be launched for a robot, using the provided test file
```
roslaunch constrained_manipulability abstract_robot.launch root_link:=<your root link> root_link:=<your end effector>  config:=<your scene stored in .yaml>
```
There are several example scenes in scene_config folder. This has been tested with Rethink's Sawyer, Universal robot (requires ur-description), and Frannka Panda (requires franka-description): 
```
 roslaunch constrained_manipulability sawyer_test.launch
 roslaunch constrained_manipulability ur_test.launch
 roslaunch constrained_manipulability franka_test.launch
```


## Usage:
The main object is initialized as follows:
```
    ConstrainedManipulability ( ros::NodeHandle nh,
                                std::string root,
                                std::string tip,
				std::string robot_description="robot_description",
                                double distance_threshold=0.3,
                                double linearization_limit=0.1,
                                double dangerfield=10
                              );
```

It reads a kinematic chain from the parameter server starting a the root and running until the tip. The joint position and velocity limits are also read and are used to define the different polytopes. The collision model of the report is also pared. A collision world is maintained, objects can be added using ros_shape_msgs and Eigen::Affine3d

```
bool addCollisionObject ( const shape_msgs::SolidPrimitive & s1,
                              const  Eigen::Affine3d  & wT1,unsigned int object_id );
bool addCollisionObject ( const shape_msgs::Mesh & s1,
                              const  Eigen::Affine3d  & wT1,unsigned int object_id );
```
Objects are removed by id.
```
bool removeCollisionObject (unsigned int object_id );
```
The polytopes are calculated by obtaining the minimum distance from each link on the robot to objects in the collision world. FCL is used to compute these distance and access via the interface package [robot_collision_checking]((https://github.com/philip-long/ros_collision_checking)). The volume of the polytopes in _Cartesian_ space is return from the get functions as follows:

```
    double getConstrainedAllowableMotionPolytope ( const sensor_msgs::JointState & joint_states,
            Eigen::MatrixXd & AHrep,
            Eigen::VectorXd & bhrep,
            bool show_polytope,
            std::vector<double>  color_pts= {0.0,0.0,0.5,1.0},
            std::vector<double>  color_line= {0.0,0.0,1.0,0.8} );
```
AHrep and bhrep represent the joint space polytope constraints i.e.
```
AHrep*dq <= bhrep
```
conversion from H-representation to V-representation is achieved using the Double Description method accessed via [eigen-cddlib]((https://github.com/philip-long/eigen-cddlib)). Static functions are also available for all the different polytopes where, both the urdf model, the KDL chain and the object set must be explicity passed:
```
    static double getConstrainedAllowableMotionPolytope ( KDL::Chain &  chain,
            urdf::Model & model,
            FCLObjectSet objects,
            const sensor_msgs::JointState & joint_states,
            Eigen::MatrixXd & AHrep,
            Eigen::VectorXd & bhrep,
            double linearization_limit=0.1,
            double distance_threshold=0.3 );
```

Different Polytopes are available more information about allowable motion polytope is available here __Optimization-Based Human-in-the-Loop Manipulation  Using Joint Space Polytopes, Long et al 2019__ more information about the constrained velocity polytope is available here __Evaluating Robot Manipulability in Constrained Environments by Velocity Polytope Reduction Long et al 2018.__ 


### Applications:
A video showing the applications of the constrained allowable motion polytope is available [here](https://youtu.be/oeqj-m25t9c). A video showing the uses of the constrained velocity polytope for humanoid robots can be seen [here](https://www.youtube.com/watch?v=1Nouc4f_rIY) and [here](https://www.youtube.com/watch?v=FzlhsLH5IPU).





#### 1. Motion planning
Planning collision free paths can be achieved by maximizing the volume of the allowable motion polytope, however since no analytical gradient is available this is typically slower than other motion planning algorithms. Nevertheless, since the polytopes are returned they can be used for fast on-line inverse kinematic solutions and guard teleoperation. 

![Planning collision free path by maximizing volume](doc/trajplanning.png)

#### 2. Guarded tele-operation
The polytopes are convex constraints that represent feasible configuration for the whole robot. By respecting them a guaranteed feasible inverse kinematic solution can be obtained very quickly, this can be useful for generating virtual fixtures for teleoperation tasks. The polytope can be vizualized (in red below) showing an operator the Cartesian motions available at all times due to joint limits, kinematic constraints and obstacles in the workspace. The original polytope is shown below in blue/

![Comparison of UR's allowable motions with and without constraints](doc/ur.png)

#### 3. Workspace Analysis
By evaluating the volume of the CMP at points in the workspace, a reachability map can be obtained see this [video](https://youtu.be/jc7X4WakdoE)


![Planar 2DOF workspace analysis](doc/wksp2.png) ![Humanoid workspace analysis](doc/wrkspval.png)


## Citing
If you use this package, please cite either :
```
@inproceedings{long2019optimization,
  title={Optimization-Based Human-in-the-Loop Manipulation  Using Joint Space Polytopes},
  author={{Philip Long, Tar{\i}k Kele\c{s}temur, Aykut \"{O}zg\"{u}n \"{O}nol and Ta\c{s}k{\i}n Pad{\i}r },
  booktitle={2019 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2019},
  organization={IEEE}
}
```

or 

```
@INPROCEEDINGS{8624962,
author={P. {Long} and T. {Padir}},
booktitle={2018 IEEE-RAS 18th International Conference on Humanoid Robots (Humanoids)},
title={Evaluating Robot Manipulability in Constrained Environments by Velocity Polytope Reduction},
year={2018},
volume={},
number={},
pages={1-9},
doi={10.1109/HUMANOIDS.2018.8624962},
ISSN={2164-0580},
month={Nov},}
```

#### TODO: 
- [ ] Add sample optimization
- [ ] Add server to add and remove objects 


