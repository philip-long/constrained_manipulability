///  @file constrained_manipulability.h
/// \class ConstrainedManipulability
/// \brief Defines the constrained manipulability polytope for a serial chain
///
/// This class reads a robot description from the parameter server and constructs the serial
/// chain, including collision geometery and joint limits. The user can add a set of objects
/// defined by simple primitives and also meshes. The constrained manipulability polytope is caculated
/// which gives a estimated of the amount of Cartesian free space aroud the last link of the serial
/// chain. The free space encode all the Cartesian and joint space constraints. Collision checking
/// is carried out using fcl library and polytopes are displated using rviz.
///
///
/// \author Philip Long <philip.long01@gmail.com>, RiVER Lab Northeastern University
/// \date Mar, 2019


#include <ros/ros.h>
#include <ros/package.h>
#include <cmath>

#include <Eigen/Eigen>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen-cddlib/Polyhedron.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>

#include <robot_collision_checking/fcl_interface.h>
#include "constrained_manipulability/utility_funcs.h"
#include "constrained_manipulability/ObjectDistances.h"
#include "constrained_manipulability/PolytopeMesh.h"
#include "constrained_manipulability/PolytopeVolume.h"
#include <pcl/surface/concave_hull.h>
#ifndef ROBOT_POLYTOPE_HPP
#define ROBOT_POLYTOPE_HPP


/// GeometryInformation contains all geomertic information obtained from chain
struct GeometryInformation {
    std::vector<std::unique_ptr<shapes::Shape>> shapes;
    TransformVector geometry_transforms;
    JacobianVector  geometry_jacobians;


    void clear() {
        shapes.clear();
        geometry_transforms.clear();
        geometry_jacobians.clear();
    }

};



class ConstrainedManipulability
{
private:
    /// number of degrees of freedom
    unsigned int ndof_;
    ros::NodeHandle nh_;
    /// RVIZ DISPLAY
    ros::Publisher mkr_pub;
    ros::Publisher poly_mesh_pub;
    ros::Publisher obj_dist_pub;

    /** Plot a Polytope defined a a set of vertices
    *   The vertices are shifted to the offset position (offset_position), for instance the robot end effector
    *   std::string frame, to define the frame of reference.
    *   PCL is used to obtain the convex hull
    *   A mesh message is created from the convex hull and published to RVIZ
    *   The facet color is defined by color_line, while points by color_pts
    *   The return value is the volume of the polytope
    */
    bool plotPolytope ( std::string polytope_name,
                        Eigen::MatrixXd  vertices,
                        std::string frame,
                        Eigen::Vector3d offset_position,
                        std::vector<double> color_pts= {1.0,0.4,0.4,1.0},
                        std::vector<double> color_line= {1.0,0.4,0.4,1.0}
                      );
    /// Display a marker, in a given reference frame defined by a visualization_msgs::Marker
    bool displayMarker ( visualization_msgs::Marker mkr,
                         const Eigen::Affine3d & T,
                         std::string frame,
                         unsigned int obj_id,
                         const Eigen::Vector4d & color );
    
    /// Collision checker
    FCLInterface fclInterface;

    /// Robot base link
    std::string base_link_;
    /// Distance threshold beyond which objects are ignored
    double distance_threshold_;
    /// Joint and linearization limits
    std::vector<double> qmax_,qmin_,qdotmax_,qdotmin_,max_lin_limit_,min_lin_limit_;

    /// desired Dangerfield value
    double dangerfield_;


    bool wait_for_rviz;

    KDL::Tree my_tree_;
    KDL::Chain chain_;
    urdf::Model model_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver>  kdl_dfk_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver_;

    Eigen::MatrixXd ndof_identity_matrix_;


    /// Convert a joint state message to a KDL joint array based on segment names
    void jointStatetoKDLJointArray ( const sensor_msgs::JointState & joint_states,
                                     KDL::JntArray & kdl_joint_positions );
    /// Statically convert a joint state message to a KDL joint array based on segment names
    static void jointStatetoKDLJointArray ( KDL::Chain & chain, const sensor_msgs::JointState & joint_states,
                                            KDL::JntArray & kdl_joint_positions );

    /** This function is taken from https://github.com/tu-darmstadt-ros-pkg with a tiny change to smart ptrs
    * https://github.com/tu-darmstadt-ros-pkg/robot_self_filter/blob/master/src/self_mask.cpp#L76
    **/
    static std::unique_ptr<shapes::Shape> constructShape ( const urdf::Geometry *geom );


    /// Convenience function to convert kdl to eigen stuff, segment=-1 returns terminal point information
    void   getKDLKinematicInformation ( const KDL::JntArray & kdl_joint_positions,
        Eigen::Affine3d & T,
        Eigen::Matrix<double,6,Eigen::Dynamic> & Jac,int segment=-1);
    
    
protected:


    /** getCollisionModel returns  kinematic information about collision geometry
     * For ith link (segment) in the kinematic serial chain, we return
     * GeometryInformation:
     *          shapes[i]: the shape description of geometery [i]
     *          geometry_transforms[i]: the transform of the collision geometry's [i] origin
     *          geometric_jacobians[i]: the jacobian matrix in the base frame at the collision geometry's [i] origin
     */
    bool getCollisionModel ( const  KDL::JntArray & jointpositions,
                             GeometryInformation & geometry_information);


    /** getPolytopeHyperPlanes returns hyperplanes for constrained joint polytope
    * For ith link (segment) in the kinematic serial chain, we return
    *  AHrep is the normal to he half spaces
    *  bhrep constains the shifted distance from the origin along the normal
    * Polytope is then defined as P= { A x <= b }
    * velocity_polytope true means returns hyperplanes based on velocity and dangerfield
    *                   false means the free spoace approximation is returned.
    */
    bool getPolytopeHyperPlanes (
        const  KDL::JntArray & jointpositions,
        GeometryInformation & geometry_information,
        Eigen::MatrixXd & AHrep,
        Eigen::VectorXd & bhrep,
        bool velocity_polytope=false
    );


public:
    enum SLICING_PLANE{
        YZ_PLANE=0,
        XZ_PLANE=1,
        XY_PLANE=2,
    };

    ConstrainedManipulability ( ros::NodeHandle nh,
                                std::string root,
                                std::string tip,
                                std::string robot_description="robot_description",
                                double distance_threshold=0.3,
                                double linearization_limit=0.1,
                                double dangerfield=10
                              );
    // Calls fcl destructor which should destroy all objects in world
    ~ConstrainedManipulability();


    /// Add a solid primitive object to FCLInterface collision world transform w.r.t base_link of chain
    bool addCollisionObject ( const shape_msgs::SolidPrimitive & s1,
                              const  Eigen::Affine3d  & wT1,unsigned int object_id );
    /// Add a mesh object to FCLInterface collision world transform w.r.t base_link of chain
    bool addCollisionObject ( const shape_msgs::Mesh & s1,
                              const  Eigen::Affine3d  & wT1,unsigned int object_id );
     /// Remove an object from FCLInterface collision world
    bool removeCollisionObject (unsigned int object_id );
    
    /// Add a set of object to FCLInterface collision world transform w.r.t base_link of chain
    bool addCollisionObject ( FCLObjectSet objects );
    
    
    
    /// Display Collision world objects
    bool displayObjects();
    /// Display calculated collision model in rviz
    bool displayCollisionModel ( sensor_msgs::JointState const & joint_state );        
    /// Checks the current state for collision
    bool checkCollision ( const sensor_msgs::JointState & joint_states );

    void setRvizWait(bool flag);
    bool const getRvizWait(bool flag);

    /// Convenience function to get end-effector pose as a geometry_msgs::Pose
    void  getCartPos(const sensor_msgs::JointState & joint_states,
        geometry_msgs::Pose& geo_pose);

    // yz plane = 0 
    // xz plane = 1 
    // xy plane = 2
    bool slicePolytope(const Eigen::MatrixXd &
        Vset, Eigen::Vector3d offset_position,
        std::vector<double>  color_pts,
        std::vector<double>  color_line,
        std::string polytope_name="sliced_polytope",
        ConstrainedManipulability::SLICING_PLANE index=ConstrainedManipulability::SLICING_PLANE::XY_PLANE,
        double plane_width=0.002 );
    
    /** getConstrainedAllowableMotionPolytope returns the polytope
    *   approximating the constrained allowable end effector motion, considering
    *  joint limits and objstacles & linearization
    *
    *  const sensor_msgs::JointState & joint_states, current joint states
    *  show_polytope -> plots the polytope in rviz
    *  color_pts -> polytope points color
    *  color_line  -> polytope lines color
    *  returns the volume of the constrained allowable end effector motion
    */
    double getConstrainedAllowableMotionPolytope ( const sensor_msgs::JointState & joint_states,
            bool show_polytope,
            std::vector<double>  color_pts= {0.0,0.0,0.5,1.0},
            std::vector<double>  color_line= {0.0,0.0,1.0,0.8} );

    /** getConstrainedAllowableMotionPolytope returns the polytope
    *   approximating the constrained allowable end effector motion, considering
    *  joint limits and objstacles & linearization
    *
    *  AHrep hyperplanes of the constrained allowable motion polytope
    *  bHrep shifted distance
    *  const sensor_msgs::JointState & joint_states, current joint states
    *  show_polytope -> plots the polytope in rviz
    *  color_pts -> polytope points color
    *  color_line  -> polytope lines color
    *  returns the volume of the constrained allowable motion polytope
    */
    double getConstrainedAllowableMotionPolytope ( const sensor_msgs::JointState & joint_states,
            Eigen::MatrixXd & AHrep,
            Eigen::VectorXd & bhrep,
            bool show_polytope,
            std::vector<double>  color_pts= {0.0,0.0,0.5,1.0},
            std::vector<double>  color_line= {0.0,0.0,1.0,0.8} );

    /** getConstrainedAllowableMotionPolytope returns the polytope
    *   approximating the constrained allowable end effector motion, considering
    *  joint limits and objstacles & linearization
    *
    *  AHrep hyperplanes of the constrained allowable motion polytope
    *  bHrep shifted distance
    *  Vset the representation of Constrained motion Polytope in Cartesian space
    *  offset_position the location in space of the Cartesian polytope
    *  const sensor_msgs::JointState & joint_states, current joint states
    *  show_polytope -> plots the polytope in rviz
    *  color_pts -> polytope points color
    *  color_line  -> polytope lines color
    *  returns the volume of the constrained allowable motion polytope
    */

    double getConstrainedAllowableMotionPolytope ( const sensor_msgs::JointState & joint_states,
            Eigen::MatrixXd & AHrep,
            Eigen::VectorXd & bhrep,
            Eigen::MatrixXd & Vset,
            Eigen::Vector3d & offset_position,
            bool show_polytope,
            std::vector<double>  color_pts,
            std::vector<double>  color_line ); 


    /** getConstrainedVelocityPolytope returns the polytope
    *   approximating the constrained allowable end effector velocities, considering
    *  joint velocity limits and objtacles and dangerfield values
    *
    *  AHrep hyperplanes of the constrained allowable motion polytope
    *  bHrep shifted distance
    *  const sensor_msgs::JointState & joint_states, current joint states
    *  show_polytope -> plots the polytope in rviz
    *  color_pts -> polytope points color
    *  color_line  -> polytope lines color
    *  returns the volume of the constrained velocity motion polytope
    */
    double getConstrainedVelocityPolytope ( const sensor_msgs::JointState & joint_states,
                                            Eigen::MatrixXd & AHrep,
                                            Eigen::VectorXd & bhrep,
                                            bool show_polytope,
                                            std::vector<double>  color_pts= {0.0,0.5,0.0,1.0},
                                            std::vector<double>  color_line= {0.0,1.0,0.0,0.8} ) ;

    /** getConstrainedVelocityPolytope returns the polytope
     *   approximating the constrained allowable end effector velocities, considering
     *  joint velocity limits and objtacles and dangerfield values
     *
     *  AHrep hyperplanes of the constrained allowable motion polytope
     *  bHrep shifted distance
     *  const sensor_msgs::JointState & joint_states, current joint states
     *  show_polytope -> plots the polytope in rviz
     *  color_pts -> polytope points color
     *  color_line  -> polytope lines color
     *  returns the volume of the constrained velocity motion polytope
     */
    double getConstrainedVelocityPolytope ( const sensor_msgs::JointState & joint_states,
                                            bool show_polytope,
                                            std::vector<double>  color_pts= {0.0,0.5,0.0,1.0},
                                            std::vector<double>  color_line= {0.0,1.0,0.0,0.8} ) ;


    /** getAllowableMotionPolytope returns the polytope
    *   considering     linearization
    *
    *  const sensor_msgs::JointState & joint_states, current joint states
    *  show_polytope -> plots the polytope in rviz
    *  color_pts -> polytope points color
    *  color_line  -> polytope lines color
    *  returns the volume of the constrained allowable motion polytope
    */
    double getAllowableMotionPolytope ( const sensor_msgs::JointState & joint_states,
                                        bool show_polytope,
                                        std::vector<double>  color_pts= {0.0,0.0,0.5,1.0},
                                        std::vector<double>  color_line= {0.0,0.0,1.0,0.8} );

    /** getAllowableMotionPolytope returns the polytope
    *   considering   linearization
    *
    *  AHrep hyperplanes of the constrained allowable motion polytope
    *  bHrep shifted distance
    *  const sensor_msgs::JointState & joint_states, current joint states
    *  show_polytope -> plots the polytope in rviz
    *  color_pts -> polytope points color
    *  color_line  -> polytope lines color
    *  returns the volume of the constrained allowable motion polytope
    */
    double getAllowableMotionPolytope ( const sensor_msgs::JointState & joint_states,
                                        Eigen::MatrixXd & AHrep,
                                        Eigen::VectorXd & bhrep,
                                        bool show_polytope,
                                        std::vector<double>  color_pts= {0.0,0.0,0.5,1.0},
                                        std::vector<double>  color_line= {0.0,0.0,1.0,0.8} );
    
    /** getAllowableMotionPolytope returns the polytope
    *   considering   linearization
    *
    *  AHrep hyperplanes of the constrained allowable motion polytope
    *  bHrep shifted distance
    *  Eigen::MatrixXd & Vset, the cartesian V representation of polytope
    *  Eigen::Vector3d & offset_position the position we translate the polytopte to
    *  const sensor_msgs::JointState & joint_states, current joint states
    *  show_polytope -> plots the polytope in rviz
    *  color_pts -> polytope points color
    *  color_line  -> polytope lines color
    *  returns the volume of the constrained allowable motion polytope
    */
    double getAllowableMotionPolytope ( const sensor_msgs::JointState & joint_states,
        Eigen::MatrixXd & AHrep,
        Eigen::VectorXd & bhrep,
        Eigen::MatrixXd & Vset,
        Eigen::Vector3d & offset_position,
        bool show_polytope,
        std::vector<double>  color_pts,
        std::vector<double>  color_line );
    
    /** getVelocityPolytope returns the manipulability polytope
    *   considering joint velocity limits
    *
    *  const sensor_msgs::JointState & joint_states, current joint states
    *  show_polytope -> plots the polytope in rviz
    *  color_pts -> polytope points color
    *  color_line  -> polytope lines color
    *  returns the volume of the manipulability polytope
    */
    double getVelocityPolytope ( const sensor_msgs::JointState & joint_states,
                                 bool show_polytope,
                                 std::vector<double>  color_pts= {0.0,0.5,0.0,1.0},
                                 std::vector<double>  color_line= {0.0,1.0,0.0,0.8} ) ;

    /** getVelocityManipulabilityPolytope returns the manipulability polytope
    *   considering joint velocity limits

    *  AHrep hyperplanes of the constrained allowable motion polytope
    *  bHrep shifted distance
    *  const sensor_msgs::JointState & joint_states, current joint states
    *  show_polytope -> plots the polytope in rviz
    *  color_pts -> polytope points color
    *  color_line  -> polytope lines color
    *  returns the volume of the manipulability polytope
    */
    double getVelocityPolytope ( const sensor_msgs::JointState & joint_states,
                                 bool show_polytope,
                                 Eigen::MatrixXd AHrep,
                                 Eigen::VectorXd bhrep,
                                 std::vector<double>  color_pts= {0.0,0.5,0.0,1.0},
                                 std::vector<double>  color_line= {0.0,1.0,0.0,0.8} ) ;




    /// Screw transform to move a twist from point a to point b, given vector L, a->b w.r.t. base frame
    static bool screwTransform ( const Eigen::Matrix<double,6,Eigen::Dynamic> &J0N_in,
                                 Eigen::Matrix<double,6,Eigen::Dynamic>& J0E_out,
                                 const Eigen::Vector3d & L );

    /// Skew symmetric matrix performing the cross product
    static bool skew ( const Eigen::Vector3d & L, Eigen::Matrix<double,3,3>& skewL ) {
        skewL ( 0,1 ) =-L ( 2 );
        skewL ( 0,2 ) =L ( 1 );
        skewL ( 1,2 ) =-L ( 0 );
        skewL ( 1,0 ) =-skewL ( 0,1 );
        skewL ( 2,0 ) =-skewL ( 0,2 );
        skewL ( 2,1 ) =-skewL ( 1,2 );
        return true;
    }

    /// Project translational Jacobian matrix along a vector
    static bool projectTranslationalJacobian ( Eigen::Vector3d nT,
            const Eigen::Matrix<double,6,Eigen::Dynamic>& J0N_in,
            Eigen::Matrix<double,1,Eigen::Dynamic>& J0N_out );

    /// Get the volume of a polytope defined by a vertex set
    static double getPolytopeVolume ( Eigen::MatrixXd  vertices );
    /// Convert from Hrep to V rep
    static bool getVrepPolytope ( const Eigen::MatrixXd & A_left,
                                  const Eigen::VectorXd & b_left,
                                  Eigen::MatrixXd & reduced_joint_vertex_set );
    /// Converts from V rep to H rep
    static bool getHrepPolytope ( const Eigen::MatrixXd& vertex_set,
                                  Eigen::MatrixXd& A_left,
                                  Eigen::VectorXd& b_left );
    /// Transform polytope to Cartesian space
    static void getCartesianPolytope ( Eigen::MatrixXd Q,
                                       Eigen::Matrix<double,3,Eigen::Dynamic> Jp,
                                       Eigen::Vector3d P,
                                       Eigen::MatrixXd& V
                                     );
    // Linear transformation defined by J of vertex set
    static void transformVertexSet ( Eigen::Matrix<double,3,Eigen::Dynamic> J,
                                     const Eigen::MatrixXd &vertex_set,
                                     Eigen::MatrixXd& vertex_set_out );



   // Function to get the intersection of 2 polytopes
   
    bool getPolytopeIntersection(const Eigen::MatrixXd & AHrep1,
        const Eigen::VectorXd & bhrep1,
        const Eigen::MatrixXd & AHrep2,
        const Eigen::VectorXd & bhrep2,
        Eigen::MatrixXd & AHrep3,
        Eigen::VectorXd & bhrep3,
        Eigen::MatrixXd & Vset);
    
    
    bool getPolytopeIntersection(const Eigen::MatrixXd & AHrep1,
        const Eigen::VectorXd & bhrep1,
        const Eigen::MatrixXd & AHrep2,
        const Eigen::VectorXd & bhrep2,      
        Eigen::MatrixXd & Vset);
    


    // Static Versions, useful for optimization engines, e.g. snopt, nlopt.



    /** getConstrainedAllowableMotionPolytope returns the polytope
    *   approximating the constrained allowable end effector motion, considering
    *  joint limits and objstacles & linearization
    *  chain KDL::Chain already initialized
    *  model  urdf::Model already initialized
    *  FCLObjectSet objects, a set of collision objects
    *  AHrep hyperplanes of the constrained allowable motion polytope
    *  bHrep shifted distance
    *  const sensor_msgs::JointState & joint_states, current joint states
    *  show_polytope -> plots the polytope in rviz
    *  color_pts -> polytope points color
    *  color_line  -> polytope lines color
    *  returns the volume of the constrained allowable motion polytope
    */
    static double getConstrainedAllowableMotionPolytope ( KDL::Chain &  chain,
            urdf::Model & model,
            FCLObjectSet objects,
            const sensor_msgs::JointState & joint_states,
            Eigen::MatrixXd & AHrep,
            Eigen::VectorXd & bhrep,
            double linearization_limit=0.1,
            double distance_threshold=0.3 );

    /** getConstrainedVelocityPolytope returns the polytope
    *   approximating the constrained velocity end effector motion, considering
    *  joint limits and objstacles & linearization
    *  chain KDL::Chain already initialized
    *  model  urdf::Model already initialized
    *  FCLObjectSet objects, a set of collision objects
    *  AHrep hyperplanes of the constrained allowable motion polytope
    *  bHrep shifted distance
    *  const sensor_msgs::JointState & joint_states, current joint states
    *  show_polytope -> plots the polytope in rviz
    *  color_pts -> polytope points color
    *  color_line  -> polytope lines color
    *  returns the volume of the constrained allowable motion polytope
    */
    static double getConstrainedVelocityPolytope ( KDL::Chain &  chain,
            urdf::Model & model,
            FCLObjectSet objects,
            const sensor_msgs::JointState & joint_states,
            Eigen::MatrixXd & AHrep,
            Eigen::VectorXd & bhrep,
            double dangerfield=10,
            double distance_threshold=0.3 );

    

    /** getPolytopeHyperPlanes returns hyperplanes for constrained joint polytope
    * For ith link (segment) in the kinematic serial chain
    *  chain KDL::Chain already initialized
    *  model  urdf::Model already initialized
    *  FCLObjectSet objects, a set of collision objects
    *  kdl_joint_positions current joint states
    *  AHrep is the normal to he half spaces
    *  bhrep constains the shifted distance from the origin along the normal
    * Polytope is then defined as P= { A x <= b }
    * velocity_polytope true means returns hyperplanes based on velocity and dangerfield
    *                   false means the free space approximation is returned.
    */
    static bool getPolytopeHyperPlanes ( KDL::Chain &  chain,
                                         urdf::Model & model,
                                         FCLObjectSet objects,
                                         const  KDL::JntArray & kdl_joint_positions,
                                         const GeometryInformation & geometry_information,
                                         Eigen::MatrixXd & AHrep,
                                         Eigen::VectorXd & bhrep,
                                         double distance_threshold=0.3,
                                         double linearization_limit=0.1,
                                         bool velocity_polytope=false,
                                         double dangerfield=10.0 );

    /** getCollisionModel returns  kinematic information about collision geometry
     * For ith link (segment) in the kinematic serial chain, we return
     *  chain KDL::Chain already initialized
     *  model  urdf::Model already initialized
     *  geometry_mkrs[i]: the shape description of geometery [i]
     *  geometry_transforms[i]: the transform of the collision geometry's [i] origin
     *  geometric_jacobians[i]: the jacobian matrix in the base frame at the collision geometry's [i] origin
     */
    static bool getCollisionModel ( KDL::Chain &  chain,
                                    urdf::Model & model,
                                    const  KDL::JntArray & kdl_joint_positions,
                                    GeometryInformation & geometry_information);



    /** Static getAllowableMotionPolytope returns the polytope
    *   considering   linearization
    *  chain KDL::Chain already initialized
    *  model  urdf::Model already initialized
    *  AHrep hyperplanes of the constrained allowable motion polytope
    *  bHrep shifted distance
    *  const sensor_msgs::JointState & joint_states, current joint states
    *  show_polytope -> plots the polytope in rviz
    *  color_pts -> polytope points color
    *  color_line  -> polytope lines color
    *  returns the volume of the constrained allowable motion polytope
    */
    static double getAllowableMotionPolytope ( KDL::Chain &  chain,
            urdf::Model & model,
            const sensor_msgs::JointState & joint_states,
            Eigen::MatrixXd & AHrep,
            Eigen::VectorXd & bhrep,
            double linearization_limit=0.1
                                             );

    /** getVelocityPolytope returns the manipulability polytope
    *   considering joint velocity limits
    *  chain KDL::Chain already initialized
    *  model  urdf::Model already initialized
    *  AHrep hyperplanes of the constrained allowable motion polytope
    *  bHrep shifted distance
    *  const sensor_msgs::JointState & joint_states, current joint states
    *  show_polytope -> plots the polytope in rviz
    *  color_pts -> polytope points color
    *  color_line  -> polytope lines color
    *  returns the volume of the manipulability polytope
    */
    static double getVelocityPolytope ( KDL::Chain &  chain,
                                        urdf::Model & model,
                                        const sensor_msgs::JointState & joint_states,
                                        Eigen::MatrixXd & AHrep,
                                        Eigen::VectorXd & bhrep );



    /// Function to return Jaocbian matrix to external users of library based on a joint state
    void getJacobian(const sensor_msgs::JointState & joint_states,Eigen::Matrix<double,6,Eigen::Dynamic> & Jac);
    
    /// Function to return transform matrix to external users of library based on a joint state
    void getTransform(const sensor_msgs::JointState & joint_states,Eigen::Affine3d & T);


    // Set a new limit for all joints
    void setLinearizationLimit(double linearization_limit);
    // Set a linearization limit for 1 joints
    void setLinearizationLimit(double linearization_limit,unsigned int joint);
    /// Function to return current linearization limit, this is the same for all joints
    double getLinearizationLimit();
    
    
};

#endif
