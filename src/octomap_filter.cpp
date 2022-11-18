#include <boost/thread.hpp>

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>

#include <octomap/OcTreeKey.h>

#include <constrained_manipulability/octomap_filter.hpp>

namespace constrained_manipulability
{
    OctomapFilter::OctomapFilter(ros::NodeHandle nh) : nh_(nh), octomap_received_(false), tree_(NULL)
    {
        octomap_pose_wrt_world_.setIdentity();
        octomap_sub_ = nh_.subscribe("constrained_manipulability/octomap_full", 1, &OctomapFilter::octomapCallback, this);
    }

    OctomapFilter::~OctomapFilter()
    {
        if(tree_ != NULL)
        {
            delete tree_;
        }
    }

    void OctomapFilter::octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
    {
        boost::mutex::scoped_lock lock(tree_mutex_);

        octomap::AbstractOcTree* abs_tree = octomap_msgs::msgToMap(*msg); 
         // In case we receive a new octomap before we processed the last one
        if(tree_ != NULL)
        {
            delete tree_;
        }

        tree_ = dynamic_cast<octomap::OcTree*>(abs_tree);

        octomap_received_ = true;
    }

    bool OctomapFilter::setOctomapPose(const Eigen::Affine3d &wT1)
    {
        boost::mutex::scoped_lock lock(tree_mutex_);

        if (octomap_received_)
        {
            octomap_pose_wrt_world_ = wT1;
            return true;
        }

        return false;
    }

    bool OctomapFilter::getOctomapProperties(octomap_msgs::Octomap &octomap, Eigen::Affine3d &octomap_pose)
    {
        if (octomap_received_ && (tree_ != NULL))
        {  
            octomap_msgs::fullMapToMsg(*tree_, octomap);
            octomap_pose = octomap_pose_wrt_world_;

            // Clear up
            delete tree_;
            tree_ = NULL;
            octomap_received_ = false;

            return true;
        }

        return false;
    }

    bool OctomapFilter::filterObjectFromOctomap(const shapes::ShapeMsg &current_shapes, const geometry_msgs::Pose &shapes_pose)
    {
        if (!octomap_received_ && (tree_ != NULL))
        {
            return false;
        }

        octomap::OcTreeKey minKey, maxKey;
        double num_leaf_nodes = tree_->getNumLeafNodes();

        bodies::AABB bbox;
        if (current_shapes.which() == 0)
        {
            shape_msgs::SolidPrimitive s1 = boost::get<shape_msgs::SolidPrimitive>(current_shapes);
            bodies::Body *body = bodies::constructBodyFromMsg(s1, shapes_pose);
            body->computeBoundingBox(bbox);
        }
        else if (current_shapes.which() == 1)
        {
            shape_msgs::Mesh s1 = boost::get<shape_msgs::Mesh>(current_shapes);
            bodies::Body *body = bodies::constructBodyFromMsg(s1, shapes_pose);
            body->computeBoundingBox(bbox);
        }
        else
        {
            ROS_WARN("Only supports MESH and SOLID Pimitives");
            return false;
        }

        tree_->coordToKeyChecked(bbox.min().x(), bbox.min().y(), bbox.min().z(), minKey);
        tree_->coordToKeyChecked(bbox.max().x(), bbox.max().y(), bbox.max().z(), maxKey);

        std::vector<std::pair<octomap::OcTreeKey, unsigned int>> keys;
        for (octomap::OcTree::leaf_iterator it = tree_->begin_leafs(),
                                            end = tree_->end_leafs();
             it != end; ++it)
        {
            octomap::OcTreeKey k = it.getKey();
            if (k[0] >= minKey[0] && k[1] >= minKey[1] && k[2] >= minKey[2] && k[0] <= maxKey[0] && k[1] <= maxKey[1] && k[2] <= maxKey[2])
            {
                keys.push_back(std::make_pair(k, it.getDepth())); // add to a stack
            }
        }

        // delete nodes which are in bounding box
        for (auto k : keys)
            tree_->deleteNode(k.first, k.second);

        return true;
    }

    bool OctomapFilter::filterObjectFromOctomap(const std::vector<shapes::ShapeMsg> &current_shapes,
                                                const std::vector<geometry_msgs::Pose> &shapes_poses)
    {
        if (!octomap_received_)
        {
            return false;
        }

        octomap::OcTreeKey minKey, maxKey;
        double num_leaf_nodes = tree_->getNumLeafNodes();

        std::vector<std::pair<octomap::OcTreeKey, unsigned int>> keys;

        for (int var = 0; var < current_shapes.size(); ++var)
        {
            bodies::AABB bbox;
            if (current_shapes[var].which() == 0)
            {
                shape_msgs::SolidPrimitive s1 = boost::get<shape_msgs::SolidPrimitive>(current_shapes[var]);
                bodies::Body *body = bodies::constructBodyFromMsg(s1, shapes_poses[var]);
                body->computeBoundingBox(bbox);
            }
            else if (current_shapes[var].which() == 1)
            {
                // ROS_WARN("This might cause a fault because of qhull depending on your install");
                // ROS_WARN("See issues CHOMP fails to plan #241 https://github.com/ros-planning/moveit_task_constructor/issues/241");

                shape_msgs::Mesh s1 = boost::get<shape_msgs::Mesh>(current_shapes[var]);
                bodies::Body *body = bodies::constructBodyFromMsg(s1, shapes_poses[var]);
                body->computeBoundingBox(bbox);
            }
            else
            {
                ROS_WARN("Only supports MESH and SOLID Pimitives");
                return false;
            }

            tree_->coordToKeyChecked(bbox.min().x(), bbox.min().y(), bbox.min().z(), minKey);
            tree_->coordToKeyChecked(bbox.max().x(), bbox.max().y(), bbox.max().z(), maxKey);

            for (octomap::OcTree::leaf_iterator it = tree_->begin_leafs(),
                                                end = tree_->end_leafs();
                 it != end; ++it)
            {
                octomap::OcTreeKey k = it.getKey();
                if (k[0] >= minKey[0] && k[1] >= minKey[1] && k[2] >= minKey[2] && k[0] <= maxKey[0] && k[1] <= maxKey[1] && k[2] <= maxKey[2])
                {
                    keys.push_back(std::make_pair(k, it.getDepth())); // add to a stack
                }
            }
        }
        // // delete nodes which are in bounding box
        for (auto k : keys)
            tree_->deleteNode(k.first, k.second);

        return true;
    }
}