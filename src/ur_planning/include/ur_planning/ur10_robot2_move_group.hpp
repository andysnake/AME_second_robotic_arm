#ifndef UR10_ROBOT2_MOVE_GROUP
#define UR10_ROBOT2_MOVE_GROUP

#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// class Move_Group_Robot_2
// {
// private:
//     /* data */
// public:

//     // Initializing ROS Parameters

//     std::string PLANNING_GROUP = "manipulator";
//     std::string ROBOT_DESCRIPTION = "ur10_robot2/robot_description";

//     // moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, ROBOT_DESCRIPTION, node_handle_rob2};

//     // static moveit::planning_interface::MoveGroupInterfacePtr ur10_robot2_group_ptr;

//     // Initializing MoveGroup Parameters
//     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//     // static const robot_state::JointModelGroup* joint_model_group;
//     moveit::planning_interface::MoveGroupInterface::Plan ur10_robot2_cartesian_plan;
//     moveit::planning_interface::MoveGroupInterface::Plan ur10_robot2_goal_plan;

//     moveit_msgs::OrientationConstraint goal_pose_constraint;

//     std::vector<std::string> joint_names;
//     std::vector<std::string> link_names;

//     //Functions to perform operations
//     Move_Group_Robot_2(ros::NodeHandle& node_handle_rob2);

//     void move_to_configuration(std::vector<double>&target_joint_angles);

//     void move_to_pose(const geometry_msgs::Pose& pose);
// };

void moveBox(std::string box_name, double x, double y, double z, double qw, double qx, double qy, double qz);

void moveBoxBySpeed(std::string box_name, double startPos_x, double endPos_x, double speed = 0.4);

void moveRobotToJointValue(const std::vector<double> &pos);

void moveRobotToPos(const geometry_msgs::Pose &pos);

void moveRobotToPos(const Eigen::Affine3d& pos);

void moveRobotToPos(double x, double y, double z, double rr, double rp, double ry);

void init();

Eigen::Affine3d getTransform(double x, double y, double z, double roll, double pitch, double yaw);



class RobotTrajectory
{
public:
    void addPoint(double x, double y, double z, double rr, double rp, double ry)
    {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        tf::Quaternion Quaternion = tf::createQuaternionFromRPY(rr, rp, ry);
        pose.orientation.x = Quaternion[0];
        pose.orientation.y = Quaternion[1];
        pose.orientation.z = Quaternion[2];
        pose.orientation.w = Quaternion[3];
        waypoints.push_back(pose);
    }

    void  move();

private:
    std::vector<geometry_msgs::Pose> waypoints;
};

#endif