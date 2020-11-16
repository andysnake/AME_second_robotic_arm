#include "ur_planning/ur10_robot2_move_group.hpp"
#include <gazebo_msgs/SetModelState.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/String.h"
// TF2

#define FREQUENCY 1000
#define BOX_INITIAL_POS_X 2.65
#define BOX_INITIAL_POS_Y -0.6
#define BOX_INITIAL_POS_Z 0.58
#define BOX_END_POX_X 2.05
#define PI 3.1415926536

static const std::string PLANNING_GROUP = "manipulator";
static const std::string ROBOT_DESCRIPTION = "ur10_robot2/robot_description";

static Eigen::Affine3d endEffector_in_box, world_in_base, tool0_in_endEffector;

moveit::planning_interface::MoveGroupInterfacePtr ur10_robot2_group_ptr;
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
robot_state::RobotStatePtr kinematic_state;
ros::Publisher gazebo_model_state_pub;

std::string currentBoxName;

double BEFORE_PICK_POS_Z = 0.5;
double PICK_POS_Z = 0.23;
bool boxIsArrived = false;

void boxArrivedCallback(const std_msgs::String::ConstPtr &msg)
{
    //   ROS_INFO("I heard: [%s]", msg->data.c_str());
    boxIsArrived = true;
}

void jointStatesCallback(const sensor_msgs::JointState &joint_states_current)
{
    std::vector<double> joints_state;
    joints_state.push_back(joint_states_current.position[2]);
    joints_state.push_back(joint_states_current.position[1]);
    joints_state.push_back(joint_states_current.position[0]);
    joints_state.push_back(joint_states_current.position[3]);
    joints_state.push_back(joint_states_current.position[4]);
    joints_state.push_back(joint_states_current.position[5]);

    kinematic_state->setJointGroupPositions("manipulator", joints_state);

    // calculate box picked by the robot while robot is moving
    Eigen::Affine3d box_in_endEffector = getTransform(0, 0, 0.1, 0, 3.1415926, 3.1415926);
    Eigen::Affine3d endEffector_in_tool0 = getTransform(0, 0, 0.05, 0, 0, 0);
    Eigen::Affine3d base_in_world = getTransform(1.8, 0, 0.5, 0, 0, 0);
    Eigen::Affine3d tool0_in_base = kinematic_state->getGlobalLinkTransform("tool0");

    Eigen::Affine3d box_in_world = base_in_world * tool0_in_base * endEffector_in_tool0 * box_in_endEffector;

    geometry_msgs::Pose pose;
    pose.position.x = box_in_world.translation().x();
    pose.position.y = box_in_world.translation().y();
    pose.position.z = box_in_world.translation().z();

    Eigen::Quaterniond quat(box_in_world.rotation());
    pose.orientation.w = quat.w();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    // ros::Duration(100).sleep();
    gazebo_msgs::ModelState model_state;
    model_state.model_name = std::string(currentBoxName);
    model_state.pose = pose;
    model_state.reference_frame = std::string("world");
    gazebo_model_state_pub.publish(model_state);
}

int main(int argc, char **argv)
{ /**************************Initialize*************************************/
    ros::init(argc, argv, "ur10_robot2_planning");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    init();

    moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, ROBOT_DESCRIPTION, nh};
    ur10_robot2_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(Opt);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = ur10_robot2_group_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    kinematic_state = std::make_shared<robot_state::RobotState>(ur10_robot2_group_ptr->getRobotModel());
    ros::Rate poll_rate(FREQUENCY);

    ros::Subscriber box_arrived_sub = nh.subscribe("/box_arrived", 1000, boxArrivedCallback);
    ros::Publisher box_transfered_pub = nh.advertise<std_msgs::String>("/box_transfered", 1000);

    std_msgs::String msg;
    std::stringstream ss;
    ss << "transfered ";
    msg.data = ss.str();

    gazebo_model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
    while (gazebo_model_state_pub.getNumSubscribers() == 0)
    {
        poll_rate.sleep();
    }

    ros::Subscriber joint_states_sub;
    Eigen::Affine3d box_in_world;
    Eigen::Affine3d tool0_in_base;

    //
    std::string box_name_prefix("ur5_robot1_box");
    for (int box_num = 1; box_num < 5; box_num++)
    {
        currentBoxName = box_name_prefix + std::to_string(box_num);

        //initical position of the robot
        box_in_world = getTransform(1.0, 0, 0.65, PI, -PI / 2, 0);
        tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
        moveRobotToPos(tool0_in_base);

        // moveRobotToJointValue(-2.9480689337784334, -1.9732195640463823, -1.2590809193607582, -3.050317978600593, -1.3779115272303324, 1.5706529371378846)

        while (!boxIsArrived)
        {
            poll_rate.sleep();
        }
        boxIsArrived = false;
        // pick
        box_in_world = getTransform(0.8, 0, 0.65, PI, -PI / 2, 0);
        tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
        moveRobotToPos(tool0_in_base);
        // up
        joint_states_sub = nh.subscribe("/ur10_robot2/joint_states", 1000, jointStatesCallback);

        box_transfered_pub.publish(msg);

        std::vector<double> box_in_world;

        switch (box_num)
        {
        case 1:
            box_in_world = {2.3, -0.175, 0.9, 0, 3.1415926 / 2, 3.1415926};
            break;
        case 2:
            box_in_world = {2.3, 0.175, 0.9, 0, 3.1415926 / 2, 3.1415926};
            break;
        case 3:
            box_in_world = {2.3, -0.175, 1.15, 0, 3.1415926 / 2, 3.1415926};
            break;
        case 4:
            box_in_world = {2.3, 0.175, 1.15, 0, 3.1415926 / 2, 3.1415926};
            break;
        default:
            break;
        }

        moveBoxByRobot(box_in_world);
        box_in_world[0] += 0.4;
        moveBoxByRobot(box_in_world);
        joint_states_sub.shutdown();
        box_in_world[0] -= 0.4;
        moveBoxByRobot(box_in_world);
    }
    ros::shutdown();
    return 0;
}

void init()
{
    endEffector_in_box = getTransform(0, 0, 0.1, 0, 3.1415926, 3.1415926).inverse();
    world_in_base = getTransform(-1.8, 0, -0.5, 0, 0, 0);
    tool0_in_endEffector = getTransform(0, 0, -0.05, 0, 0, 0);
}
void moveBox(std::string box_name, double x, double y, double z, double qw, double qx, double qy, double qz)
{
    gazebo_msgs::ModelState model_state;
    model_state.model_name = box_name;
    model_state.reference_frame = std::string("world");
    model_state.pose.position.x = x; //start point  , end point
    model_state.pose.position.y = y;
    model_state.pose.position.z = z;

    model_state.pose.orientation.w = qw;
    model_state.pose.orientation.x = qx;
    model_state.pose.orientation.y = qy;
    model_state.pose.orientation.z = qz;
    gazebo_model_state_pub.publish(model_state);
}

void moveBoxBySpeed(std::string box_name, double startPos_x, double endPos_x, double speed)
{
    double currentPos_x = startPos_x;
    ros::Rate poll_rate(FREQUENCY);
    while (ros::ok() && currentPos_x > endPos_x)
    {
        currentPos_x = currentPos_x + speed * 1.0 / FREQUENCY;
        moveBox(box_name, currentPos_x, BOX_INITIAL_POS_Y, BOX_INITIAL_POS_Z, 1, 0, 0, 0);
        ros::spinOnce();
        poll_rate.sleep();
    }
}

void moveRobotToJointValue(const std::vector<double> &pos)
{
    ur10_robot2_group_ptr->setJointValueTarget(pos);
    bool success = (ur10_robot2_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ur10_robot2_group_ptr->move();
}

void moveRobotToJointValue(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6){
    std::vector<double> pos{joint1, joint2, joint3, joint4, joint5, joint6};
    moveRobotToJointValue(pos);
}

void moveRobotToPos(const geometry_msgs::Pose &pos)
{
    ur10_robot2_group_ptr->setPoseTarget(pos);
    bool success = (ur10_robot2_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ur10_robot2_group_ptr->move();
}

void moveRobotToPos(const Eigen::Affine3d &pos)
{
    geometry_msgs::Pose pose;
    pose.position.x = pos.translation().x();
    pose.position.y = pos.translation().y();
    pose.position.z = pos.translation().z();

    Eigen::Quaterniond quat(pos.rotation());
    pose.orientation.w = quat.w();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    moveRobotToPos(pose);
}

void moveRobotToPos(double x, double y, double z, double rr, double rp, double ry)
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
    moveRobotToPos(pose);
}

Eigen::Affine3d getTransform(double x, double y, double z, double roll, double pitch, double yaw)
{
    Eigen::Affine3d trans;
    trans.matrix()(0, 3) = x;
    trans.matrix()(1, 3) = y;
    trans.matrix()(2, 3) = z;

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    trans.matrix().block<3, 3>(0, 0) = q.matrix();
    return trans;
}

void RobotTrajectory::move()
{
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = ur10_robot2_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ur10_robot2_group_ptr->execute(trajectory);
    waypoints.clear();
}

void moveBoxByRobot(const std::vector<double> &pos)
{
    Eigen::Affine3d box_in_world = getTransform(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
    Eigen::Affine3d tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);
}