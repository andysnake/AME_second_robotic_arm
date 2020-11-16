#include "ur_planning/ur5_robot1_move_group.hpp"
#include <gazebo_msgs/SetModelState.h>
#include "std_msgs/String.h"

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2

#define FREQUENCY 1000
#define BOX_INITIAL_POS_X -1.1
#define BOX_INITIAL_POS_Y 0.6
#define BOX_INITIAL_POS_Z 0.58
#define BOX_END_POX_X -0.1
#define PI 3.1415926536

static const std::string PLANNING_GROUP = "manipulator";
static const std::string ROBOT_DESCRIPTION = "ur5_robot1/robot_description";

static Eigen::Affine3d endEffector_in_box, world_in_base,  tool0_in_endEffector;

moveit::planning_interface::MoveGroupInterfacePtr ur5_robot1_group_ptr;
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
robot_state::RobotStatePtr kinematic_state;
ros::Publisher gazebo_model_state_pub;

std::string currentBoxName;
bool boxIsTransfered = false;


void boxTransferedCallback(const std_msgs::String::ConstPtr& msg)
{
    boxIsTransfered = true;
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
    Eigen::Affine3d base_in_world = getTransform(0,0, 0.3, 0, 0, 0);
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
    ros::init(argc, argv, "ur5_robot1_planning");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    init();

    moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, ROBOT_DESCRIPTION, nh};
    ur5_robot1_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(Opt);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = ur5_robot1_group_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    kinematic_state = std::make_shared<robot_state::RobotState>(ur5_robot1_group_ptr->getRobotModel());
    ros::Rate poll_rate(FREQUENCY);

    gazebo_model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);

    ros::Publisher box_arrived_pub = nh.advertise<std_msgs::String>("/box_arrived", 1000);
    ros::Subscriber box_transfered_sub = nh.subscribe("/box_transfered", 1000, boxTransferedCallback);


    std_msgs::String msg;
    std::stringstream ss;
    ss << "arrived ";
    msg.data = ss.str();

    while (box_arrived_pub.getNumSubscribers() == 0)
    {
        poll_rate.sleep();
    }

    while (gazebo_model_state_pub.getNumSubscribers() == 0)
    {
        poll_rate.sleep();
    }

  
    moveBox(currentBoxName, BOX_INITIAL_POS_X, BOX_INITIAL_POS_Y, BOX_INITIAL_POS_Z, 1, 0, 0, 0); //initical position of the box

      //////////////////////////////first time//////////////////////////////////////////////////////////

    //initical position of the robot
    std::vector<double> robot_pos = {2.1638830636965913, -1.9281731851461448, 1.8554271998957468, -1.4975848537807899, -1.5702273917826703, 0.59340612948853};
    moveRobotToJointValue(robot_pos);
    ros::Duration(3).sleep();
    //move box to the end position of the platform
    currentBoxName = "ur5_robot1_box1";
    moveBoxBySpeed(currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);

    RobotTrajectory  robotTrajectory;
    // before pick
    moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    // pick
    moveRobotToPos(-0.1, 0.6, 0.43, PI, 0, 0);
    // up
    ros::Subscriber joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    
    // before place
    Eigen::Affine3d box_in_world = getTransform(0.35, 0.175, 0.65, 0, -3.1415926 / 2, 0);
    Eigen::Affine3d tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);
    joint_states_sub.shutdown();

    box_arrived_pub.publish(msg);

    while(!boxIsTransfered){ //wait
        poll_rate.sleep();
    }



    // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
    // moveRobotToJointValue(robot_pos);
    // place
    box_in_world = getTransform(0.65, 0.175, 0.15, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

    joint_states_sub.shutdown();

    // leave 弄远一点
    box_in_world = getTransform(0.35, 0.175, 0.15, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);
    // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
    // moveRobotToJointValue(robot_pos);

   ////////////////////////////////////////////////second time////////////////////////////////////////////////////////////////////////////////

    currentBoxName = "ur5_robot1_box2";
    //initial position
    robot_pos = {2.1638830636965913, -1.9281731851461448, 1.8554271998957468, -1.4975848537807899, -1.5702273917826703, 0.59340612948853};
    moveRobotToJointValue(robot_pos);
    //move box2
    moveBoxBySpeed(currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X)source devel/setup.bash
    moveRobotToPos(-0.1, 0.6, 0.43, PI, 0, 0);
    // up
    joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
   // before place
    box_in_world = getTransform(0.35, -0.175, 0.15, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);
    // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
    // moveRobotToJointValue(robot_pos);
    // place
    box_in_world = getTransform(0.65, -0.175, 0.15, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

    joint_states_sub.shutdown();

    // leave 弄远一点
    box_in_world = getTransform(0.35, -0.175, 0.15, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);
    // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
    // moveRobotToJointValue(robot_pos);

////////////////////////////////////////////////third time////////////////////////////////////////////////////////////////////////////////
    currentBoxName = "ur5_robot1_box3";
    //initial position
    robot_pos = {2.1638830636965913, -1.9281731851461448, 1.8554271998957468, -1.4975848537807899, -1.5702273917826703, 0.59340612948853};
    moveRobotToJointValue(robot_pos);
    //move box3
    moveBoxBySpeed(currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
     // before pick
    moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    // pick
    moveRobotToPos(-0.1, 0.6, 0.43, PI, 0, 0);
    // up
    joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    // before place
    box_in_world = getTransform(0.35, 0.175, 0.4, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);
    // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
    // moveRobotToJointValue(robot_pos);
    // place
    box_in_world = getTransform(0.65, 0.175, 0.4, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

    joint_states_sub.shutdown();

    // leave 弄远一点
    box_in_world = getTransform(0.35, 0.175, 0.4, 0, -3.1415926 / 2, 0);
    // tool0_in_base = world_in_base *    ros::Duration(10000).sleep();pos);

////////////////////////////////////////////////fourth time////////////////////////////////////////////////////////////////////////////////
    currentBoxName = "ur5_robot1_box4";
    //initial position
    robot_pos = {2.1638830636965913, -1.9281731851461448, 1.8554271998957468, -1.4975848537807899, -1.5702273917826703, 0.59340612948853};
    moveRobotToJointValue(robot_pos);
    //move box3
    moveBoxBySpeed(currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
     // before pick
    moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    // pick
    moveRobotToPos(-0.1, 0.6, 0.43, PI, 0, 0);
    // up
    joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    // before place
    box_in_world = getTransform(0.35, -0.175, 0.4, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);
    // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
    // moveRobotToJointValue(robot_pos);
    // place
    box_in_world = getTransform(0.65, -0.175, 0.4, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

    joint_states_sub.shutdown();

    // leave 弄远一点
    box_in_world = getTransform(0.35, -0.175, 0.4, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);
    // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
    // moveRobotToJointValue(robot_pos);
////////////////////////////////////////////////fifth time////////////////////////////////////////////////////////////////////////////////
    currentBoxName = "ur5_robot1_box5";
    //initial position
    robot_pos = {2.1638830636965913, -1.9281731851461448, 1.8554271998957468, -1.4975848537807899, -1.5702273917826703, 0.59340612948853};
    moveRobotToJointValue(robot_pos);
    //move box3
    moveBoxBySpeed(currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
     // before pick
    moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    // pick
    moveRobotToPos(-0.1, 0.6, 0.43, PI, 0, 0);
    // up
    joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    // before place
    box_in_world = getTransform(0.35, 0.175, 0.65, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);
    // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
    // moveRobotToJointValue(robot_pos);
    // place
    box_in_world = getTransform(0.65, 0.175, 0.65, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

    joint_states_sub.shutdown();

    // leave 弄远一点
    box_in_world = getTransform(0.35, 0.175, 0.65, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);
    // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
    // moveRobotToJointValue(robot_pos);
////////////////////////////////////////////////sixth time////////////////////////////////////////////////////////////////////////////////
    currentBoxName = "ur5_robot1_box6";
    //initial position
    robot_pos = {2.1638830636965913, -1.9281731851461448, 1.8554271998957468, -1.4975848537807899, -1.5702273917826703, 0.59340612948853};
    moveRobotToJointValue(robot_pos);
    //move box3
    moveBoxBySpeed(currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
     // before pick
    moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    // pick
    moveRobotToPos(-0.1, 0.6, 0.43, PI, 0, 0);
    // up
    joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    // before place
    box_in_world = getTransform(0.35, -0.175, 0.65, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);
    // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
    // moveRobotToJointValue(robot_pos);
    // place
    box_in_world = getTransform(0.65, -0.175, 0.65, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

    joint_states_sub.shutdown();

    // leave 弄远一点
    box_in_world = getTransform(0.35, -0.175,0.65, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);
    // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
    // moveRobotToJointValue(robot_pos);

    ros::Duration(10000).sleep();

    ros::shutdown();
    return 0;
}

void init(){
    endEffector_in_box = getTransform(0, 0, 0.1, 0, 3.1415926, 3.1415926).inverse();
    world_in_base = getTransform(0, 0, -0.3, 0, 0, 0);
    tool0_in_endEffector = getTransform(0, 0, -0.05, 0, 0, 0);
}
void moveBox(std::string box_name, double x, double y, double z, double qw, double qx, double qy, double qz)
{
    gazebo_msgs::ModelState model_state;
    model_state.model_name = box_name;
    model_state.reference_frame = std::string("world");
    model_state.pose.position.x = x; //start point -1.1 , end point -0.05
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
    while (ros::ok() && currentPos_x < endPos_x)
    {
        currentPos_x = currentPos_x + speed * 1.0 / FREQUENCY;
        moveBox(box_name, currentPos_x, BOX_INITIAL_POS_Y, BOX_INITIAL_POS_Z, 1, 0, 0, 0);
        ros::spinOnce();
        poll_rate.sleep();
    }
}

void moveRobotToJointValue(const std::vector<double> &pos)
{
    ur5_robot1_group_ptr->setJointValueTarget(pos);
    bool success = (ur5_robot1_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ur5_robot1_group_ptr->move();
}

void moveRobotToPos(const geometry_msgs::Pose &pos)
{
    ur5_robot1_group_ptr->setPoseTarget(pos);
    bool success = (ur5_robot1_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ur5_robot1_group_ptr->move();
}

void moveRobotToPos(const Eigen::Affine3d& pos){
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

void moveRobotToPos(double x, double y, double z, double rr, double rp, double ry){
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

void RobotTrajectory::move(){
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = ur5_robot1_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ur5_robot1_group_ptr->execute(trajectory);
    waypoints.clear();
}