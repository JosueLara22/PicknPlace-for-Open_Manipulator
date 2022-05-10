﻿/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* Authors of the teleop operation of Open Manipulator robot with keyboard
* Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na
*
* Authors of the pick and place implementation with the Open Manipulator robot
* based on the teleoperation program developed by robotis 
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "open_manipulator_teleop/open_manipulator_teleop_keyboard.h"

OpenManipulatorTeleop::OpenManipulatorTeleop()
: node_handle_(""),
  priv_node_handle_("~")
{
  /************************************************************
  ** Initialize variables
  ************************************************************/
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initSubscriber();
  initClient();

  disableWaitingForEnter();
  ROS_INFO("OpenManipulator teleoperation using keyboard start");
}

OpenManipulatorTeleop::~OpenManipulatorTeleop()
{
  restoreTerminalSettings();
  ROS_INFO("Terminate OpenManipulator Joystick");
  ros::shutdown();
}

void OpenManipulatorTeleop::initClient()
{
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_joint_space_path_from_present_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
  goal_task_space_path_from_present_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
}

void OpenManipulatorTeleop::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorTeleop::jointStatesCallback, this);
  kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorTeleop::kinematicsPoseCallback, this);
}

void OpenManipulatorTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void OpenManipulatorTeleop::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

std::vector<double> OpenManipulatorTeleop::getPresentJointAngle()
{
  return present_joint_angle_;
}

std::vector<double> OpenManipulatorTeleop::getPresentKinematicsPose()
{
  return present_kinematic_position_;
}

bool OpenManipulatorTeleop::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if (goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;

  if (goal_task_space_path_from_present_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void OpenManipulatorTeleop::printText()
{
  printf("\n");
  printf("---------------------------\n");
  printf("Pick and place application for OpenManipulator!\n");
  printf("---------------------------\n");
  printf("Picks three objects located at the left side of the workspace and places them at the right side\n");
  printf("\n");
  printf("Then it performs the inverse task, pick the objects at the right and locate them at the left");
  printf("---------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         getPresentJointAngle().at(0),
         getPresentJointAngle().at(1),
         getPresentJointAngle().at(2),
         getPresentJointAngle().at(3));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         getPresentKinematicsPose().at(0),
         getPresentKinematicsPose().at(1),
         getPresentKinematicsPose().at(2));
  printf("---------------------------\n");

}
void OpenManipulatorTeleop::setGoal(char ch)
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT, 0.0);

  if (ch == 'w' || ch == 'W')
  {
    printf("input : w \tincrease(++) x axis in task space\n");
    goalPose.at(0) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 's' || ch == 'S')
  {
    printf("input : s \tdecrease(--) x axis in task space\n");
    goalPose.at(0) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'a' || ch == 'A')
  {
    printf("input : a \tincrease(++) y axis in task space\n");{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT, 0.0);
  }
  }

  if (ch == 'g' || ch == 'G')
  {
    printf("input : g \topen gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(0.01);
    setToolControl(joint_angle);
  }
  else if (ch == 'f' || ch == 'F')
  {
    printf("input : f \tclose gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);
    setToolControl(joint_angle);
  }
  else if (ch == '2')
  {
    printf("input : 2 \thome pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
    joint_name.push_back("joint3"); joint_angle.push_back(0.35);
    joint_name.push_back("joint4"); joint_angle.push_back(0.70);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '1')
  {
    printf("input : 1 \tinit pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '3')
  {
    printf("input : 3 \our pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.425);
    joint_name.push_back("joint2"); joint_angle.push_back(0.618);
    joint_name.push_back("joint3"); joint_angle.push_back(-0.792);
    joint_name.push_back("joint4"); joint_angle.push_back(1.689);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '4')
  {
    printf("input : 4 \our pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(-0.535);
    joint_name.push_back("joint2"); joint_angle.push_back(0.554);
    joint_name.push_back("joint3"); joint_angle.push_back(-0.403);
    joint_name.push_back("joint4"); joint_angle.push_back(1.388);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '5')
  {
    printf("input : 5 \our pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.423);
    joint_name.push_back("joint2"); joint_angle.push_back(0.625);
    joint_name.push_back("joint3"); joint_angle.push_back(-0.676);
    joint_name.push_back("joint4"); joint_angle.push_back(1.597);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '6')
  {
    printf("input : 6 \our pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(-0.535);
    joint_name.push_back("joint2"); joint_angle.push_back(0.563);
    joint_name.push_back("joint3"); joint_angle.push_back(-0.614);
    joint_name.push_back("joint4"); joint_angle.push_back(1.621);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '7')
  {
    printf("input : 7 \our pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.423);
    joint_name.push_back("joint2"); joint_angle.push_back(0.643);
    joint_name.push_back("joint3"); joint_angle.push_back(-0.591);
    joint_name.push_back("joint4"); joint_angle.push_back(1.516);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '8')
  {
    printf("input : 8 \our pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(-0.534);
    joint_name.push_back("joint2"); joint_angle.push_back(0.588);
    joint_name.push_back("joint3"); joint_angle.push_back(-0.695);
    joint_name.push_back("joint4"); joint_angle.push_back(1.707);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  sleep(2);
}
void OpenManipulatorTeleop::restoreTerminalSettings(void)
{
  tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void OpenManipulatorTeleop::disableWaitingForEnter(void)
{
  struct termios newt;

  tcgetattr(0, &oldt_);  /* Save terminal settings */
  newt = oldt_;  /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
  tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_teleop_keyboard");
OpenManipulatorTeleop openManipulatorTeleop;

  char ch;
  openManipulatorTeleop.printText();
  while (ros::ok() )
  {
  
    ch='2';
    openManipulatorTeleop.setGoal(ch);
    ch='g';
    openManipulatorTeleop.setGoal(ch);
    ch='3';
    openManipulatorTeleop.setGoal(ch);
    ch='f';
    openManipulatorTeleop.setGoal(ch);
    ch='2';
    openManipulatorTeleop.setGoal(ch);
    ch='4';
    openManipulatorTeleop.setGoal(ch);
    ch='g';
    openManipulatorTeleop.setGoal(ch);
    ch='2';
    openManipulatorTeleop.setGoal(ch);
    ch='5';
    openManipulatorTeleop.setGoal(ch);
    ch='f';
    openManipulatorTeleop.setGoal(ch);
    ch='2';
    openManipulatorTeleop.setGoal(ch);
    ch='6';
    openManipulatorTeleop.setGoal(ch);
    ch='g';
    openManipulatorTeleop.setGoal(ch);
    ch='2';
    openManipulatorTeleop.setGoal(ch);
    ch='7';
    openManipulatorTeleop.setGoal(ch);
    ch='f';
    openManipulatorTeleop.setGoal(ch);
    ch='2';
    openManipulatorTeleop.setGoal(ch);
    ch='8';
    openManipulatorTeleop.setGoal(ch);
    ch='g';
    openManipulatorTeleop.setGoal(ch);    
    ch='2';
    openManipulatorTeleop.setGoal(ch);
    ch='8';
    openManipulatorTeleop.setGoal(ch);
    ch='f';
    openManipulatorTeleop.setGoal(ch);
    ch='2';
    openManipulatorTeleop.setGoal(ch);
    ch='7';
    openManipulatorTeleop.setGoal(ch);
    ch='g';
    openManipulatorTeleop.setGoal(ch);
    ch='2';
    openManipulatorTeleop.setGoal(ch);
    ch='6';
    openManipulatorTeleop.setGoal(ch);
    ch='f';
    openManipulatorTeleop.setGoal(ch);
    ch='2';
    openManipulatorTeleop.setGoal(ch);
    ch='5';
    openManipulatorTeleop.setGoal(ch);
    ch='g';
    openManipulatorTeleop.setGoal(ch);
    ch='2';
    openManipulatorTeleop.setGoal(ch);
    ch='4';
    openManipulatorTeleop.setGoal(ch);
    ch='f';
    openManipulatorTeleop.setGoal(ch);
    ch='2';
    openManipulatorTeleop.setGoal(ch);
    ch='3';
    openManipulatorTeleop.setGoal(ch);
    ch='g';
    openManipulatorTeleop.setGoal(ch);
  }
  return 0;
}
