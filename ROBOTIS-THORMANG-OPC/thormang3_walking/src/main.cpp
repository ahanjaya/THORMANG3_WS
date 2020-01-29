/*******************************************************************************
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
*******************************************************************************/

/*
 * main.cpp
 *
 *  Edited on: 2019. 08. 16.
 *      Author: Hanjaya Mandala
 */

#include "thormang3_walking/thormang3_walking.h"

ros::Subscriber g_walking_opc_command_sub;

bool is_init_pose = false;

void sendWalkingCommand(const std::string &command)
{
  thormang3_foot_step_generator::FootStepCommand msg;
  msg.command           = command;
  msg.step_num          = 2; //2
  msg.step_time         = 0.5;
  msg.step_length       = 0.1;
  msg.side_step_length  = 0.05;
  msg.step_angle_rad    = deg2rad<double>(5.0);
  setWalkingCommand(msg);
}

void walking_opcCommandCallback(const std_msgs::String::ConstPtr& msg)
{

  ROS_INFO_STREAM("[walking_opc]  : receive [" << msg->data << "] msg " );

  if(msg->data == "ini_pose")
  {
    ROS_INFO("walking_opc: go to initial pose");
    moveToInitPose();
    is_init_pose = true;
    ROS_INFO("[walking_opc]  : please wait 5 seconds");
  }
  else if(msg->data == "reset_pose")
  {
    ROS_INFO("walking_opc: go to reset pose");
    moveToResetPose();
    ROS_INFO("[walking_opc]  : please wait 2 seconds");
  }
  else if ( msg->data == "set_mode")
  {
    ROS_INFO("[walking_opc]: set walking control mode");
    setCtrlModule();
  }  
  else if( msg->data == "balance_on" )
  {
    ROS_INFO("[walking_opc]: balance enable");
    setBalanceOn();
  }
  else if( msg->data == "balance_off" )
  {
    ROS_INFO("[walking_opc]: balance disable");
    setBalanceOff();
  }
  else if( msg->data == "forward" )
  {
    ROS_INFO("[walking_opc]: forward walking");
    sendWalkingCommand("forward");
  }
  else if( msg->data == "backward" )
  {
    ROS_INFO("[walking_opc]: backward walking");
    sendWalkingCommand("backward");
  }
  else if( msg->data == "turn_left" )
  {
    ROS_INFO("[walking_opc]: turn left walking");
    sendWalkingCommand("turn left");
  }
  else if( msg->data == "turn_right" )
  {
    ROS_INFO("[walking_opc]: turn right walking");
    sendWalkingCommand("turn right");
  }
  else if( msg->data == "left" )
  {
    ROS_INFO("[walking_opc]: left walking");
    sendWalkingCommand("left");
  }
  else if( msg->data == "right" )
  {
    ROS_INFO("[walking_opc]: right walking");
    sendWalkingCommand("right");
  }
  else if( msg->data == "stop" )
  {
    ROS_INFO("[walking_opc]: stop walking");
    sendWalkingCommand("stop");
  }
  else {
    ROS_ERROR_STREAM("Invalid Command: " << msg->data);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thormang3_walking");
  ROS_INFO("ROBOTIS THORMANG3 OPC Walking");

  initialize();

  ros::NodeHandle nh;
  g_walking_opc_command_sub = nh.subscribe("/robotis/walking/command", 10, walking_opcCommandCallback);

  ros::spin();
  return 0;
}

