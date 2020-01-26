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

/* Authors: Kayman Jung, SCH */

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <sstream>

#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>

#include "../include/thormang3_offset_tuner_client/qnode.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace thormang3_offset_tuner_client
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

QNode::QNode(int argc, char** argv)
    : init_argc_(argc),
      init_argv_(argv),
      is_refresh_(false)
{
}

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc_, init_argv_, "thormang3_offset_tuner_client");

  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle nh;

  // Add your ros communications here
  joint_offset_data_pub_ = nh.advertise<thormang3_offset_tuner_msgs::JointOffsetData>(
      "/robotis/offset_tuner/joint_offset_data", 0);
  torque_enable_pub_ = nh.advertise<thormang3_offset_tuner_msgs::JointTorqueOnOffArray>(
      "/robotis/offset_tuner/torque_enable", 0);
  command_pub_ = nh.advertise<std_msgs::String>("/robotis/offset_tuner/command", 0);

  get_present_joint_offset_data_client_ = nh.serviceClient<thormang3_offset_tuner_msgs::GetPresentJointOffsetData>(
      "/robotis/offset_tuner/get_present_joint_offset_data");

  std::string _config_path = ros::package::getPath("thormang3_offset_tuner_client") + "/config/joint_data.yaml";
  ParseOffsetGroup(_config_path);

  start();
  return true;
}

void QNode::run()
{

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::send_torque_enable_msg(thormang3_offset_tuner_msgs::JointTorqueOnOffArray msg)
{
  torque_enable_pub_.publish(msg);

  log(Info, "Joint Torque On/Off");
}

void QNode::send_joint_offset_data_msg(thormang3_offset_tuner_msgs::JointOffsetData msg)
{
  joint_offset_data_pub_.publish(msg);

  log(Info, "Send Joint Offset Data");
}

void QNode::send_command_msg(std_msgs::String msg)
{
  command_pub_.publish(msg);

  std::stringstream log_msg;
  log_msg << "Send Command : " << msg.data;

  log(Info, log_msg.str());
}

void QNode::getPresentJointOffsetData(bool recalculate_offset)
{
  is_refresh_ = true;
  thormang3_offset_tuner_msgs::GetPresentJointOffsetData get_present_joint_offset_data;

  //request

  //response
  if (get_present_joint_offset_data_client_.call(get_present_joint_offset_data))
  {
    for (int id = 0; id < get_present_joint_offset_data.response.present_data_array.size(); id++)
    {
      thormang3_offset_tuner_msgs::JointOffsetPositionData present_joint_data = get_present_joint_offset_data.response
          .present_data_array[id];

      if(recalculate_offset == true)
        present_joint_data.offset_value = present_joint_data.present_value - present_joint_data.goal_value;

      Q_EMIT update_present_joint_offset_data(present_joint_data);
    }
  }
  else
    log(Error, "Fail to get joint offset data");

  is_refresh_ = false;
}

void QNode::log(const LogLevel &level, const std::string &msg)
{
  logging_model_.insertRows(logging_model_.rowCount(), 1);
  std::stringstream logging_model_msg;
  switch (level)
  {
    case (Debug):
    {
      ROS_DEBUG_STREAM(msg);
      logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Info):
    {
      ROS_INFO_STREAM(msg);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Warn):
    {
      ROS_WARN_STREAM(msg);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Error):
    {
      ROS_ERROR_STREAM(msg);
      logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Fatal):
    {
      ROS_FATAL_STREAM(msg);
      logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
      break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(logging_model_.rowCount() - 1), new_row);
  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}

void QNode::ParseOffsetGroup(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load offset config yaml.");
    return;
  }

  // parse right_arm
  YAML::Node right_arm_node = doc["right_arm"];
  for (YAML::iterator yaml_it = right_arm_node.begin(); yaml_it != right_arm_node.end(); ++yaml_it)
  {
    int index;
    std::string joint_name;

    index = yaml_it->first.as<int>();
    joint_name = yaml_it->second.as<std::string>();

    right_arm_offset_group[index] = joint_name;
  }

  YAML::Node left_arm_node = doc["left_arm"];
  for (YAML::iterator yaml_it = left_arm_node.begin(); yaml_it != left_arm_node.end(); ++yaml_it)
  {
    int index;
    std::string joint_name;

    index = yaml_it->first.as<int>();
    joint_name = yaml_it->second.as<std::string>();

    left_arm_offset_group[index] = joint_name;
  }

  YAML::Node legs_node = doc["legs"];
  for (YAML::iterator yaml_it = legs_node.begin(); yaml_it != legs_node.end(); ++yaml_it)
  {
    int index;
    std::string joint_name;

    index = yaml_it->first.as<int>();
    joint_name = yaml_it->second.as<std::string>();

    legs_offset_group[index] = joint_name;
  }

  YAML::Node body_node = doc["body"];
  for (YAML::iterator yaml_it = body_node.begin(); yaml_it != body_node.end(); ++yaml_it)
  {
    int index;
    std::string joint_name;

    index = yaml_it->first.as<int>();
    joint_name = yaml_it->second.as<std::string>();

    body_offset_group[index] = joint_name;
  }
}
}  // namespace thormang3_offset_tuner_client
