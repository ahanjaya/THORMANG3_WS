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
 * thormang3_walking.cpp
 *
 *  Created on: 2016. 2. 18.
 *      Author: Jay Song
 */

#include "thormang3_walking/thormang3_walking.h"

ros::Publisher      g_wholebody_ini_pose_pub;
ros::Publisher      g_enable_ctrl_module_pub;
ros::Publisher      set_walking_command_pub_;

ros::ServiceClient  g_get_ref_step_data_client;
ros::ServiceClient  g_add_step_data_array_client;
ros::ServiceClient  g_is_running_client;
ros::ServiceClient  g_set_balance_param_client;
ros::ServiceClient  g_set_feedback_gain_client;

ros::Subscriber     g_walking_module_status_msg_sub;


double g_start_end_time = 2.0; //sec
double g_step_time      = 1.0; //sec
double g_step_length    = 0.1; //meter
double g_body_z_swap    = 0.01; //meter
double g_foot_z_swap    = 0.1; //meter


void walkingModuleStatusMSGCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
  ROS_INFO_STREAM("[" << msg->module_name <<"] : " << msg->status_msg);
}

void initialize()
{
  ros::NodeHandle nh;

  g_wholebody_ini_pose_pub        = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  g_enable_ctrl_module_pub        = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  set_walking_command_pub_        = nh.advertise<thormang3_foot_step_generator::FootStepCommand>("/robotis/thormang3_foot_step_generator/walking_command", 0);

  g_get_ref_step_data_client      = nh.serviceClient<thormang3_walking_module_msgs::GetReferenceStepData>("/robotis/walking/get_reference_step_data");
  g_add_step_data_array_client    = nh.serviceClient<thormang3_walking_module_msgs::AddStepDataArray>("/robotis/walking/add_step_data");
  g_is_running_client             = nh.serviceClient<thormang3_walking_module_msgs::IsRunning>("/robotis/walking/is_running");

  g_set_balance_param_client      = nh.serviceClient<thormang3_walking_module_msgs::SetBalanceParam>("/robotis/walking/set_balance_param");
  g_set_feedback_gain_client      = nh.serviceClient<thormang3_walking_module_msgs::SetJointFeedBackGain>("/robotis/walking/joint_feedback_gain");

  g_walking_module_status_msg_sub = nh.subscribe("/robotis/status", 10, walkingModuleStatusMSGCallback);
}

void moveToInitPose()
{
  std_msgs::String str_msg;
  str_msg.data = "ini_pose";

  g_wholebody_ini_pose_pub.publish( str_msg );
}

void setCtrlModule()
{
  std_msgs::String set_ctrl_mode_msg;
  set_ctrl_mode_msg.data = "walking_module";
  g_enable_ctrl_module_pub.publish( set_ctrl_mode_msg );
}

bool loadBalanceParam(thormang3_walking_module_msgs::SetBalanceParam& set_param)
{
  ros::NodeHandle ros_node;
  std::string balance_yaml_path = "";
  balance_yaml_path = ros::package::getPath("thormang3_walking") + "/data/balance_param.yaml";

  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(balance_yaml_path.c_str());
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Failed to load balance param yaml file.");
    return false;
  }

  double cob_x_offset_m                      = doc["cob_x_offset_m"].as<double>();
  double cob_y_offset_m                      = doc["cob_y_offset_m"].as<double>();

  double hip_roll_swap_angle_rad             = doc["hip_roll_swap_angle_rad"].as<double>();

  double foot_roll_gyro_p_gain               = doc["foot_roll_gyro_p_gain"].as<double>();
  double foot_roll_gyro_d_gain               = doc["foot_roll_gyro_d_gain"].as<double>();
  double foot_pitch_gyro_p_gain              = doc["foot_pitch_gyro_p_gain"].as<double>();
  double foot_pitch_gyro_d_gain              = doc["foot_pitch_gyro_d_gain"].as<double>();

  double foot_roll_angle_p_gain              = doc["foot_roll_angle_p_gain"].as<double>();
  double foot_roll_angle_d_gain              = doc["foot_roll_angle_d_gain"].as<double>();
  double foot_pitch_angle_p_gain             = doc["foot_pitch_angle_p_gain"].as<double>();
  double foot_pitch_angle_d_gain             = doc["foot_pitch_angle_d_gain"].as<double>();

  double foot_x_force_p_gain                 = doc["foot_x_force_p_gain"].as<double>();
  double foot_x_force_d_gain                 = doc["foot_x_force_d_gain"].as<double>();
  double foot_y_force_p_gain                 = doc["foot_y_force_p_gain"].as<double>();
  double foot_y_force_d_gain                 = doc["foot_y_force_d_gain"].as<double>();
  double foot_z_force_p_gain                 = doc["foot_z_force_p_gain"].as<double>();
  double foot_z_force_d_gain                 = doc["foot_z_force_d_gain"].as<double>();
  double foot_roll_torque_p_gain             = doc["foot_roll_torque_p_gain"].as<double>();
  double foot_roll_torque_d_gain             = doc["foot_roll_torque_d_gain"].as<double>();
  double foot_pitch_torque_p_gain            = doc["foot_pitch_torque_p_gain"].as<double>();
  double foot_pitch_torque_d_gain            = doc["foot_pitch_torque_d_gain"].as<double>();

  double roll_gyro_cut_off_frequency         = doc["roll_gyro_cut_off_frequency"].as<double>();
  double pitch_gyro_cut_off_frequency        = doc["pitch_gyro_cut_off_frequency"].as<double>();
  double roll_angle_cut_off_frequency        = doc["roll_angle_cut_off_frequency"].as<double>();
  double pitch_angle_cut_off_frequency       = doc["pitch_angle_cut_off_frequency"].as<double>();
  double foot_x_force_cut_off_frequency      = doc["foot_x_force_cut_off_frequency"].as<double>();
  double foot_y_force_cut_off_frequency      = doc["foot_y_force_cut_off_frequency"].as<double>();
  double foot_z_force_cut_off_frequency      = doc["foot_z_force_cut_off_frequency"].as<double>();
  double foot_roll_torque_cut_off_frequency  = doc["foot_roll_torque_cut_off_frequency"].as<double>();
  double foot_pitch_torque_cut_off_frequency = doc["foot_pitch_torque_cut_off_frequency"].as<double>();

  set_param.request.balance_param.cob_x_offset_m                      = cob_x_offset_m;
  set_param.request.balance_param.cob_y_offset_m                      = cob_y_offset_m;
  set_param.request.balance_param.hip_roll_swap_angle_rad             = hip_roll_swap_angle_rad;
  set_param.request.balance_param.foot_roll_gyro_p_gain               = foot_roll_gyro_p_gain;
  set_param.request.balance_param.foot_roll_gyro_d_gain               = foot_roll_gyro_d_gain;
  set_param.request.balance_param.foot_pitch_gyro_p_gain              = foot_pitch_gyro_p_gain;
  set_param.request.balance_param.foot_pitch_gyro_d_gain              = foot_pitch_gyro_d_gain;
  set_param.request.balance_param.foot_roll_angle_p_gain              = foot_roll_angle_p_gain;
  set_param.request.balance_param.foot_roll_angle_d_gain              = foot_roll_angle_d_gain;
  set_param.request.balance_param.foot_pitch_angle_p_gain             = foot_pitch_angle_p_gain;
  set_param.request.balance_param.foot_pitch_angle_d_gain             = foot_pitch_angle_d_gain;
  set_param.request.balance_param.foot_x_force_p_gain                 = foot_x_force_p_gain;
  set_param.request.balance_param.foot_x_force_d_gain                 = foot_x_force_d_gain;
  set_param.request.balance_param.foot_y_force_p_gain                 = foot_y_force_p_gain;
  set_param.request.balance_param.foot_y_force_d_gain                 = foot_y_force_d_gain;
  set_param.request.balance_param.foot_z_force_p_gain                 = foot_z_force_p_gain;
  set_param.request.balance_param.foot_z_force_d_gain                 = foot_z_force_d_gain;
  set_param.request.balance_param.foot_roll_torque_p_gain             = foot_roll_torque_p_gain;
  set_param.request.balance_param.foot_roll_torque_d_gain             = foot_roll_torque_d_gain;
  set_param.request.balance_param.foot_pitch_torque_p_gain            = foot_pitch_torque_p_gain;
  set_param.request.balance_param.foot_pitch_torque_d_gain            = foot_pitch_torque_d_gain;
  set_param.request.balance_param.roll_gyro_cut_off_frequency         = roll_gyro_cut_off_frequency;
  set_param.request.balance_param.pitch_gyro_cut_off_frequency        = pitch_gyro_cut_off_frequency;
  set_param.request.balance_param.roll_angle_cut_off_frequency        = roll_angle_cut_off_frequency;
  set_param.request.balance_param.pitch_angle_cut_off_frequency       = pitch_angle_cut_off_frequency;
  set_param.request.balance_param.foot_x_force_cut_off_frequency      = foot_x_force_cut_off_frequency;
  set_param.request.balance_param.foot_y_force_cut_off_frequency      = foot_y_force_cut_off_frequency;
  set_param.request.balance_param.foot_z_force_cut_off_frequency      = foot_z_force_cut_off_frequency;
  set_param.request.balance_param.foot_roll_torque_cut_off_frequency  = foot_roll_torque_cut_off_frequency;
  set_param.request.balance_param.foot_pitch_torque_cut_off_frequency = foot_pitch_torque_cut_off_frequency;

  return true;
}

bool loadFeedBackGain(thormang3_walking_module_msgs::SetJointFeedBackGain& set_gain)
{
  ros::NodeHandle ros_node;
  std::string feedback_gain_yaml_path = "";
  feedback_gain_yaml_path = ros::package::getPath("thormang3_walking") + "/data/joint_feedback_gain.yaml";

  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(feedback_gain_yaml_path.c_str());
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Failed to load feedback gain yaml file.");
    return false;
  }

  double r_leg_hip_y_p_gain = doc["r_leg_hip_y_p_gain"].as<double>();
  double r_leg_hip_y_d_gain = doc["r_leg_hip_y_d_gain"].as<double>();
  double r_leg_hip_r_p_gain = doc["r_leg_hip_r_p_gain"].as<double>();
  double r_leg_hip_r_d_gain = doc["r_leg_hip_r_d_gain"].as<double>();
  double r_leg_hip_p_p_gain = doc["r_leg_hip_p_p_gain"].as<double>();
  double r_leg_hip_p_d_gain = doc["r_leg_hip_p_d_gain"].as<double>();
  double r_leg_kn_p_p_gain  = doc["r_leg_kn_p_p_gain"].as<double>();
  double r_leg_kn_p_d_gain  = doc["r_leg_kn_p_d_gain"].as<double>();
  double r_leg_an_p_p_gain  = doc["r_leg_an_p_p_gain"].as<double>();
  double r_leg_an_p_d_gain  = doc["r_leg_an_p_d_gain"].as<double>();
  double r_leg_an_r_p_gain  = doc["r_leg_an_r_p_gain"].as<double>();
  double r_leg_an_r_d_gain  = doc["r_leg_an_r_d_gain"].as<double>();
  double l_leg_hip_y_p_gain = doc["l_leg_hip_y_p_gain"].as<double>();
  double l_leg_hip_y_d_gain = doc["l_leg_hip_y_d_gain"].as<double>();
  double l_leg_hip_r_p_gain = doc["l_leg_hip_r_p_gain"].as<double>();
  double l_leg_hip_r_d_gain = doc["l_leg_hip_r_d_gain"].as<double>();
  double l_leg_hip_p_p_gain = doc["l_leg_hip_p_p_gain"].as<double>();
  double l_leg_hip_p_d_gain = doc["l_leg_hip_p_d_gain"].as<double>();
  double l_leg_kn_p_p_gain  = doc["l_leg_kn_p_p_gain"].as<double>();
  double l_leg_kn_p_d_gain  = doc["l_leg_kn_p_d_gain"].as<double>();
  double l_leg_an_p_p_gain  = doc["l_leg_an_p_p_gain"].as<double>();
  double l_leg_an_p_d_gain  = doc["l_leg_an_p_d_gain"].as<double>();
  double l_leg_an_r_p_gain  = doc["l_leg_an_r_p_gain"].as<double>();
  double l_leg_an_r_d_gain  = doc["l_leg_an_r_d_gain"].as<double>();

  set_gain.request.feedback_gain.r_leg_hip_y_p_gain = r_leg_hip_y_p_gain;
  set_gain.request.feedback_gain.r_leg_hip_y_d_gain = r_leg_hip_y_d_gain;
  set_gain.request.feedback_gain.r_leg_hip_r_p_gain = r_leg_hip_r_p_gain;
  set_gain.request.feedback_gain.r_leg_hip_r_d_gain = r_leg_hip_r_d_gain;
  set_gain.request.feedback_gain.r_leg_hip_p_p_gain = r_leg_hip_p_p_gain;
  set_gain.request.feedback_gain.r_leg_hip_p_d_gain = r_leg_hip_p_d_gain;
  set_gain.request.feedback_gain.r_leg_kn_p_p_gain  = r_leg_kn_p_p_gain ;
  set_gain.request.feedback_gain.r_leg_kn_p_d_gain  = r_leg_kn_p_d_gain ;
  set_gain.request.feedback_gain.r_leg_an_p_p_gain  = r_leg_an_p_p_gain ;
  set_gain.request.feedback_gain.r_leg_an_p_d_gain  = r_leg_an_p_d_gain ;
  set_gain.request.feedback_gain.r_leg_an_r_p_gain  = r_leg_an_r_p_gain ;
  set_gain.request.feedback_gain.r_leg_an_r_d_gain  = r_leg_an_r_d_gain ;
  set_gain.request.feedback_gain.l_leg_hip_y_p_gain = l_leg_hip_y_p_gain;
  set_gain.request.feedback_gain.l_leg_hip_y_d_gain = l_leg_hip_y_d_gain;
  set_gain.request.feedback_gain.l_leg_hip_r_p_gain = l_leg_hip_r_p_gain;
  set_gain.request.feedback_gain.l_leg_hip_r_d_gain = l_leg_hip_r_d_gain;
  set_gain.request.feedback_gain.l_leg_hip_p_p_gain = l_leg_hip_p_p_gain;
  set_gain.request.feedback_gain.l_leg_hip_p_d_gain = l_leg_hip_p_d_gain;
  set_gain.request.feedback_gain.l_leg_kn_p_p_gain  = l_leg_kn_p_p_gain ;
  set_gain.request.feedback_gain.l_leg_kn_p_d_gain  = l_leg_kn_p_d_gain ;
  set_gain.request.feedback_gain.l_leg_an_p_p_gain  = l_leg_an_p_p_gain ;
  set_gain.request.feedback_gain.l_leg_an_p_d_gain  = l_leg_an_p_d_gain ;
  set_gain.request.feedback_gain.l_leg_an_r_p_gain  = l_leg_an_r_p_gain ;
  set_gain.request.feedback_gain.l_leg_an_r_d_gain  = l_leg_an_r_d_gain ;

  return true;
}


void setBalanceOn()
{
  // update balance parameter
  thormang3_walking_module_msgs::SetBalanceParam set_balance_param_srv;
  set_balance_param_srv.request.updating_duration =  2.0*1.0; //sec

  if(loadBalanceParam(set_balance_param_srv) == false)
  {
    ROS_ERROR("[Walking OPC]  : Failed to Load Balance YAML");
    return;
  }


  if(g_set_balance_param_client.call(set_balance_param_srv) == true)
  {
    int set_balance_param_srv_result = set_balance_param_srv.response.result;
    if( set_balance_param_srv_result == thormang3_walking_module_msgs::SetBalanceParam::Response::NO_ERROR)
      ROS_INFO("[Walking OPC]  : Succeed to set balance param");
    else
    {
      if(set_balance_param_srv_result & thormang3_walking_module_msgs::SetBalanceParam::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Walking OPC]  : BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE");
      if(set_balance_param_srv_result & thormang3_walking_module_msgs::SetBalanceParam::Response::PREV_REQUEST_IS_NOT_FINISHED)
        ROS_ERROR("[Walking OPC]  : BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED");
//      if(set_balance_param_srv_result & thormang3_walking_module_msgs::SetBalanceParam::Response::CUT_OFF_FREQUENCY_IS_ZERO_OR_NEGATIVE)
//        ROS_ERROR("[Walking OPC]  : BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE");
    }
  }
  else
  {
    ROS_ERROR("[Walking OPC]  : Failed to set balance param ");
  }

  // update joint feed back gain
  thormang3_walking_module_msgs::SetJointFeedBackGain set_feedback_gain_srv;
  set_feedback_gain_srv.request.updating_duration =  2.0*1.0; //sec

  if(loadFeedBackGain(set_feedback_gain_srv) == false)
  {
    ROS_ERROR("[Walking OPC]  : Failed to Load Balance YAML");
    return;
  }


  if(g_set_feedback_gain_client.call(set_feedback_gain_srv) == true)
  {
    int set_feedback_gain_srv_result = set_feedback_gain_srv.response.result;
    if( set_feedback_gain_srv_result == thormang3_walking_module_msgs::SetJointFeedBackGain::Response::NO_ERROR)
      ROS_INFO("[Walking OPC]  : Succeed to set joint feedback gain");
    else
    {
      if(set_feedback_gain_srv_result & thormang3_walking_module_msgs::SetJointFeedBackGain::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Walking OPC]  : FEEDBACK_GAIN_ERR::NOT_ENABLED_WALKING_MODULE");
      if(set_feedback_gain_srv_result & thormang3_walking_module_msgs::SetJointFeedBackGain::Response::PREV_REQUEST_IS_NOT_FINISHED)
        ROS_ERROR("[Walking OPC]  : FEEDBACK_GAIN_ERR::PREV_REQUEST_IS_NOT_FINISHED");
    }
  }
  else
  {
    ROS_ERROR("[Walking OPC]  : Failed to set joint feedback gain");
  }
}

void setBalanceOff()
{
  thormang3_walking_module_msgs::SetBalanceParam set_balance_param_srv;
  set_balance_param_srv.request.updating_duration                             = 1.0; //sec

  if(loadBalanceParam(set_balance_param_srv) == false)
  {
    ROS_ERROR("[Walking OPC]  : Failed to Load Balance YAML");
    return;
  }

  set_balance_param_srv.request.balance_param.hip_roll_swap_angle_rad  = 0;
  set_balance_param_srv.request.balance_param.foot_roll_gyro_p_gain    = 0;
  set_balance_param_srv.request.balance_param.foot_roll_gyro_d_gain    = 0;
  set_balance_param_srv.request.balance_param.foot_pitch_gyro_p_gain   = 0;
  set_balance_param_srv.request.balance_param.foot_pitch_gyro_d_gain   = 0;
  set_balance_param_srv.request.balance_param.foot_roll_angle_p_gain   = 0;
  set_balance_param_srv.request.balance_param.foot_roll_angle_d_gain   = 0;
  set_balance_param_srv.request.balance_param.foot_pitch_angle_p_gain  = 0;
  set_balance_param_srv.request.balance_param.foot_pitch_angle_d_gain  = 0;
  set_balance_param_srv.request.balance_param.foot_x_force_p_gain      = 0;
  set_balance_param_srv.request.balance_param.foot_x_force_d_gain      = 0;
  set_balance_param_srv.request.balance_param.foot_y_force_p_gain      = 0;
  set_balance_param_srv.request.balance_param.foot_y_force_d_gain      = 0;
  set_balance_param_srv.request.balance_param.foot_z_force_p_gain      = 0;
  set_balance_param_srv.request.balance_param.foot_z_force_d_gain      = 0;
  set_balance_param_srv.request.balance_param.foot_roll_torque_p_gain  = 0;
  set_balance_param_srv.request.balance_param.foot_roll_torque_d_gain  = 0;
  set_balance_param_srv.request.balance_param.foot_pitch_torque_p_gain = 0;
  set_balance_param_srv.request.balance_param.foot_pitch_torque_d_gain = 0;

  if(g_set_balance_param_client.call(set_balance_param_srv) == true)
  {
    int set_balance_param_srv_result = set_balance_param_srv.response.result;
    if( set_balance_param_srv_result == thormang3_walking_module_msgs::SetBalanceParam::Response::NO_ERROR)
      ROS_INFO("[Walking OPC]  : Succeed to set balance param");
    else
    {
      if(set_balance_param_srv_result & thormang3_walking_module_msgs::SetBalanceParam::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Walking OPC]  : BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE");
      if(set_balance_param_srv_result & thormang3_walking_module_msgs::SetBalanceParam::Response::PREV_REQUEST_IS_NOT_FINISHED)
        ROS_ERROR("[Walking OPC]  : BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED");
//      if(set_balance_param_srv_result & thormang3_walking_module_msgs::SetBalanceParam::Response::CUT_OFF_FREQUENCY_IS_ZERO_OR_NEGATIVE)
//        ROS_ERROR("[Walking OPC]  : BALANCE_PARAM_ERR::TIME_CONST_IS_ZERO_OR_NEGATIVE");
    }
  }
  else
  {
    ROS_ERROR("[Walking OPC]  : Failed to set balance param ");
  }
}

void setWalkingCommand(thormang3_foot_step_generator::FootStepCommand msg)
{
  set_walking_command_pub_.publish(msg);

  std::stringstream ss;
  ss << "Set Walking Command : " << msg.command << std::endl;
  ss << "- Number of Step : " << msg.step_num << std::endl;
  ss << "- Step Length : " << msg.step_length << std::endl;
  ss << "- Side Step Length : " << msg.side_step_length << std::endl;
  ss << "- Rotation Angle : " << msg.step_angle_rad << std::endl;
}