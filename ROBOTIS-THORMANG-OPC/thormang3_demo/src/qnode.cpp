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

/* Author: Kayman Jung */

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "../include/thormang3_demo/qnode.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace thormang3_demo
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

QNodeThor3::QNodeThor3(int argc, char** argv)
    : init_argc_(argc),
      init_argv_(argv),
      marker_name_("THORMANG3_demo_marker"),
      frame_id_("pelvis_link")
{
  // code to DEBUG
  debug_print_ = false;

  if (argc >= 2)
  {
    std::string debug_code(argv[1]);
    if (debug_code == "debug")
      debug_print_ = true;
    else
      debug_print_ = false;
  }
}

QNodeThor3::~QNodeThor3()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }

  wait();
}

bool QNodeThor3::init()
{
  ros::init(init_argc_, init_argv_, "thormang3_demo");

  if (ros::master::check() == false)
  {
    return false;
  }

  ros::start();  // explicitly needed since our nodehandle is going out of scope.

  ros::NodeHandle nh;

  package_name_ = ros::package::getPath("thormang3_demo");

  balance_yaml_path_ = nh.param<std::string>("balance_file_path", package_name_ + "/config/balance_param.yaml");
  joint_feedback_yaml_path_ = nh.param<std::string>("joint_feedback_file_path", package_name_ + "/config/joint_feedback_gain.yaml");

  // Add your ros communications here.
  status_msg_sub_ = nh.subscribe("/robotis/status", 10, &QNodeThor3::statusMsgCallback, this);
  current_module_control_sub_ = nh.subscribe("/robotis/present_joint_ctrl_modules", 10,
                                             &QNodeThor3::refreshCurrentJointControlCallback, this);
  current_joint_states_sub_ = nh.subscribe("/robotis/present_joint_states", 10,
                                           &QNodeThor3::updateHeadJointStatesCallback, this);

  get_module_control_client_ = nh.serviceClient<robotis_controller_msgs::GetJointModule>(
      "/robotis/get_present_joint_ctrl_modules");

  move_lidar_pub_ = nh.advertise<std_msgs::String>("/robotis/head_control/move_lidar", 0);
  module_control_pub_ = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 0);
  module_control_preset_pub_ = nh.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  init_pose_pub_ = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  init_ft_pub_ = nh.advertise<std_msgs::String>("/robotis/feet_ft/ft_calib_command", 0);

  init_ft_foot_sub_ = nh.subscribe("/robotis/feet_ft/both_ft_value", 10, &QNodeThor3::initFTFootCallback, this);

  // demo
  rviz_clicked_point_sub_ = nh.subscribe("clicked_point", 0, &QNodeThor3::pointStampedCallback, this);
  interactive_marker_server_.reset(new interactive_markers::InteractiveMarkerServer("THORMANG_Pose", "", false));

  // Manipulation
  kenematics_pose_sub_ = nh.subscribe("/thormang3_demo/ik_target_pose", 10, &QNodeThor3::getKinematicsPoseCallback,
                                      this);

  send_ini_pose_msg_pub_ = nh.advertise<std_msgs::String>("/robotis/manipulation/ini_pose_msg", 0);
  send_des_joint_msg_pub_ = nh.advertise<thormang3_manipulation_module_msgs::JointPose>(
      "/robotis/manipulation/joint_pose_msg", 0);
  send_ik_msg_pub_ = nh.advertise<thormang3_manipulation_module_msgs::KinematicsPose>(
      "/robotis/manipulation/kinematics_pose_msg", 0);

  get_joint_pose_client_ = nh.serviceClient<thormang3_manipulation_module_msgs::GetJointPose>(
      "/robotis/manipulation/get_joint_pose");
  get_kinematics_pose_client_ = nh.serviceClient<thormang3_manipulation_module_msgs::GetKinematicsPose>(
      "/robotis/manipulation/get_kinematics_pose");
  send_gripper_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis/gripper/joint_pose_msg", 0);

  // Walking
  set_balance_param_client_ = nh.serviceClient<thormang3_walking_module_msgs::SetBalanceParam>(
      "/robotis/walking/set_balance_param");
  set_joint_feedback_gain_client_ = nh.serviceClient<thormang3_walking_module_msgs::SetJointFeedBackGain>("/robotis/walking/joint_feedback_gain");
  set_walking_command_pub_ = nh.advertise<thormang3_foot_step_generator::FootStepCommand>(
      "/robotis/thormang3_foot_step_generator/walking_command", 0);
  set_walking_footsteps_pub_ = nh.advertise<thormang3_foot_step_generator::Step2DArray>(
      "/robotis/thormang3_foot_step_generator/footsteps_2d", 0);
  set_walking_balance_pub_ = nh.advertise<std_msgs::Bool>("/robotis/thormang3_foot_step_generator/balance_command", 0);

  humanoid_footstep_client_ = nh.serviceClient<humanoid_nav_msgs::PlanFootsteps>("plan_footsteps");
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/robotis/demo/foot_step_marker", 0);
  pose_sub_ = nh.subscribe("/robotis/demo/pose", 10, &QNodeThor3::poseCallback, this);

  // Head control
  set_head_joint_angle_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states", 0);

  // Action
  motion_index_pub_ = nh.advertise<std_msgs::Int32>("/robotis/demo/action_index", 0);
  motion_page_pub_ = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);

  // Overload - Alarm
  overload_com_pub_ = nh.advertise<std_msgs::String>("/robotis/overload/command", 0);
  overload_status_sub_ = nh.subscribe("/robotis/overload/status", 10, &QNodeThor3::overloadStatusCallback, this);

  // Config
  std::string default_config_path = ros::package::getPath("thormang3_demo") + "/config/demo_config.yaml";
  std::string config_path = nh.param<std::string>("demo_config", default_config_path);
  parseJointNameFromYaml(config_path);

  std::string motion_path = ros::package::getPath("thormang3_demo") + "/config/motion.yaml";
  parseMotionMapFromYaml(motion_path);

  // set start time
  start_time_ = ros::Time::now();

  // start qthread
  start();

  return true;
}

void QNodeThor3::run()
{
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  interactive_marker_server_.reset();

  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNodeThor3::parseJointNameFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load id_joint table yaml.");
    return;
  }

  // parse id_joint table
  YAML::Node id_sub_node = doc["id_joint"];
  for (YAML::iterator yaml_it = id_sub_node.begin(); yaml_it != id_sub_node.end(); ++yaml_it)
  {
    int id;
    std::string joint_name;

    id = yaml_it->first.as<int>();
    joint_name = yaml_it->second.as<std::string>();

    id_joint_table_[id] = joint_name;
    joint_id_table_[joint_name] = id;

    ROS_DEBUG_STREAM_COND(debug_print_, "Joint ID : " << id << " - " << joint_name);
  }

  // parse module
  std::vector<std::string> modules = doc["module_list"].as<std::vector<std::string> >();

  int module_index = 0;
  for (std::vector<std::string>::iterator module_it = modules.begin(); module_it != modules.end(); ++module_it)
  {
    std::string module_name = *module_it;

    index_mode_table_[module_index] = module_name;
    mode_index_table_[module_name] = module_index++;

    using_mode_table_[module_name] = false;
  }

  // parse module_joint preset
  YAML::Node sub_node = doc["module_button"];
  for (YAML::iterator button_it = sub_node.begin(); button_it != sub_node.end(); ++button_it)
  {
    int key;
    std::string module_name;

    key = button_it->first.as<int>();
    module_name = button_it->second.as<std::string>();

    module_table_[key] = module_name;

    ROS_DEBUG_STREAM_COND(debug_print_, "Preset : " << module_name);
  }
}

void QNodeThor3::parseMotionMapFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load motion yaml.");
    return;
  }

  // parse motion_table
  YAML::Node motion_sub_node = doc["motion"];
  for (YAML::iterator motion_it = motion_sub_node.begin(); motion_it != motion_sub_node.end(); ++motion_it)
  {
    int motion_index;
    std::string motion_name;

    motion_index = motion_it->first.as<int>();
    motion_name = motion_it->second.as<std::string>();

    motion_table_[motion_index] = motion_name;

    ROS_DEBUG_STREAM_COND(debug_print_, "Motion Index : " << motion_index << " - " << motion_name);
  }
}

// joint id -> joint name
bool QNodeThor3::getJointNameFromID(const int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator map_it;

  map_it = id_joint_table_.find(id);
  if (map_it == id_joint_table_.end())
    return false;

  joint_name = map_it->second;
  return true;
}

// joint name -> joint id
bool QNodeThor3::getIDFromJointName(const std::string &joint_name, int &id)
{
  std::map<std::string, int>::iterator map_it;

  map_it = joint_id_table_.find(joint_name);
  if (map_it == joint_id_table_.end())
    return false;

  id = map_it->second;
  return true;
}

// map index -> joint id & joint name
bool QNodeThor3::getIDJointNameFromIndex(const int &index, int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator map_it;

  int count = 0;
  for (map_it = id_joint_table_.begin(); map_it != id_joint_table_.end(); ++map_it, count++)
  {
    if (index == count)
    {
      id = map_it->first;
      joint_name = map_it->second;
      return true;
    }
  }

  return false;
}

// mode(module) index -> mode(module) name
std::string QNodeThor3::getModuleName(const int &index)
{
  std::string mode = "";
  std::map<int, std::string>::iterator map_it = index_mode_table_.find(index);

  if (map_it != index_mode_table_.end())
    mode = map_it->second;

  return mode;
}

// mode(module) name -> mode(module) index, fail to find out :-1
int QNodeThor3::getModuleIndex(const std::string &mode_name)
{
  int mode_index = -1;
  std::map<std::string, int>::iterator map_it = mode_index_table_.find(mode_name);

  if (map_it != mode_index_table_.end())
    mode_index = map_it->second;

  return mode_index;
}

// number of mode(module)s
int QNodeThor3::getModuleTableSize()
{
  return index_mode_table_.size();
}

// number of joints
int QNodeThor3::getJointTableSize()
{
  return id_joint_table_.size();
}

void QNodeThor3::clearUsingModule()
{
  for (std::map<std::string, bool>::iterator map_it = using_mode_table_.begin(); map_it != using_mode_table_.end();
      ++map_it)
    map_it->second = false;
}

bool QNodeThor3::isUsingModule(const std::string &module_name)
{
  std::map<std::string, bool>::iterator map_it = using_mode_table_.find(module_name);

  if (map_it == using_mode_table_.end())
    return false;

  return map_it->second;
}

void QNodeThor3::setCurrentControlUI(int mode)
{
  current_control_ui_ = mode;

  ROS_INFO("Current UI : %d", mode);
}

// move to init pose : base module
void QNodeThor3::moveInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub_.publish(init_msg);

  log(Info, "Go to robot initial pose.");
}

void QNodeThor3::initFTCommand(std::string command)
{
  std_msgs::String ft_msg;
  ft_msg.data = command;

  init_ft_pub_.publish(ft_msg);
}

// move head to assemble 3d lidar(pointcloud)
void QNodeThor3::assembleLidar()
{
  std_msgs::String lidar_msg;
  lidar_msg.data = "start";

  move_lidar_pub_.publish(lidar_msg);
  log(Info, "Publish move_lidar topic");
}

// enable mode(module)
void QNodeThor3::enableControlModule(const std::string &mode)
{
  std_msgs::String msg;
  msg.data = mode;

  module_control_preset_pub_.publish(msg);

  std::stringstream ss;
  ss << "Set Mode : " << mode;
  log(Info, ss.str());
}

// get current mode(module) of joints
void QNodeThor3::getJointControlModule()
{
  robotis_controller_msgs::GetJointModule get_joint;
  std::map<std::string, int> service_map;

  // get_joint.request
  std::map<int, std::string>::iterator map_it;
  int index = 0;
  for (map_it = id_joint_table_.begin(); map_it != id_joint_table_.end(); ++map_it, index++)
  {
    get_joint.request.joint_name.push_back(map_it->second);
    service_map[map_it->second] = index;
  }

  if (get_module_control_client_.call(get_joint))
  {
    // get_joint.response
    std::vector<int> modules;
    modules.resize(getJointTableSize());

    // clear current using modules
    clearUsingModule();

    for (int ix = 0; ix < get_joint.response.joint_name.size(); ix++)
    {
      std::string joint_name = get_joint.response.joint_name[ix];
      std::string module_name = get_joint.response.module_name[ix];

      std::map<std::string, int>::iterator service_iter = service_map.find(joint_name);
      if (service_iter == service_map.end())
        continue;

      index = service_iter->second;

      service_iter = mode_index_table_.find(module_name);
      if (service_iter == mode_index_table_.end())
        continue;

      ROS_DEBUG_STREAM_COND(debug_print_, "joint[" << ix << "] : " << service_iter->second);

      modules.at(index) = service_iter->second;

      std::map<std::string, bool>::iterator module_iter = using_mode_table_.find(module_name);
      if (module_iter != using_mode_table_.end())
        module_iter->second = true;
    }

    // update ui
    Q_EMIT updatePresentJointControlModules(modules);
    log(Info, "Get current Mode");
  }
  else
    log(Error, "Fail to get current joint control module.");
}

void QNodeThor3::refreshCurrentJointControlCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg)
{
  ROS_INFO("set current joint module");

  std::vector<int> modules;
  modules.resize(getJointTableSize());

  std::map<std::string, int> joint_module;

  // clear current using modules
  clearUsingModule();

  for (int ix = 0; ix < msg->joint_name.size(); ix++)
  {
    std::string joint_name = msg->joint_name[ix];
    std::string module_name = msg->module_name[ix];

    joint_module[joint_name] = getModuleIndex(module_name);

    std::map<std::string, bool>::iterator module_iter = using_mode_table_.find(module_name);
    if (module_iter != using_mode_table_.end())
      module_iter->second = true;
  }

  for (int ix = 0; ix < getJointTableSize(); ix++)
  {
    int id = 0;
    std::string joint_name = "";

    if (getIDJointNameFromIndex(ix, id, joint_name) == false)
      continue;

    std::map<std::string, int>::iterator module_iter = joint_module.find(joint_name);
    if (module_iter == joint_module.end())
      continue;

    ROS_DEBUG_STREAM_COND(debug_print_, "joint[" << ix << "] : " << module_iter->second);
    modules.at(ix) = module_iter->second;
  }

  // update ui
  Q_EMIT updatePresentJointControlModules(modules);

  log(Info, "Applied Mode", "Manager");
}

void QNodeThor3::updateHeadJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  double head_pan, head_tilt;
  int count_getting_joint = 0;

  for (int ix = 0; ix < msg->name.size(); ix++)
  {
    if (msg->name[ix] == "head_y")
    {
      head_pan = -msg->position[ix];
      count_getting_joint += 1;
    }
    else if (msg->name[ix] == "head_p")
    {
      head_tilt = -msg->position[ix];
      count_getting_joint += 1;
    }

    if (count_getting_joint == 2)
      break;
  }

  if (count_getting_joint > 0)
    Q_EMIT updateHeadJointsAngle(head_pan, head_tilt);
}

void QNodeThor3::initFTFootCallback(const thormang3_feet_ft_module_msgs::BothWrench::ConstPtr &msg)
{
  std::stringstream ss;
  ss << "Type : " << msg->name << std::endl;
  ss << " - Right - " << std::endl << msg->right << std::endl;
  ss << " - Left - " << std::endl << msg->left;

  log(Info, ss.str());
}

void QNodeThor3::setHeadJoint(double pan, double tilt)
{
  sensor_msgs::JointState head_angle_msg;

  head_angle_msg.name.push_back("head_y");
  head_angle_msg.name.push_back("head_p");

  head_angle_msg.position.push_back(-pan);
  head_angle_msg.position.push_back(-tilt);

  set_head_joint_angle_pub_.publish(head_angle_msg);
}

// Manipulation
void QNodeThor3::sendInitPoseMsg(std_msgs::String msg)
{
  send_ini_pose_msg_pub_.publish(msg);

  log(Info, "Send Ini. Pose");
}

void QNodeThor3::sendDestJointMsg(thormang3_manipulation_module_msgs::JointPose msg)
{
  send_des_joint_msg_pub_.publish(msg);

  log(Info, "Set Des. Joint Vale");

  std::stringstream log_msg;

  log_msg << " \n " << "joint name : " << msg.name << " \n " << "joint value : " << msg.value * RADIAN2DEGREE << " \n ";

  log(Info, log_msg.str());
}

void QNodeThor3::sendIkMsg(thormang3_manipulation_module_msgs::KinematicsPose msg)
{
  send_ik_msg_pub_.publish(msg);

  log(Info, "Solve Inverse Kinematics");
  log(Info, "Set Des. End Effector's Pose : ");

  std::stringstream log_msgs;

  log_msgs << " \n " << "group name : " << msg.name << " \n " << "des. pos. x : " << msg.pose.position.x << " \n "
           << "des. pos. y : " << msg.pose.position.y << " \n " << "des. pos. z : " << msg.pose.position.z << " \n "
           << "des. ori. x : " << msg.pose.orientation.x << " \n " << "des. ori. y : " << msg.pose.orientation.y
           << " \n " << "des. ori. z : " << msg.pose.orientation.z << " \n " << "des. ori. w : "
           << msg.pose.orientation.w << " \n ";

  log(Info, log_msgs.str());
}

void QNodeThor3::sendGripperPosition(sensor_msgs::JointState msg)
{
  // publish gripper angle
  send_gripper_pub_.publish(msg);
}

void QNodeThor3::getJointPose(std::string joint_name)
{
  thormang3_manipulation_module_msgs::GetJointPose get_joint_pose;

  // requeset
  get_joint_pose.request.joint_name = joint_name;

  log(Info, "Get Curr. Joint Value");

  std::stringstream log_msg;

  log_msg << " \n " << "joint name : " << joint_name << " \n ";

  log(Info, log_msg.str());

  //response
  if (get_joint_pose_client_.call(get_joint_pose))
  {
    double joint_value = get_joint_pose.response.joint_value;

    log(Info, "Joint Curr. Value");

    std::stringstream log_msg;

    log_msg << " \n " << "curr. value : " << joint_value << " \n ";

    log(Info, log_msg.str());

    Q_EMIT updateCurrJoint(joint_value);
  }
  else
    log(Error, "fail to get joint pose.");
}

void QNodeThor3::getKinematicsPose(std::string group_name)
{
  thormang3_manipulation_module_msgs::GetKinematicsPose get_kinematics_pose;

  //request
  get_kinematics_pose.request.group_name = group_name;

  log(Info, "Solve Forward Kinematics");

  log(Info, "Get Curr. End Effector's Pose");

  std::stringstream log_msg;

  log_msg << " \n " << "group name : " << group_name << " \n ";

  log(Info, log_msg.str());

  //response
  if (get_kinematics_pose_client_.call(get_kinematics_pose))
  {
    double pos_x = get_kinematics_pose.response.group_pose.position.x;
    double pos_y = get_kinematics_pose.response.group_pose.position.y;
    double pos_z = get_kinematics_pose.response.group_pose.position.z;

    double ori_x = get_kinematics_pose.response.group_pose.orientation.x;
    double ori_y = get_kinematics_pose.response.group_pose.orientation.y;
    double ori_z = get_kinematics_pose.response.group_pose.orientation.z;
    double ori_w = get_kinematics_pose.response.group_pose.orientation.w;

    log(Info, "End Effector Curr. Pose : ");

    std::stringstream log_msg;

    log_msg << " \n " << "curr. pos. x : " << pos_x << " \n " << "curr. pos. y : " << pos_y << " \n "
            << "curr. pos. z : " << pos_z << " \n " << "curr. ori. w : " << ori_w << " \n " << "curr. ori. x : "
            << ori_x << " \n " << "curr. ori. y : " << ori_y << " \n " << "curr. ori. z : " << ori_z << " \n ";

    log(Info, log_msg.str());

    Q_EMIT updateCurrPos(pos_x, pos_y, pos_z);
    Q_EMIT updateCurrOri(ori_x, ori_y, ori_z, ori_w);
  }
  else
    log(Error, "fail to get kinematics pose.");
}

void QNodeThor3::getKinematicsPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  double z_offset = 0.801;
  Q_EMIT updateCurrPos(msg->position.x, msg->position.y, msg->position.z + z_offset);
  Q_EMIT updateCurrOri(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

// Walking
void QNodeThor3::setWalkingCommand(thormang3_foot_step_generator::FootStepCommand msg)
{
  set_walking_command_pub_.publish(msg);

  std::stringstream ss;
  ss << "Set Walking Command : " << msg.command << std::endl;
  ss << "- Number of Step : " << msg.step_num << std::endl;
  ss << "- Step Length : " << msg.step_length << std::endl;
  ss << "- Side Step Length : " << msg.side_step_length << std::endl;
  ss << "- Rotation Angle : " << msg.step_angle_rad << std::endl;

  log(Info, ss.str());
}

void QNodeThor3::setWalkingBalance(bool on_command)
{
  if (on_command == true)
    turnOnBalance();
  else
    turnOffBalance();
}

void QNodeThor3::setWalkingBalanceParam(const double &gyro_gain, const double &ft_gain_ratio,
                                        const double &imu_time_const, const double &ft_time_const)
{
  // todo : make the walking balance param msg and fill it, sent to the mpc of thormange3
  if (loadBalanceParameterFromYaml() == false)
  {
    return;
  }

  //  set_balance_param_srv_.request.updating_duration = 2.0;

  //  set_balance_param_srv_.request.balance_param.gyro_gain = gyro_gain;
  //  set_balance_param_srv_.request.balance_param.foot_x_force_gain *= ft_gain_ratio;
  //  set_balance_param_srv_.request.balance_param.foot_y_force_gain *= ft_gain_ratio;
  //  set_balance_param_srv_.request.balance_param.foot_z_force_gain *= ft_gain_ratio;
  //  set_balance_param_srv_.request.balance_param.foot_roll_torque_gain *= ft_gain_ratio;
  //  set_balance_param_srv_.request.balance_param.foot_pitch_torque_gain *= ft_gain_ratio;

  //  set_balance_param_srv_.request.balance_param.foot_roll_angle_time_constant = imu_time_const;
  //  set_balance_param_srv_.request.balance_param.foot_pitch_angle_time_constant = imu_time_const;

  //  set_balance_param_srv_.request.balance_param.foot_x_force_time_constant = ft_time_const;
  //  set_balance_param_srv_.request.balance_param.foot_y_force_time_constant = ft_time_const;
  //  set_balance_param_srv_.request.balance_param.foot_z_force_time_constant = ft_time_const;
  //  set_balance_param_srv_.request.balance_param.foot_roll_torque_time_constant = ft_time_const;
  //  set_balance_param_srv_.request.balance_param.foot_pitch_torque_time_constant = ft_time_const;

  //  setBalanceParameter();
}

void QNodeThor3::setWalkingFootsteps()
{
  if (preview_foot_steps_.size() != preview_foot_types_.size())
  {
    log(Error, "Footsteps are corrupted.");
    return;
  }
  else if (preview_foot_steps_.size() == 0)
  {
    log(Warn, "No Footsteps");
    return;
  }

  thormang3_foot_step_generator::Step2DArray footsteps;

  for (int ix = 0; ix < preview_foot_steps_.size(); ix++)
  {
    thormang3_foot_step_generator::Step2D step;

    int type = preview_foot_types_[ix];
    if (type == humanoid_nav_msgs::StepTarget::right)
      step.moving_foot = thormang3_foot_step_generator::Step2D::RIGHT_FOOT_SWING;
    else if (type == humanoid_nav_msgs::StepTarget::left)
      step.moving_foot = thormang3_foot_step_generator::Step2D::LEFT_FOOT_SWING;
    else
      step.moving_foot = thormang3_foot_step_generator::Step2D::STANDING;

    step.step2d = preview_foot_steps_[ix];

    footsteps.footsteps_2d.push_back(step);
  }

  set_walking_footsteps_pub_.publish(footsteps);

  log(Info, "Set command to walk using footsteps");

  clearFootsteps();
}

void QNodeThor3::clearFootsteps()
{
  // clear foot step marker array
  visualizePreviewFootsteps(true);

  preview_foot_steps_.clear();
  preview_foot_types_.clear();
}

void QNodeThor3::makeFootstepUsingPlanner()
{
  makeFootstepUsingPlanner(pose_from_ui_);
}

void QNodeThor3::makeFootstepUsingPlanner(const geometry_msgs::Pose &target_foot_pose)
{
  //foot step service
  humanoid_nav_msgs::PlanFootsteps get_step;

  geometry_msgs::Pose2D start;
  geometry_msgs::Pose2D goal;
  goal.x = target_foot_pose.position.x;
  goal.y = target_foot_pose.position.y;

  Eigen::Quaterniond goal_orientation;
  tf::quaternionMsgToEigen(target_foot_pose.orientation, goal_orientation);

  Eigen::Vector3d forward, f_x(1, 0, 0);
  forward = goal_orientation.toRotationMatrix() * f_x;
  double theta = forward.y() > 0 ? acos(forward.x()) : -acos(forward.x());
  goal.theta = theta;

  get_step.request.start = start;
  get_step.request.goal = goal;

  std::stringstream call_msg;
  call_msg << "Start [" << start.x << ", " << start.y << " | " << start.theta << "]" << " , Goal [" << goal.x << ", "
           << goal.y << " | " << goal.theta << "]";
  log(Info, call_msg.str());

  // clear visualization
  visualizePreviewFootsteps(true);

  // init foot steps
  preview_foot_steps_.clear();
  preview_foot_types_.clear();

  if (humanoid_footstep_client_.call(get_step))
  {
    if (get_step.response.result)
    {
      for (int ix = 0; ix < get_step.response.footsteps.size(); ix++)
      {
        // foot step log
        std::stringstream msg_stream;
        int foot_type = get_step.response.footsteps[ix].leg;
        std::string foot = (foot_type == humanoid_nav_msgs::StepTarget::right) ? "right" : "left";
        geometry_msgs::Pose2D foot_pose = get_step.response.footsteps[ix].pose;

        // log footsteps
        msg_stream << "Foot Step #" << ix + 1 << " [ " << foot << "] - [" << foot_pose.x << ", " << foot_pose.y << " | "
                   << (foot_pose.theta * RADIAN2DEGREE) << "]";
        log(Info, msg_stream.str());

        preview_foot_steps_.push_back(foot_pose);
        preview_foot_types_.push_back(foot_type);
      }

      // visualize foot steps
      visualizePreviewFootsteps(false);
    }
    else
    {
      log(Info, "fail to get foot step from planner");
      return;
    }
  }
  else
  {
    log(Error, "cannot call service");
    return;
  }

  return;
}

void QNodeThor3::visualizePreviewFootsteps(bool clear)
{
  if (clear && preview_foot_steps_.size() == 0)
    return;

  visualization_msgs::MarkerArray marker_array;
  ros::Time now = ros::Time::now();
  visualization_msgs::Marker rviz_marker;

  rviz_marker.header.frame_id = "pelvis_link";
  rviz_marker.header.stamp = now;
  rviz_marker.ns = "foot_step_marker";

  rviz_marker.id = 1;
  rviz_marker.type = visualization_msgs::Marker::CUBE;
  rviz_marker.action = (clear == false) ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;

  rviz_marker.scale.x = 0.216;
  rviz_marker.scale.y = 0.144;
  rviz_marker.scale.z = 0.01;

  double alpha = 0.7;
  double height = -0.723;

  for (int ix = preview_foot_types_.size() - 1; ix >= 0; ix--)
  {
    // foot step marker array
    rviz_marker.id += 10;

    if (!clear)
    {
      Eigen::Vector3d marker_position(preview_foot_steps_[ix].x, preview_foot_steps_[ix].y, height);
      Eigen::Vector3d marker_position_offset;

      Eigen::Vector3d toward(1, 0, 0), direction(cos(preview_foot_steps_[ix].theta), sin(preview_foot_steps_[ix].theta),
                                                 0);
      Eigen::Quaterniond marker_orientation(Eigen::Quaterniond::FromTwoVectors(toward, direction));

      if (debug_print_)
      {
        std::stringstream msg;
        msg << "Foot Step #" << ix << " [ " << preview_foot_types_[ix] << "] - [" << rviz_marker.pose.position.x << ", "
            << rviz_marker.pose.position.y << "]";
        log(Info, msg.str());
      }
      alpha *= 0.9;

      // set foot step color
      if (preview_foot_types_[ix] == humanoid_nav_msgs::StepTarget::left)  // left
      {
        rviz_marker.color.r = 0.0;
        rviz_marker.color.g = 0.0;
        rviz_marker.color.b = 1.0;
        rviz_marker.color.a = alpha + 0.3;

        Eigen::Vector3d offset_y(0, 0.015, 0);
        marker_position_offset = marker_orientation.toRotationMatrix() * offset_y;

      }
      else if (preview_foot_types_[ix] == humanoid_nav_msgs::StepTarget::right)  //right
      {
        rviz_marker.color.r = 1.0;
        rviz_marker.color.g = 0.0;
        rviz_marker.color.b = 0.0;
        rviz_marker.color.a = alpha + 0.3;

        Eigen::Vector3d offset_y(0, -0.015, 0);
        marker_position_offset = marker_orientation.toRotationMatrix() * offset_y;
      }

      marker_position = marker_position_offset + marker_position;

      tf::pointEigenToMsg(marker_position, rviz_marker.pose.position);
      tf::quaternionEigenToMsg(marker_orientation, rviz_marker.pose.orientation);

      // apply foot x offset
    }

    marker_array.markers.push_back(rviz_marker);
  }

  // publish foot step marker array
  if (clear == false)
    log(Info, "Visualize Preview Footstep Marker Array");
  else
    log(Info, "Clear Visualize Preview Footstep Marker Array");

  marker_pub_.publish(marker_array);
}

void QNodeThor3::setBalanceParameter()
{
  bool service_result = false;

  // call service
  service_result = set_balance_param_client_.call(set_balance_param_srv_);
  if (service_result == true)
  {
    int _result = set_balance_param_srv_.response.result;
    if (_result == thormang3_walking_module_msgs::SetBalanceParam::Response::NO_ERROR)
    {
      ROS_INFO("[Demo]  : Succeed to set balance param");
      ROS_INFO("[Demo]  : Please wait 2 sec for turning on balance");
      log(Info, "Set Walking Balance parameters");
    }
    else
    {
      if (_result & thormang3_walking_module_msgs::SetBalanceParam::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::NOT_ENABLED_WALKING_MODULE");
      if (_result & thormang3_walking_module_msgs::SetBalanceParam::Response::PREV_REQUEST_IS_NOT_FINISHED)
        ROS_ERROR("[Demo]  : BALANCE_PARAM_ERR::PREV_REQUEST_IS_NOT_FINISHED");
    }
  }
  else
    ROS_ERROR("[Demo]  : Failed to set balance param ");

  setFeedBackGain();
}

bool QNodeThor3::loadBalanceParameterFromYaml()
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(balance_yaml_path_.c_str());

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

    set_balance_param_srv_.request.updating_duration                                 = 2.0;
    set_balance_param_srv_.request.balance_param.cob_x_offset_m                      =  cob_x_offset_m                     ;
    set_balance_param_srv_.request.balance_param.cob_y_offset_m                      =  cob_y_offset_m                     ;
    set_balance_param_srv_.request.balance_param.hip_roll_swap_angle_rad             =  hip_roll_swap_angle_rad            ;
    set_balance_param_srv_.request.balance_param.foot_roll_gyro_p_gain               =  foot_roll_gyro_p_gain              ;
    set_balance_param_srv_.request.balance_param.foot_roll_gyro_d_gain               =  foot_roll_gyro_d_gain              ;
    set_balance_param_srv_.request.balance_param.foot_pitch_gyro_p_gain              =  foot_pitch_gyro_p_gain             ;
    set_balance_param_srv_.request.balance_param.foot_pitch_gyro_d_gain              =  foot_pitch_gyro_d_gain             ;
    set_balance_param_srv_.request.balance_param.foot_roll_angle_p_gain              =  foot_roll_angle_p_gain             ;
    set_balance_param_srv_.request.balance_param.foot_roll_angle_d_gain              =  foot_roll_angle_d_gain             ;
    set_balance_param_srv_.request.balance_param.foot_pitch_angle_p_gain             =  foot_pitch_angle_p_gain            ;
    set_balance_param_srv_.request.balance_param.foot_pitch_angle_d_gain             =  foot_pitch_angle_d_gain            ;
    set_balance_param_srv_.request.balance_param.foot_x_force_p_gain                 =  foot_x_force_p_gain                ;
    set_balance_param_srv_.request.balance_param.foot_x_force_d_gain                 =  foot_x_force_d_gain                ;
    set_balance_param_srv_.request.balance_param.foot_y_force_p_gain                 =  foot_y_force_p_gain                ;
    set_balance_param_srv_.request.balance_param.foot_y_force_d_gain                 =  foot_y_force_d_gain                ;
    set_balance_param_srv_.request.balance_param.foot_z_force_p_gain                 =  foot_z_force_p_gain                ;
    set_balance_param_srv_.request.balance_param.foot_z_force_d_gain                 =  foot_z_force_d_gain                ;
    set_balance_param_srv_.request.balance_param.foot_roll_torque_p_gain             =  foot_roll_torque_p_gain            ;
    set_balance_param_srv_.request.balance_param.foot_roll_torque_d_gain             =  foot_roll_torque_d_gain            ;
    set_balance_param_srv_.request.balance_param.foot_pitch_torque_p_gain            =  foot_pitch_torque_p_gain           ;
    set_balance_param_srv_.request.balance_param.foot_pitch_torque_d_gain            =  foot_pitch_torque_d_gain           ;
    set_balance_param_srv_.request.balance_param.roll_gyro_cut_off_frequency         =  roll_gyro_cut_off_frequency        ;
    set_balance_param_srv_.request.balance_param.pitch_gyro_cut_off_frequency        =  pitch_gyro_cut_off_frequency       ;
    set_balance_param_srv_.request.balance_param.roll_angle_cut_off_frequency        =  roll_angle_cut_off_frequency       ;
    set_balance_param_srv_.request.balance_param.pitch_angle_cut_off_frequency       =  pitch_angle_cut_off_frequency      ;
    set_balance_param_srv_.request.balance_param.foot_x_force_cut_off_frequency      =  foot_x_force_cut_off_frequency     ;
    set_balance_param_srv_.request.balance_param.foot_y_force_cut_off_frequency      =  foot_y_force_cut_off_frequency     ;
    set_balance_param_srv_.request.balance_param.foot_z_force_cut_off_frequency      =  foot_z_force_cut_off_frequency     ;
    set_balance_param_srv_.request.balance_param.foot_roll_torque_cut_off_frequency  =  foot_roll_torque_cut_off_frequency ;
    set_balance_param_srv_.request.balance_param.foot_pitch_torque_cut_off_frequency =  foot_pitch_torque_cut_off_frequency;

    return true;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Failed to load balance param yaml file.");
    return false;
  }

  return true;
}

bool QNodeThor3::setFeedBackGain()
{
  // load joint feedback gain from yaml file
  bool result_load = loadFeedbackGainFromYaml();

  if (result_load == false)
    return false;

  bool service_result = false;

  // call service
  service_result = set_joint_feedback_gain_client_.call(set_joint_feedback_gain_srv_);
  if (service_result == true)
  {
    int _result = set_joint_feedback_gain_srv_.response.result;

    if (_result == thormang3_walking_module_msgs::SetJointFeedBackGain::Response::NO_ERROR)
    {
      ROS_INFO("[Demo]  : Succeed to set joint feedback gain");
      log(Info, "Set Walking Joint FeedBack gain");
    }
    else
    {
      if (_result & thormang3_walking_module_msgs::SetJointFeedBackGain::Response::NOT_ENABLED_WALKING_MODULE)
        ROS_ERROR("[Demo]  : FRRDBACK_GAIN_ERR::NOT_ENABLED_WALKING_MODULE");
      if (_result & thormang3_walking_module_msgs::SetJointFeedBackGain::Response::PREV_REQUEST_IS_NOT_FINISHED)
        ROS_ERROR("[Demo]  : FRRDBACK_GAIN_ERR::PREV_REQUEST_IS_NOT_FINISHED");
    }
  }
  else
  {
    ROS_ERROR("[Demo]  : Failed to set Joint Feedback Gain");
    log(Error, "Fain to set Walking Joint FeedBack gain");
  }
    return true;
}

bool QNodeThor3::loadFeedbackGainFromYaml()
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(joint_feedback_yaml_path_.c_str());

    set_joint_feedback_gain_srv_.request.updating_duration                 = 2.0;
    set_joint_feedback_gain_srv_.request.feedback_gain.r_leg_hip_y_p_gain  = doc["r_leg_hip_y_p_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.r_leg_hip_y_d_gain  = doc["r_leg_hip_y_d_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.r_leg_hip_r_p_gain  = doc["r_leg_hip_r_p_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.r_leg_hip_r_d_gain  = doc["r_leg_hip_r_d_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.r_leg_hip_p_p_gain  = doc["r_leg_hip_p_p_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.r_leg_hip_p_d_gain  = doc["r_leg_hip_p_d_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.r_leg_kn_p_p_gain   = doc["r_leg_kn_p_p_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.r_leg_kn_p_d_gain   = doc["r_leg_kn_p_d_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.r_leg_an_p_p_gain   = doc["r_leg_an_p_p_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.r_leg_an_p_d_gain   = doc["r_leg_an_p_d_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.r_leg_an_r_p_gain   = doc["r_leg_an_r_p_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.r_leg_an_r_d_gain   = doc["r_leg_an_r_d_gain"].as<double>();

    set_joint_feedback_gain_srv_.request.feedback_gain.l_leg_hip_y_p_gain  = doc["l_leg_hip_y_p_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.l_leg_hip_y_d_gain  = doc["l_leg_hip_y_d_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.l_leg_hip_r_p_gain  = doc["l_leg_hip_r_p_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.l_leg_hip_r_d_gain  = doc["l_leg_hip_r_d_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.l_leg_hip_p_p_gain  = doc["l_leg_hip_p_p_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.l_leg_hip_p_d_gain  = doc["l_leg_hip_p_d_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.l_leg_kn_p_p_gain   = doc["l_leg_kn_p_p_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.l_leg_kn_p_d_gain   = doc["l_leg_kn_p_d_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.l_leg_an_p_p_gain   = doc["l_leg_an_p_p_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.l_leg_an_p_d_gain   = doc["l_leg_an_p_d_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.l_leg_an_r_p_gain   = doc["l_leg_an_r_p_gain"].as<double>();
    set_joint_feedback_gain_srv_.request.feedback_gain.l_leg_an_r_d_gain   = doc["l_leg_an_r_d_gain"].as<double>();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Failed to load joint feedback gain yaml file.");
    return false;
  }

  return true;
}

void QNodeThor3::turnOnBalance()
{
  // load param from yaml file
  bool result_load = loadBalanceParameterFromYaml();

  if (result_load == false)
    return;

  setBalanceParameter();

  log(Info, "Turn On Walking Balance");
}

void QNodeThor3::turnOffBalance()
{
  // load param from yaml file
  bool result_load = loadBalanceParameterFromYaml();

  if (result_load == false)
    return;

  set_balance_param_srv_.request.updating_duration = 2.0;
  set_balance_param_srv_.request.balance_param.foot_roll_gyro_p_gain               =  0.0;
  set_balance_param_srv_.request.balance_param.foot_roll_gyro_d_gain               =  0.0;
  set_balance_param_srv_.request.balance_param.foot_pitch_gyro_p_gain              =  0.0;
  set_balance_param_srv_.request.balance_param.foot_pitch_gyro_d_gain              =  0.0;
  set_balance_param_srv_.request.balance_param.foot_roll_angle_p_gain              =  0.0;
  set_balance_param_srv_.request.balance_param.foot_roll_angle_d_gain              =  0.0;
  set_balance_param_srv_.request.balance_param.foot_pitch_angle_p_gain             =  0.0;
  set_balance_param_srv_.request.balance_param.foot_pitch_angle_d_gain             =  0.0;
  set_balance_param_srv_.request.balance_param.foot_x_force_p_gain                 =  0.0;
  set_balance_param_srv_.request.balance_param.foot_x_force_d_gain                 =  0.0;
  set_balance_param_srv_.request.balance_param.foot_y_force_p_gain                 =  0.0;
  set_balance_param_srv_.request.balance_param.foot_y_force_d_gain                 =  0.0;
  set_balance_param_srv_.request.balance_param.foot_z_force_p_gain                 =  0.0;
  set_balance_param_srv_.request.balance_param.foot_z_force_d_gain                 =  0.0;
  set_balance_param_srv_.request.balance_param.foot_roll_torque_p_gain             =  0.0;
  set_balance_param_srv_.request.balance_param.foot_roll_torque_d_gain             =  0.0;
  set_balance_param_srv_.request.balance_param.foot_pitch_torque_p_gain            =  0.0;
  set_balance_param_srv_.request.balance_param.foot_pitch_torque_d_gain            =  0.0;

  setBalanceParameter();

  log(Info, "Turn Off Walking Balance");
}

// Motion
void QNodeThor3::playMotion(int motion_index, bool to_action_script)
{
  if (motion_table_.find(motion_index) == motion_table_.end())
  {
    log(Error, "Motion index is not valid.");
    return;
  }

  std::stringstream log_stream;
  switch (motion_index)
  {
    case -2:
      log_stream << "BRAKE Motion";
      break;

    case -1:
      log_stream << "STOP Motion";
      break;

    default:
      std::string motion_name = motion_table_[motion_index];
      log_stream << "Play Motion : [" << motion_index << "] " << motion_name;
  }

  // publish motion index
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;

  if (to_action_script == true)
    motion_index_pub_.publish(motion_msg);
  else
    motion_page_pub_.publish(motion_msg);

  log(Info, log_stream.str());
}

void QNodeThor3::poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  switch (current_control_ui_)
  {
    case WALKING_UI:
    {
      pose_from_ui_ = *msg;
      log(Info, "Get Pose For Step");
      break;
    }

    case MANIPULATION_UI:
    {
      double z_offset = 0.723;
      Q_EMIT updateCurrPos(msg->position.x, msg->position.y, msg->position.z + z_offset);
      Q_EMIT updateCurrOri(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
      log(Info, "Get Pose For IK");
      break;
    }

    default:
      break;
  }
}

// demo
void QNodeThor3::pointStampedCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  ROS_INFO("get position from rviz");

  frame_id_ = msg->header.frame_id;

  // update point ui
  Q_EMIT updateDemoPoint(msg->point);
}

void QNodeThor3::interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  // event
  switch (feedback->event_type)
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      current_pose_ = feedback->pose;

      // update pose ui
      Q_EMIT updateDemoPose(feedback->pose);

      break;
    }
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      break;

    default:
      break;
  }

  interactive_marker_server_->applyChanges();
}

void QNodeThor3::makeInteractiveMarker(const geometry_msgs::Pose &marker_pose)
{
  if (frame_id_ == "")
  {
    ROS_ERROR("No frame id!!!");
    // return;

    frame_id_ = "pelvis_link";
  }

  ROS_INFO_STREAM(
      "Make Interactive Marker! - " << marker_pose.position.x << ", " << marker_pose.position.y << ", " << marker_pose.position.z << " [" << marker_pose.orientation.x << ", " << marker_pose.orientation.y << ", " << marker_pose.orientation.z << " | " << marker_pose.orientation.w << "]");

  interactive_marker_server_->clear();

  visualization_msgs::InteractiveMarker interactive_marker;
  interactive_marker.pose = marker_pose;    // set pose

  // Visualize Interactive Marker
  interactive_marker.header.frame_id = frame_id_;
  interactive_marker.scale = 0.3;

  interactive_marker.name = marker_name_;  //"pose_marker";
  interactive_marker.description = "3D Pose Control";

  // ----- center marker
  visualization_msgs::InteractiveMarkerControl center_marker_control;

  center_marker_control.always_visible = true;
  center_marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;

  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;

  // center cube
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;

  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  center_marker_control.markers.push_back(marker);

  // axis x
  marker.pose.position.x = 0.05;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  marker.scale.x = 0.1;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  center_marker_control.markers.push_back(marker);

  // axis y
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.05;
  marker.pose.position.z = 0.0;

  marker.scale.x = 0.01;
  marker.scale.y = 0.1;
  marker.scale.z = 0.01;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  center_marker_control.markers.push_back(marker);

  // axis z
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.05;

  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.1;

  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  center_marker_control.markers.push_back(marker);

  interactive_marker.controls.push_back(center_marker_control);

  // ----- controller
  visualization_msgs::InteractiveMarkerControl interactive_control;

  // move and rotate along axis x : default
  interactive_control.orientation.x = 1;
  interactive_control.orientation.y = 0;
  interactive_control.orientation.z = 0;
  interactive_control.orientation.w = 1;
  interactive_control.name = "rotate";
  interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactive_marker.controls.push_back(interactive_control);
  interactive_control.name = "move";
  interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(interactive_control);

  // move and rotate along axis y
  interactive_control.orientation.x = 0;
  interactive_control.orientation.y = 1;
  interactive_control.orientation.z = 0;
  interactive_control.orientation.w = 1;
  interactive_control.name = "rotate";
  interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactive_marker.controls.push_back(interactive_control);
  interactive_control.name = "move";
  interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(interactive_control);

  // move and rotate along axis z
  interactive_control.orientation.x = 0;
  interactive_control.orientation.y = 0;
  interactive_control.orientation.z = 1;
  interactive_control.orientation.w = 1;
  interactive_control.name = "rotate";
  interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactive_marker.controls.push_back(interactive_control);
  interactive_control.name = "move";
  interactive_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactive_marker.controls.push_back(interactive_control);

  interactive_marker_server_->insert(interactive_marker);
  interactive_marker_server_->setCallback(interactive_marker.name,
                                          boost::bind(&QNodeThor3::interactiveMarkerFeedback, this, _1));

  interactive_marker_server_->applyChanges();
}

void QNodeThor3::updateInteractiveMarker(const geometry_msgs::Pose &pose)
{
  ROS_INFO("Update Interactive Marker Pose");

  visualization_msgs::InteractiveMarker interactive_marker;
  bool result_getting = false;

  result_getting = interactive_marker_server_->get(marker_name_, interactive_marker);
  if (result_getting == false)
  {
    ROS_ERROR("No Interactive marker to set pose");
    return;
  }

  interactive_marker_server_->setPose(interactive_marker.name, pose);
  interactive_marker_server_->applyChanges();
}

void QNodeThor3::getInteractiveMarkerPose()
{
  ROS_INFO("Get Interactive Marker Pose");

  visualization_msgs::InteractiveMarker _interactive_marker;
  if (!(interactive_marker_server_->get(marker_name_, _interactive_marker)))
  {
    ROS_ERROR("No Interactive marker to get pose");
    return;
  }

  // update pose ui
  Q_EMIT updateDemoPose(_interactive_marker.pose);

  clearInteractiveMarker();
}

void QNodeThor3::clearInteractiveMarker()
{
  ROS_INFO("Clear Interactive Marker");

  // clear and apply
  interactive_marker_server_->clear();
  interactive_marker_server_->applyChanges();
}

void QNodeThor3::kickDemo(const std::string &kick_foot)
{
  if (kick_foot == "right kick")
  {
    bool result = loadBalanceParameterFromYaml();
    if (result == false)
      return;

    double old_hip_swap = set_balance_param_srv_.request.balance_param.hip_roll_swap_angle_rad;
    set_balance_param_srv_.request.balance_param.hip_roll_swap_angle_rad = 0;
    set_balance_param_srv_.request.balance_param.cob_x_offset_m -= 0.03;
    set_balance_param_srv_.request.balance_param.cob_y_offset_m += 0.02;
    setBalanceParameter();

    thormang3_foot_step_generator::FootStepCommand msg;
    msg.command = kick_foot;
    setWalkingCommand(msg);

    // wait for kick
    usleep(7.2 * 1000 * 1000);

    set_balance_param_srv_.request.balance_param.hip_roll_swap_angle_rad = old_hip_swap;
    set_balance_param_srv_.request.balance_param.cob_x_offset_m += 0.03;
    set_balance_param_srv_.request.balance_param.cob_y_offset_m -= 0.02;
    setBalanceParameter();

    // wait for recovering balance
    usleep(2 * 1000 * 1000);
  }
  else if (kick_foot == "left kick")
  {
    bool result = loadBalanceParameterFromYaml();
    if (result == false)
      return;

    double old_hip_swap = set_balance_param_srv_.request.balance_param.hip_roll_swap_angle_rad;
    set_balance_param_srv_.request.balance_param.cob_x_offset_m -= 0.03;
    set_balance_param_srv_.request.balance_param.cob_y_offset_m -= 0.02;
    setBalanceParameter();

    thormang3_foot_step_generator::FootStepCommand msg;
    msg.command = kick_foot;
    setWalkingCommand(msg);

    // wait for kick
    usleep(7.2 * 1000 * 1000);

    set_balance_param_srv_.request.balance_param.hip_roll_swap_angle_rad = old_hip_swap;
    set_balance_param_srv_.request.balance_param.cob_x_offset_m += 0.03;
    set_balance_param_srv_.request.balance_param.cob_y_offset_m += 0.02;
    setBalanceParameter();

    // wait for recovering balance
    usleep(2 * 1000 * 1000);
  }
}

// Overload
void QNodeThor3::overloadStatusCallback(const thormang3_alarm_module_msgs::JointOverloadStatus::ConstPtr &msg)
{
  for(int ix = 0; ix < msg->name.size(); ix++)
  {
    if(msg->name[ix].find("r_leg_kn_p") != std::string::npos)
    {
      Q_EMIT updateOverloadStatus(Right, msg->status[ix], msg->warning_count[ix], msg->error_count[ix]);
    }
    else if(msg->name[ix].find("l_leg_kn_p") != std::string::npos)
    {
      Q_EMIT updateOverloadStatus(Left, msg->status[ix], msg->warning_count[ix], msg->error_count[ix]);
    }
  }
}

void QNodeThor3::publishAlarmCommand(const std::string &command)
{
  std_msgs::String comm_msg;
  comm_msg.data = command;

  overload_com_pub_.publish(comm_msg);
  log(Info, "send overload command" + command);
}

// LOG
void QNodeThor3::statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
  log((LogLevel) msg->type, msg->status_msg, msg->module_name);
}

void QNodeThor3::log(const LogLevel &level, const std::string &msg, std::string sender)
{
  logging_model_.insertRows(logging_model_.rowCount(), 1);
  std::stringstream logging_model_msg;

  ros::Duration duration_time = ros::Time::now() - start_time_;
  int current_time = duration_time.sec;
  int min_time = 0, sec_time = 0;
  min_time = (int) (current_time / 60);
  sec_time = (int) (current_time % 60);

  std::stringstream min_str, sec_str;
  if (min_time < 10)
    min_str << "0";
  if (sec_time < 10)
    sec_str << "0";
  min_str << min_time;
  sec_str << sec_time;

  std::stringstream stream_sender;
  stream_sender << "[" << sender << "] ";

  switch (level)
  {
    case (Debug):
    {
      ROS_DEBUG_STREAM(msg);
      logging_model_msg << "[DEBUG] [" << min_str.str() << ":" << sec_str.str() << "]: " << stream_sender.str() << msg;
      break;
    }
    case (Info):
    {
      ROS_INFO_STREAM(msg);
      logging_model_msg << "[INFO] [" << min_str.str() << ":" << sec_str.str() << "]: " << stream_sender.str() << msg;
      break;
    }
    case (Warn):
    {
      ROS_WARN_STREAM(msg);
      logging_model_msg << "[WARN] [" << min_str.str() << ":" << sec_str.str() << "]: " << stream_sender.str() << msg;
      break;
    }
    case (Error):
    {
      ROS_ERROR_STREAM(msg);
      logging_model_msg << "<ERROR> [" << min_str.str() << ":" << sec_str.str() << "]: " << stream_sender.str() << msg;
      break;
    }
    case (Fatal):
    {
      ROS_FATAL_STREAM(msg);
      logging_model_msg << "[FATAL] [" << min_str.str() << ":" << sec_str.str() << "]: " << stream_sender.str() << msg;
      break;
    }
  }

  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(logging_model_.rowCount() - 1), new_row);

  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}

void QNodeThor3::clearLog()
{
  if (logging_model_.rowCount() == 0)
    return;

  logging_model_.removeRows(0, logging_model_.rowCount());
}

}  // namespace thormang3_demo
