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

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/thormang3_demo/main_window.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace thormang3_demo
{

using namespace Qt;

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent),
    qnode_thor3_(argc, argv),
    is_updating_(false)
{
  // code to DEBUG
  debug_print_ = false;
  demo_mode_ = false;

  if (argc >= 2)
  {
    std::string args_code(argv[1]);
    if (args_code == "debug")
      debug_print_ = true;
    else
      debug_print_ = false;

    if (args_code == "demo")
      demo_mode_ = true;
    else
      demo_mode_ = false;
  }

  ui_.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui_.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));  // qApp is a global variable for the application

  readSettings();
  setWindowIcon(QIcon(":/images/icon.png"));

  ui_.tab_manager->setCurrentIndex(0);  // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode_thor3_, SIGNAL(rosShutdown()), this, SLOT(close()));

  qRegisterMetaType<std::vector<int> >("std::vector<int>");
  QObject::connect(&qnode_thor3_, SIGNAL(updatePresentJointControlModules(std::vector<int>)), this,
                   SLOT(updatePresentJointModule(std::vector<int>)));
  QObject::connect(&qnode_thor3_, SIGNAL(updateHeadJointsAngle(double,double)), this,
                   SLOT(updateHeadJointsAngle(double,double)));

  QObject::connect(ui_.head_pan_slider, SIGNAL(valueChanged(int)), this, SLOT(setHeadJointsAngle()));
  QObject::connect(ui_.head_tilt_slider, SIGNAL(valueChanged(int)), this, SLOT(setHeadJointsAngle()));

  QObject::connect(&qnode_thor3_, SIGNAL(updateCurrJoint(double)), this, SLOT(updateCurrJointSpinbox(double)));
  QObject::connect(&qnode_thor3_, SIGNAL(updateCurrPos(double , double , double)), this,
                   SLOT(updateCurrPosSpinbox(double , double , double)));
  QObject::connect(&qnode_thor3_, SIGNAL(updateCurrOri(double , double , double, double)), this,
                   SLOT(updateCurrOriSpinbox(double , double , double , double)));

  QObject::connect(ui_.tabWidget_control, SIGNAL(currentChanged(int)), &qnode_thor3_, SLOT(setCurrentControlUI(int)));

  qRegisterMetaType<geometry_msgs::Point>("geometry_msgs::Point");
  qRegisterMetaType<geometry_msgs::Pose>("geometry_msgs::Pose");
  connect(&qnode_thor3_, SIGNAL(updateDemoPoint(geometry_msgs::Point)), this,
          SLOT(updatePointPanel(geometry_msgs::Point)));
  connect(&qnode_thor3_, SIGNAL(updateDemoPose(geometry_msgs::Pose)), this, SLOT(updatePosePanel(geometry_msgs::Pose)));

  connect(&qnode_thor3_, SIGNAL(updateOverloadStatus(int,int,int,int)), this, SLOT(updateOverloadStatus(int,int,int,int)));

  /*********************
   ** Logging
   **********************/
  ui_.view_logging->setModel(qnode_thor3_.loggingModel());
  QObject::connect(&qnode_thor3_, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  /*********************
   ** Init
   **********************/
  qnode_thor3_.init();
  initModeUnit();
  setUserShortcut();
  updateModuleUI();
  clearOverload();
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/

void MainWindow::on_button_assemble_lidar_clicked(bool check)
{
  qnode_thor3_.assembleLidar();
}
void MainWindow::on_button_clear_log_clicked(bool check)
{
  qnode_thor3_.clearLog();
}
void MainWindow::on_button_init_pose_clicked(bool check)
{
  qnode_thor3_.moveInitPose();
}

void MainWindow::on_button_ft_air_clicked(bool check)
{
  qnode_thor3_.initFTCommand("ft_air");
}
void MainWindow::on_button_ft_gnd_clicked(bool check)
{
  qnode_thor3_.initFTCommand("ft_gnd");
}
void MainWindow::on_button_ft_calc_clicked(bool check)
{
  qnode_thor3_.initFTCommand("ft_send");
  qnode_thor3_.log(QNodeThor3::Info, "Apply new FT config");
}
void MainWindow::on_button_ft_save_clicked(bool check)
{
  qnode_thor3_.initFTCommand("ft_save");
  qnode_thor3_.log(QNodeThor3::Info, "Save FT config data.");
}

void MainWindow::on_tabWidget_control_currentChanged(int index)
{
  if (demo_mode_ == false)
    return;

  std::string tab_name = ui_.tabWidget_control->tabText(ui_.tabWidget_control->currentIndex()).toStdString();
  if (tab_name != "Demo")
    ui_.tabWidget_control->currentWidget()->setEnabled(false);
}

// Manipulation
void MainWindow::on_inipose_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "ini_pose";

  qnode_thor3_.sendInitPoseMsg(msg);
}

void MainWindow::on_currjoint_button_clicked(bool check)
{
  qnode_thor3_.getJointPose(ui_.joint_combobox->currentText().toStdString());
}

void MainWindow::on_desjoint_button_clicked(bool check)
{
  thormang3_manipulation_module_msgs::JointPose msg;

  msg.name = ui_.joint_combobox->currentText().toStdString();
  msg.value = deg2rad<double>(ui_.joint_spinbox->value());

  qnode_thor3_.sendDestJointMsg(msg);
}

void MainWindow::on_get_despos_button_clicked(bool check)
{
  double z_offset = 0.723;

  updateCurrPosSpinbox(ui_.dSpinBox_marker_pos_x->value(), ui_.dSpinBox_marker_pos_y->value(),
                       ui_.dSpinBox_marker_pos_z->value() + z_offset);

  updateCurrOriSpinbox(ui_.dSpinBox_marker_ori_r->value(), ui_.dSpinBox_marker_ori_p->value(),
                       ui_.dSpinBox_marker_ori_y->value());
}

void MainWindow::on_currpos_button_clicked(bool check)
{
  qnode_thor3_.getKinematicsPose(ui_.group_combobox->currentText().toStdString());
}

void MainWindow::on_despos_button_clicked(bool check)
{
  thormang3_manipulation_module_msgs::KinematicsPose msg;

  msg.name = ui_.group_combobox->currentText().toStdString();

  msg.pose.position.x = ui_.pos_x_spinbox->value();
  msg.pose.position.y = ui_.pos_y_spinbox->value();
  msg.pose.position.z = ui_.pos_z_spinbox->value();

  // deg -> rad
  double roll = deg2rad<double>(ui_.ori_roll_spinbox->value());
  double pitch = deg2rad<double>(ui_.ori_pitch_spinbox->value());
  double yaw = deg2rad<double>(ui_.ori_yaw_spinbox->value());

  Eigen::Quaterniond orientation = rpy2quaternion(roll, pitch, yaw);

  msg.pose.orientation.x = orientation.x();
  msg.pose.orientation.y = orientation.y();
  msg.pose.orientation.z = orientation.z();
  msg.pose.orientation.w = orientation.w();

  qnode_thor3_.sendIkMsg(msg);
}

void MainWindow::on_button_grip_on_clicked(bool check)
{
  setGripper(GRIPPER_ON_ANGLE, GRIPPER_TORQUE_LIMIT, ui_.gripper_comboBox->currentText().toStdString());
}

void MainWindow::on_button_grip_off_clicked(bool check)
{
  setGripper(GRIPPER_OFF_ANGLE, GRIPPER_TORQUE_LIMIT, ui_.gripper_comboBox->currentText().toStdString());
}

void MainWindow::on_A0_button_fl_clicked(bool check)
{
  sendWalkingCommand("turn left");
}

void MainWindow::on_A1_button_f_clicked(bool check)
{
  sendWalkingCommand("forward");
}

void MainWindow::on_A2_button_fr_clicked(bool check)
{
  sendWalkingCommand("turn right");
}

void MainWindow::on_B0_button_l_clicked(bool check)
{
  sendWalkingCommand("left");
}

void MainWindow::on_B1_button_stop_clicked(bool check)
{
  sendWalkingCommand("stop");
}

void MainWindow::on_B2_button_r_clicked(bool check)
{
  sendWalkingCommand("right");
}

void MainWindow::on_C0_button_bl_clicked(bool check)
{
}    // disable

void MainWindow::on_C1_button_b_clicked(bool check)
{
  sendWalkingCommand("backward");
}

void MainWindow::on_C2_button_br_clicked(bool check)
{
}    // disable

void MainWindow::on_button_balance_on_clicked(bool check)
{
  qnode_thor3_.setWalkingBalance(true);
}

void MainWindow::on_button_balance_off_clicked(bool check)
{
  qnode_thor3_.setWalkingBalance(false);
}

void MainWindow::on_button_feedback_gain_apply_clicked(bool check)
{
  qnode_thor3_.setFeedBackGain();
}

void MainWindow::on_A0_button_get_step_clicked(bool check)
{
  geometry_msgs::Pose target_pose;
  target_pose.position.x = ui_.dSpinBox_marker_pos_x->value();
  target_pose.position.y = ui_.dSpinBox_marker_pos_y->value();
  target_pose.position.z = ui_.dSpinBox_marker_pos_z->value();

  double roll = deg2rad<double>(ui_.dSpinBox_marker_ori_r->value());
  double pitch = deg2rad<double>(ui_.dSpinBox_marker_ori_p->value());
  double yaw = deg2rad<double>(ui_.dSpinBox_marker_ori_y->value());

  Eigen::Quaterniond orientation = rpy2quaternion(roll, pitch, yaw);

  target_pose.orientation.x = orientation.x();
  target_pose.orientation.y = orientation.y();
  target_pose.orientation.z = orientation.z();
  target_pose.orientation.w = orientation.w();

  // generate foot steps
  qnode_thor3_.makeFootstepUsingPlanner(target_pose);

  ui_.A1_button_clear_step->setEnabled(true);
  ui_.A2_button_go_walking->setEnabled(true);
}

void MainWindow::on_A1_button_clear_step_clicked(bool check)
{
  qnode_thor3_.clearFootsteps();

  ui_.A1_button_clear_step->setEnabled(false);
  ui_.A2_button_go_walking->setEnabled(false);
}

void MainWindow::on_A2_button_go_walking_clicked(bool check)
{
  qnode_thor3_.setWalkingFootsteps();

  ui_.A1_button_clear_step->setEnabled(false);
  ui_.A2_button_go_walking->setEnabled(false);
}

void MainWindow::on_head_center_button_clicked(bool check)
{
  qnode_thor3_.log(QNodeThor3::Info, "Go Head init position");
  setHeadJointsAngle(0.0, 0.0);
}

void MainWindow::on_dSpinBox_marker_pos_x_valueChanged(double value)
{
  updateInteractiveMarker();
}

void MainWindow::on_dSpinBox_marker_pos_y_valueChanged(double value)
{
  updateInteractiveMarker();
}

void MainWindow::on_dSpinBox_marker_pos_z_valueChanged(double value)
{
  updateInteractiveMarker();
}

void MainWindow::on_dSpinBox_marker_ori_r_valueChanged(double value)
{
  updateInteractiveMarker();
}

void MainWindow::on_dSpinBox_marker_ori_p_valueChanged(double value)
{
  updateInteractiveMarker();
}

void MainWindow::on_dSpinBox_marker_ori_y_valueChanged(double value)
{
  updateInteractiveMarker();
}

void MainWindow::on_button_marker_set_clicked()
{
  makeInteractiveMarker();
}

void MainWindow::on_button_marker_clear_clicked()
{
  clearMarkerPanel();
}

/////////////////////////////////////////////////
//             Manupulation Demo
/////////////////////////////////////////////////
void MainWindow::on_button_manipulation_demo_0_clicked(bool check)
{
  // init pose : base
  qnode_thor3_.moveInitPose();
}

void MainWindow::on_button_manipulation_demo_1_clicked(bool check)
{
  // manipulation mode
  qnode_thor3_.enableControlModule("manipulation_module");
  qnode_thor3_.enableControlModule("gripper_module");
}

void MainWindow::on_button_manipulation_demo_2_clicked(bool check)
{
  // manipulation init pose
  on_inipose_button_clicked(false);
}

void MainWindow::on_button_manipulation_demo_3_clicked(bool check)
{
  // head control mode
  qnode_thor3_.enableControlModule("head_control_module");

  // wait for setting the module
  usleep(10 * 1000);

  // scan
  qnode_thor3_.assembleLidar();
}

void MainWindow::on_button_manipulation_demo_4_clicked(bool check)
{
  // set interactive marker
  geometry_msgs::Pose current_pose;
  getPoseFromMarkerPanel(current_pose);

  // set default value
  if (current_pose.position.x == 0 && current_pose.position.y == 0 && current_pose.position.z == 0)
  {
    current_pose.position.x = 0.305;
    current_pose.position.y = (ui_.comboBox_arm_group->currentText().toStdString() == "Right Arm") ? -0.3 : 0.3;
    current_pose.position.z = 0.108;

    updatePosePanel(current_pose);
  }

  qnode_thor3_.makeInteractiveMarker(current_pose);
}

void MainWindow::on_button_manipulation_demo_5_clicked(bool check)
{
  // send pose
  thormang3_manipulation_module_msgs::KinematicsPose msg;
  double z_offset = 0.723;

  // arm group : left_arm_with_torso / right_arm_with_torso
  std::string selected_arm = ui_.comboBox_arm_group->currentText().toStdString();
  std::string arm_group = (selected_arm == "Right Arm") ? "right_arm_with_torso" : "left_arm_with_torso";
  msg.name = arm_group;

  msg.pose.position.x = ui_.dSpinBox_marker_pos_x->value() + ui_.dSpinBox_offset_x->value();
  msg.pose.position.y = ui_.dSpinBox_marker_pos_y->value() + ui_.dSpinBox_offset_y->value();
  msg.pose.position.z = ui_.dSpinBox_marker_pos_z->value() + ui_.dSpinBox_offset_z->value() + z_offset;

  double roll = deg2rad<double>(ui_.dSpinBox_marker_ori_r->value());
  double pitch = deg2rad<double>(ui_.dSpinBox_marker_ori_p->value());
  double yaw = deg2rad<double>(ui_.dSpinBox_marker_ori_y->value());

  Eigen::Quaterniond orientation = rpy2quaternion(roll, pitch, yaw);

  msg.pose.orientation.x = orientation.x();
  msg.pose.orientation.y = orientation.y();
  msg.pose.orientation.z = orientation.z();
  msg.pose.orientation.w = orientation.w();

  qnode_thor3_.sendIkMsg(msg);

  // clear marker and foot steps
  qnode_thor3_.clearInteractiveMarker();
  qnode_thor3_.clearFootsteps();
}

void MainWindow::on_button_manipulation_demo_6_clicked(bool check)
{
  // grip on : l_arm_grip / r_arm_grip
  std::string arm_group =
      (ui_.comboBox_arm_group->currentText().toStdString() == "Right Arm") ? "r_arm_grip" : "l_arm_grip";
  setGripper(GRIPPER_ON_ANGLE, GRIPPER_TORQUE_LIMIT, arm_group);
}

void MainWindow::on_button_manipulation_demo_7_clicked(bool check)
{
  // grip off : l_arm_grip / r_arm_grip
  std::string arm_group =
      (ui_.comboBox_arm_group->currentText().toStdString() == "Right Arm") ? "r_arm_grip" : "l_arm_grip";
  setGripper(GRIPPER_OFF_ANGLE, GRIPPER_TORQUE_LIMIT, arm_group);
}

/////////////////////////////////////////////////
//                 Walking Demo
/////////////////////////////////////////////////
void MainWindow::on_button_walking_demo_0_clicked(bool check)
{
  // init pose : base
  qnode_thor3_.moveInitPose();
}

void MainWindow::on_button_walking_demo_1_clicked(bool check)
{
  // head control mode
  qnode_thor3_.enableControlModule("head_control_module");

  // wait for setting the module
  usleep(10 * 1000);

  // scan
  qnode_thor3_.assembleLidar();
}

void MainWindow::on_button_walking_demo_2_clicked(bool check)
{
  // walking mode
  qnode_thor3_.enableControlModule("walking_module");

  // wait for setting module
  usleep(10 * 1000);

  // balance on
  qnode_thor3_.setWalkingBalance(true);
}

void MainWindow::on_button_walking_demo_3_clicked(bool check)
{
  // set interactive marker
  makeInteractiveMarker();
}

void MainWindow::on_button_walking_demo_4_clicked(bool check)
{
  double y_offset = (ui_.comboBox_kick_foot->currentText().toStdString() == "Right Foot") ? 0.093 : -0.093;

  geometry_msgs::Pose target_pose;
  target_pose.position.x = ui_.dSpinBox_marker_pos_x->value();
  target_pose.position.y = ui_.dSpinBox_marker_pos_y->value() + y_offset;
  target_pose.position.z = ui_.dSpinBox_marker_pos_z->value();

  double roll = deg2rad<double>(ui_.dSpinBox_marker_ori_r->value());
  double pitch = deg2rad<double>(ui_.dSpinBox_marker_ori_p->value());
  double yaw = deg2rad<double>(ui_.dSpinBox_marker_ori_y->value());

  Eigen::Quaterniond orientation = rpy2quaternion(roll, pitch, yaw);

  target_pose.orientation.x = orientation.x();
  target_pose.orientation.y = orientation.y();
  target_pose.orientation.z = orientation.z();
  target_pose.orientation.w = orientation.w();

  // generate foot steps
  qnode_thor3_.makeFootstepUsingPlanner(target_pose);
}

void MainWindow::on_button_walking_demo_5_clicked(bool check)
{
  // start walking
  qnode_thor3_.setWalkingFootsteps();

  // clear marker and foot steps
  qnode_thor3_.clearInteractiveMarker();
  qnode_thor3_.clearFootsteps();
}

void MainWindow::on_button_walking_demo_6_clicked(bool check)
{
  // head control mode
  qnode_thor3_.enableControlModule("head_control_module");

  // wait for setting the module
  usleep(10 * 1000);

  // scan
  qnode_thor3_.assembleLidar();
}

void MainWindow::on_button_walking_demo_7_clicked(bool check)
{
  // foot : right kick / left kick
  std::string kick_command =
      (ui_.comboBox_kick_foot->currentText().toStdString() == "Right Foot") ? "right kick" : "left kick";
  qnode_thor3_.kickDemo(kick_command);
}

/////////////////////////////////////////////////
//                Action Demo
/////////////////////////////////////////////////
void MainWindow::on_button_motion_demo_0_clicked(bool check)
{
  // init pose : base
  qnode_thor3_.moveInitPose();
}

void MainWindow::on_button_motion_demo_1_clicked(bool check)
{
  // action mode
  qnode_thor3_.enableControlModule("action_module");
}

void MainWindow::on_button_overload_count_reset_clicked(bool check)
{
  qnode_thor3_.publishAlarmCommand("reset");
  clearOverload();
}

/*****************************************************************************
 ** Implemenation [Slots][manually connected]
 *****************************************************************************/

void MainWindow::updateLoggingView()
{
  ui_.view_logging->scrollToBottom();
}

// user shortcut
void MainWindow::setUserShortcut()
{
  // Setup a signal mapper to avoid creating custom slots for each tab
  QSignalMapper *sig_map = new QSignalMapper(this);

  // Setup the shortcut for the first tab : Mode
  QShortcut *short_tab1 = new QShortcut(QKeySequence("F1"), this);
  connect(short_tab1, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab1, 0);

  // Setup the shortcut for the second tab : Manipulation
  QShortcut *short_tab2 = new QShortcut(QKeySequence("F2"), this);
  connect(short_tab2, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab2, 1);

  // Setup the shortcut for the third tab : Walking
  QShortcut *short_tab3 = new QShortcut(QKeySequence("F3"), this);
  connect(short_tab3, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab3, 2);

  // Setup the shortcut for the fouth tab : Head control
  QShortcut *short_tab4 = new QShortcut(QKeySequence("F4"), this);
  connect(short_tab4, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab4, 3);

  // Setup the shortcut for the fifth tab : Motion
  QShortcut *short_tab5 = new QShortcut(QKeySequence("F5"), this);
  connect(short_tab5, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab5, 4);

  // Demo tab
  QShortcut *short_tab6 = new QShortcut(QKeySequence("F6"), this);
  connect(short_tab6, SIGNAL(activated()), sig_map, SLOT(map()));
  sig_map->setMapping(short_tab6, 5);

  // Wire the signal mapper to the tab widget index change slot
  connect(sig_map, SIGNAL(mapped(int)), ui_.tabWidget_control, SLOT(setCurrentIndex(int)));
}

void MainWindow::updatePresentJointModule(std::vector<int> mode)
{
  QList<QComboBox *> combo_children = ui_.widget_mode->findChildren<QComboBox *>();

  for (int ix = 0; ix < combo_children.length(); ix++)
  {
    int control_index = mode.at(ix);
    combo_children.at(ix)->setCurrentIndex(control_index);

    if (debug_print_)
    {
      std::stringstream log_stream;
      std::string joint_name;
      int id;

      std::string control_mode = combo_children.at(ix)->currentText().toStdString();

      bool result = qnode_thor3_.getIDJointNameFromIndex(ix, id, joint_name);
      if (result == true)
        log_stream << "[" << (id < 10 ? "0" : "") << id << "] " << joint_name << " : " << control_mode;
      else
        log_stream << "id " << ix << " : " << control_mode;

      qnode_thor3_.log(QNodeThor3::Info, log_stream.str());
    }
  }

  // set module UI
  updateModuleUI();
}

void MainWindow::updateModuleUI()
{
  if (debug_print_)
    return;

  for (int index = 0; index < qnode_thor3_.getModuleTableSize(); index++)
  {
    std::string mode = qnode_thor3_.getModuleName(index);
    if (mode == "")
      continue;

    std::map<std::string, QList<QWidget *> >::iterator module_iter = module_ui_table_.find(mode);
    if (module_iter == module_ui_table_.end())
      continue;

    QList<QWidget *> list = module_iter->second;
    for (int ix = 0; ix < list.size(); ix++)
    {
      bool is_enable = qnode_thor3_.isUsingModule(mode);
      list.at(ix)->setEnabled(is_enable);
    }
  }
}

// head control
void MainWindow::updateHeadJointsAngle(double pan, double tilt)
{
  if (ui_.head_pan_slider->underMouse() == true)
    return;
  if (ui_.head_pan_spinbox->underMouse() == true)
    return;
  if (ui_.head_tilt_slider->underMouse() == true)
    return;
  if (ui_.head_tilt_spinbox->underMouse() == true)
    return;

  is_updating_ = true;

  ui_.head_pan_slider->setValue(rad2deg<double>(pan));
  ui_.head_tilt_slider->setValue(rad2deg<double>(tilt));

  is_updating_ = false;
}

void MainWindow::setHeadJointsAngle()
{
  if (is_updating_ == true)
    return;
  qnode_thor3_.setHeadJoint(deg2rad<double>(ui_.head_pan_slider->value()),
                            deg2rad<double>(ui_.head_tilt_slider->value()));
}

void MainWindow::setHeadJointsAngle(double pan, double tilt)
{
  qnode_thor3_.setHeadJoint(deg2rad<double>(pan), deg2rad<double>(tilt));
}

void MainWindow::playMotion(int motion_index)
{
  bool to_action_script = ui_.checkBox_action_script->isChecked();

  qnode_thor3_.playMotion(motion_index, to_action_script);
}

// manipulation
void MainWindow::updateCurrJointSpinbox(double value)
{
  ui_.joint_spinbox->setValue(rad2deg<double>(value));
}

void MainWindow::updateCurrPosSpinbox(double x, double y, double z)
{
  ui_.pos_x_spinbox->setValue(x);
  ui_.pos_y_spinbox->setValue(y);
  ui_.pos_z_spinbox->setValue(z);
}

void MainWindow::updateCurrOriSpinbox(double x, double y, double z, double w)
{
  Eigen::Quaterniond orientation(w, x, y, z);
  Eigen::Vector3d euler = rad2deg<Eigen::Vector3d>(quaternion2rpy(orientation));

  ui_.ori_roll_spinbox->setValue(euler[0]);
  ui_.ori_pitch_spinbox->setValue(euler[1]);
  ui_.ori_yaw_spinbox->setValue(euler[2]);
}

void MainWindow::updateCurrOriSpinbox(double r, double p, double y)
{
  ui_.ori_roll_spinbox->setValue(r);
  ui_.ori_pitch_spinbox->setValue(p);
  ui_.ori_yaw_spinbox->setValue(y);
}

void MainWindow::setGripper(const double angle_deg, const double torque_limit, const std::string &arm_type)
{
  sensor_msgs::JointState gripper_joint;
  gripper_joint.name.push_back(arm_type);
  gripper_joint.position.push_back(deg2rad<double>(angle_deg));
  gripper_joint.effort.push_back(torque_limit);

  qnode_thor3_.sendGripperPosition(gripper_joint);
}

// walking
void MainWindow::sendWalkingCommand(const std::string &command)
{
  thormang3_foot_step_generator::FootStepCommand msg;

  msg.command = command;
  msg.step_num = ui_.A1_spinbox_step_num->value();
  msg.step_time = ui_.AB1_spinbox_step_time->value();
  msg.step_length = ui_.B1_spinbox_f_step_length->value();
  msg.side_step_length = ui_.C1_spinbox_s_step_length->value();
  msg.step_angle_rad = deg2rad<double>(ui_.D1_spinbox_r_angle->value());

  qnode_thor3_.setWalkingCommand(msg);
}

// Update UI - position
void MainWindow::updatePointPanel(const geometry_msgs::Point point)
{
  is_updating_ = true;

  setPointToMarkerPanel(point);

  ROS_INFO("Update Position Panel");
  is_updating_ = false;
}

// Update UI - pose
void MainWindow::updatePosePanel(const geometry_msgs::Pose pose)
{
  is_updating_ = true;

  setPoseToMarkerPanel(pose);

  ROS_INFO("Update Pose Panel");
  is_updating_ = false;
}

void MainWindow::getPoseFromMarkerPanel(geometry_msgs::Pose &current)
{
  // position
  current.position.x = ui_.dSpinBox_marker_pos_x->value();
  current.position.y = ui_.dSpinBox_marker_pos_y->value();
  current.position.z = ui_.dSpinBox_marker_pos_z->value();

  // orientation
  Eigen::Vector3d euler(ui_.dSpinBox_marker_ori_r->value(), ui_.dSpinBox_marker_ori_p->value(),
                        ui_.dSpinBox_marker_ori_y->value());
  Eigen::Quaterniond orientation = rpy2quaternion(deg2rad<Eigen::Vector3d>(euler));

  tf::quaternionEigenToMsg(orientation, current.orientation);
}

void MainWindow::setPoseToMarkerPanel(const geometry_msgs::Pose &current)
{
  // position
  ui_.dSpinBox_marker_pos_x->setValue(current.position.x);
  ui_.dSpinBox_marker_pos_y->setValue(current.position.y);
  ui_.dSpinBox_marker_pos_z->setValue(current.position.z);

  // orientation
  Eigen::Vector3d euler = rad2deg<Eigen::Vector3d>(quaternion2rpy(current.orientation));

  ui_.dSpinBox_marker_ori_r->setValue(euler[0]);
  ui_.dSpinBox_marker_ori_p->setValue(euler[1]);
  ui_.dSpinBox_marker_ori_y->setValue(euler[2]);
}

void MainWindow::getPointFromMarkerPanel(geometry_msgs::Point &current)
{
  // position
  current.x = ui_.dSpinBox_marker_pos_x->value();
  current.y = ui_.dSpinBox_marker_pos_y->value();
  current.z = ui_.dSpinBox_marker_pos_z->value();
}

void MainWindow::setPointToMarkerPanel(const geometry_msgs::Point &current)
{
  // position
  ui_.dSpinBox_marker_pos_x->setValue(current.x);
  ui_.dSpinBox_marker_pos_y->setValue(current.y);
  ui_.dSpinBox_marker_pos_z->setValue(current.z);

  // orientation
  ui_.dSpinBox_marker_ori_r->setValue(0.0);
  ui_.dSpinBox_marker_ori_p->setValue(0.0);
  ui_.dSpinBox_marker_ori_y->setValue(0.0);
}

// make interactive marker
void MainWindow::makeInteractiveMarker()
{
  geometry_msgs::Pose current_pose;
  getPoseFromMarkerPanel(current_pose);

  qnode_thor3_.makeInteractiveMarker(current_pose);
}

// update interactive marker pose from ui
void MainWindow::updateInteractiveMarker()
{
  if (is_updating_ == true)
    return;

  geometry_msgs::Pose current_pose;
  getPoseFromMarkerPanel(current_pose);

  qnode_thor3_.updateInteractiveMarker(current_pose);
}

void MainWindow::clearMarkerPanel()
{
  geometry_msgs::Pose init_pose;
  updatePosePanel(init_pose);

  ROS_INFO("Clear Panel");

  qnode_thor3_.clearInteractiveMarker();
}

// overload status
void MainWindow::updateOverloadStatus(int side, int overload_status, int warning_count, int error_count)
{
  if(side == QNodeThor3::Right)
  {
    switch(overload_status)
    {
    case thormang3_alarm_module_msgs::JointOverloadStatus::NORMAL:
      ui_.label_overload_right->setStyleSheet(sytlesheet_overload_normal);
      break;

    case thormang3_alarm_module_msgs::JointOverloadStatus::WARNING:
      ui_.label_overload_right->setStyleSheet(sytlesheet_overload_warning);
      break;

    case thormang3_alarm_module_msgs::JointOverloadStatus::ERROR:
      ui_.label_overload_right->setStyleSheet(sytlesheet_overload_error);
      break;

    case thormang3_alarm_module_msgs::JointOverloadStatus::EMERGENCY:
      ui_.label_overload_right->setStyleSheet(sytlesheet_overload_error);
      break;

    default:
      break;
    }

    if(warning_count <= 0)
    {
      std::string msg = "Warning Count : 0";
      ui_.label_overload_right_warning->setText(tr(msg.c_str()));
      ui_.label_overload_right_warning->setStyleSheet(sytlesheet_overload_none);
    }
    else
    {
      std::stringstream msg_ss;
      msg_ss << "Warning Count : " << warning_count;
      ui_.label_overload_right_warning->setText(tr(msg_ss.str().c_str()));
      ui_.label_overload_right_warning->setStyleSheet(sytlesheet_overload_warning);
    }

    if(error_count <= 0)
    {
      std::string msg = "Warning Count : 0";
      ui_.label_overload_right_error->setText(tr(msg.c_str()));
      ui_.label_overload_right_error->setStyleSheet(sytlesheet_overload_none);
    }
    else
    {
      std::stringstream msg_ss;
      msg_ss << "Error Count : " << error_count;
      ui_.label_overload_right_error->setText(tr(msg_ss.str().c_str()));
      ui_.label_overload_right_error->setStyleSheet(sytlesheet_overload_error);
    }
  }
  else if(side == QNodeThor3::Left)
  {
    switch(overload_status)
    {
    case thormang3_alarm_module_msgs::JointOverloadStatus::NORMAL:
      ui_.label_overload_left->setStyleSheet(sytlesheet_overload_normal);
      break;

    case thormang3_alarm_module_msgs::JointOverloadStatus::WARNING:
      ui_.label_overload_left->setStyleSheet(sytlesheet_overload_warning);
      break;

    case thormang3_alarm_module_msgs::JointOverloadStatus::ERROR:
      ui_.label_overload_left->setStyleSheet(sytlesheet_overload_error);
      break;

    case thormang3_alarm_module_msgs::JointOverloadStatus::EMERGENCY:
      ui_.label_overload_left->setStyleSheet(sytlesheet_overload_error);
      break;

    default:
      break;
    }

    if(warning_count <= 0)
    {
      std::string msg = "Warning Count : 0";
      ui_.label_overload_left_warning->setText(tr(msg.c_str()));
      ui_.label_overload_left_warning->setStyleSheet(sytlesheet_overload_none);
    }
    else
    {
      std::stringstream msg_ss;
      msg_ss << "Warning Count : " << warning_count;
      ui_.label_overload_left_warning->setText(tr(msg_ss.str().c_str()));
      ui_.label_overload_left_warning->setStyleSheet(sytlesheet_overload_warning);
    }

    if(error_count <= 0)
    {
      std::string msg = "Warning Count : 0";
      ui_.label_overload_left_error->setText(tr(msg.c_str()));
      ui_.label_overload_left_error->setStyleSheet(sytlesheet_overload_none);
    }
    else
    {
      std::stringstream msg_ss;
      msg_ss << "Error Count : " << error_count;
      ui_.label_overload_left_error->setText(tr(msg_ss.str().c_str()));
      ui_.label_overload_left_error->setStyleSheet(sytlesheet_overload_error);
    }
  }
}

void MainWindow::clearOverload()
{
  updateOverloadStatus(QNodeThor3::Right, thormang3_alarm_module_msgs::JointOverloadStatus::NORMAL, 0, 0);
  updateOverloadStatus(QNodeThor3::Left, thormang3_alarm_module_msgs::JointOverloadStatus::NORMAL, 0, 0);
}

/*****************************************************************************
 ** Implementation [Menu]
 *****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this, tr("About ..."), tr("<h2>THORMANG3 Demo</h2><p>Copyright Robotis</p>"));
}

/*****************************************************************************
 ** Implementation [Configuration]
 *****************************************************************************/

void MainWindow::initModeUnit()
{
  int number_joint = qnode_thor3_.getJointTableSize();

  // preset module button
  QHBoxLayout *preset_layout = new QHBoxLayout;
  QSignalMapper *signalMapper = new QSignalMapper(this);

  // parse yaml : preset modules
  for (std::map<int, std::string>::iterator iter = qnode_thor3_.module_table_.begin();
       iter != qnode_thor3_.module_table_.end(); ++iter)
  {
    std::string preset_name = iter->second;
    QPushButton *preset_button = new QPushButton(tr(preset_name.c_str()));
    if (debug_print_)
      std::cout << "name : " << preset_name << std::endl;

    preset_layout->addWidget(preset_button);

    signalMapper->setMapping(preset_button, preset_button->text());
    QObject::connect(preset_button, SIGNAL(clicked()), signalMapper, SLOT(map()));
  }

  QObject::connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(enableModule(QString)));

  ui_.widget_mode_preset->setLayout(preset_layout);

  // joints
  QGridLayout *grid_mod = new QGridLayout;
  for (int ix = 0; ix < number_joint; ix++)
  {
    std::stringstream stream;
    std::string joint;
    int id;

    bool result = false;
    result = qnode_thor3_.getIDJointNameFromIndex(ix, id, joint);

    if (result == false)
      continue;

    stream << "[" << (id < 10 ? "0" : "") << id << "] " << joint;
    QLabel *label = new QLabel(tr(stream.str().c_str()));

    QStringList list;
    for (int index = 0; index < qnode_thor3_.getModuleTableSize(); index++)
    {
      std::string mode = qnode_thor3_.getModuleName(index);
      if (mode != "")
        list << mode.c_str();
    }

    QComboBox *combo = new QComboBox();
    combo->setObjectName(tr(joint.c_str()));
    combo->addItems(list);
    combo->setEnabled(false);      // not changable
    int row = ix / 2 + 1;
    int col = (ix % 2) * 3;
    grid_mod->addWidget(label, row, col, 1, 1);
    grid_mod->addWidget(combo, row, col + 1, 1, 2);
  }

  // get buttons
  QPushButton *get_mode_button = new QPushButton(tr("Get Mode"));
  grid_mod->addWidget(get_mode_button, (number_joint / 2) + 2, 0, 1, 3);
  QObject::connect(get_mode_button, SIGNAL(clicked(bool)), &qnode_thor3_, SLOT(getJointControlModule()));

  ui_.widget_mode->setLayout(grid_mod);

  // make module widget table
  for (int index = 0; index < qnode_thor3_.getModuleTableSize(); index++)
  {
    std::string mode = qnode_thor3_.getModuleName(index);
    if (mode == "")
      continue;
    std::string mode_reg = "*" + mode;

    QRegExp rx(QRegExp(tr(mode_reg.c_str())));
    rx.setPatternSyntax(QRegExp::Wildcard);

    QList<QWidget *> list = ui_.centralwidget->findChildren<QWidget *>(rx);
    module_ui_table_[mode] = list;

    if (debug_print_)
      std::cout << "Module widget : " << mode << " [" << list.size() << "]" << std::endl;
  }

  // make motion tab
  if (qnode_thor3_.getModuleIndex("action_module") != -1)
    initMotionUnit();
}

void MainWindow::initMotionUnit()
{
  // preset button
  QGridLayout *motion_layout = new QGridLayout;
  QGridLayout *demo_motion_layout = new QGridLayout;
  QSignalMapper *signalMapper = new QSignalMapper(this);
  QSignalMapper *demo_signalMapper = new QSignalMapper(this);

  // yaml preset
  int index = 0;
  for (std::map<int, std::string>::iterator iter = qnode_thor3_.motion_table_.begin();
       iter != qnode_thor3_.motion_table_.end(); ++iter)
  {
    int motion_index = iter->first;
    std::string motion_name = iter->second;
    QString q_motion_name = QString::fromStdString(motion_name);
    QPushButton *motion_button = new QPushButton(q_motion_name);
    QPushButton *demo_motion_button = new QPushButton(q_motion_name);

    int size = (motion_index < 0) ? 2 : 1;
    int row = index / 4;
    int col = index % 4;
    motion_layout->addWidget(motion_button, row, col, 1, size);
    demo_motion_layout->addWidget(demo_motion_button, row, col, 1, size);

    signalMapper->setMapping(motion_button, motion_index);
    QObject::connect(motion_button, SIGNAL(clicked()), signalMapper, SLOT(map()));
    demo_signalMapper->setMapping(demo_motion_button, motion_index);
    QObject::connect(demo_motion_button, SIGNAL(clicked()), demo_signalMapper, SLOT(map()));

    index += size;
  }

  int row = index / 4;
  row = (index % 4 == 0) ? row : row + 1;
  QSpacerItem *verticalSpacer = new QSpacerItem(20, 400, QSizePolicy::Minimum, QSizePolicy::Expanding);
  motion_layout->addItem(verticalSpacer, row, 0, 1, 4);
  QSpacerItem *demo_verticalSpacer = new QSpacerItem(20, 400, QSizePolicy::Minimum, QSizePolicy::Expanding);
  demo_motion_layout->addItem(demo_verticalSpacer, row, 0, 1, 4);

  QObject::connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(playMotion(int)));
  QObject::connect(demo_signalMapper, SIGNAL(mapped(int)), this, SLOT(playMotion(int)));

  ui_.scroll_widget_motion->setLayout(motion_layout);
  ui_.scroll_widget_demo_motion->setLayout(demo_motion_layout);
}

void MainWindow::enableModule(QString mode_name)
{
  qnode_thor3_.enableControlModule(mode_name.toStdString());
}

void MainWindow::readSettings()
{
  QSettings settings("Qt-Ros Package", "thormang3_demo");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::writeSettings()
{
  QSettings settings("Qt-Ros Package", "thormang3_demo");
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  writeSettings();
  QMainWindow::closeEvent(event);
}

/*****************************************************************************
 ** Implementation [Util]
 *****************************************************************************/
// math : euler & quaternion & rotation mat
Eigen::Vector3d MainWindow::rotation2rpy(const Eigen::MatrixXd &rotation)
{
  Eigen::Vector3d rpy;

  rpy[0] = atan2(rotation.coeff(2, 1), rotation.coeff(2, 2));
  rpy[1] = atan2(-rotation.coeff(2, 0), sqrt(pow(rotation.coeff(2, 1), 2) + pow(rotation.coeff(2, 2), 2)));
  rpy[2] = atan2(rotation.coeff(1, 0), rotation.coeff(0, 0));

  return rpy;
}

Eigen::MatrixXd MainWindow::rpy2rotation(const double &roll, const double &pitch, const double &yaw)
{
  Eigen::MatrixXd rotation = rotationZ(yaw) * rotationY(pitch) * rotationX(roll);

  return rotation;
}

Eigen::Quaterniond MainWindow::rpy2quaternion(const Eigen::Vector3d &euler)
{
  return rpy2quaternion(euler[0], euler[1], euler[2]);
}

Eigen::Quaterniond MainWindow::rpy2quaternion(const double &roll, const double &pitch, const double &yaw)
{
  Eigen::MatrixXd rotation = rpy2rotation(roll, pitch, yaw);

  Eigen::Matrix3d rotation3d;
  rotation3d = rotation.block(0, 0, 3, 3);

  Eigen::Quaterniond quaternion;

  quaternion = rotation3d;

  return quaternion;
}

Eigen::Quaterniond MainWindow::rotation2quaternion(const Eigen::MatrixXd &rotation)
{
  Eigen::Matrix3d rotation3d;

  rotation3d = rotation.block(0, 0, 3, 3);

  Eigen::Quaterniond quaternion;
  quaternion = rotation3d;

  return quaternion;
}

Eigen::Vector3d MainWindow::quaternion2rpy(const Eigen::Quaterniond &quaternion)
{
  Eigen::Vector3d rpy = rotation2rpy(quaternion.toRotationMatrix());

  return rpy;
}

Eigen::Vector3d MainWindow::quaternion2rpy(const geometry_msgs::Quaternion &quaternion)
{
  Eigen::Quaterniond eigen_quaternion;
  tf::quaternionMsgToEigen(quaternion, eigen_quaternion);

  Eigen::Vector3d rpy = rotation2rpy(eigen_quaternion.toRotationMatrix());

  return rpy;
}

Eigen::MatrixXd MainWindow::quaternion2rotation(const Eigen::Quaterniond &quaternion)
{
  Eigen::MatrixXd rotation = quaternion.toRotationMatrix();

  return rotation;
}

Eigen::MatrixXd MainWindow::rotationX(const double &angle)
{
  Eigen::MatrixXd rotation(3, 3);

  rotation << 1.0, 0.0, 0.0, 0.0, cos(angle), -sin(angle), 0.0, sin(angle), cos(angle);

  return rotation;
}

Eigen::MatrixXd MainWindow::rotationY(const double &angle)
{
  Eigen::MatrixXd rotation(3, 3);

  rotation << cos(angle), 0.0, sin(angle), 0.0, 1.0, 0.0, -sin(angle), 0.0, cos(angle);

  return rotation;
}

Eigen::MatrixXd MainWindow::rotationZ(const double &angle)
{
  Eigen::MatrixXd rotation(3, 3);

  rotation << cos(angle), -sin(angle), 0.0, sin(angle), cos(angle), 0.0, 0.0, 0.0, 1.0;

  return rotation;
}

}  // namespace thormang3_demo
