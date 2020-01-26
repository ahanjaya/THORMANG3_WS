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

#ifndef thormang3_demo_MAIN_WINDOW_H
#define thormang3_demo_MAIN_WINDOW_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace thormang3_demo
{

/*****************************************************************************
 ** Interface [MainWindow]
 *****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
Q_OBJECT

 public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void readSettings();  // Load up qt program settings at startup
  void writeSettings();  // Save qt program settings when closing

  void closeEvent(QCloseEvent *event);  // Overloaded function

 public Q_SLOTS:
  /******************************************
   ** Auto-connections (connectSlotsByName())
   *******************************************/
  void on_actionAbout_triggered();
  void on_button_assemble_lidar_clicked(bool check);
  void on_button_clear_log_clicked(bool check);

  void on_button_init_pose_clicked(bool check);
  void on_button_ft_air_clicked(bool check);
  void on_button_ft_gnd_clicked(bool check);
  void on_button_ft_calc_clicked(bool check);
  void on_button_ft_save_clicked(bool check);

  void on_tabWidget_control_currentChanged(int index);

  // Manipulation
  void on_inipose_button_clicked(bool check);
  void on_currjoint_button_clicked(bool check);
  void on_desjoint_button_clicked(bool check);
  void on_get_despos_button_clicked(bool check);
  void on_currpos_button_clicked(bool check);
  void on_despos_button_clicked(bool check);
  void on_button_grip_on_clicked(bool check);
  void on_button_grip_off_clicked(bool check);

  // Walking
  void on_A0_button_fl_clicked(bool check);
  void on_A1_button_f_clicked(bool check);
  void on_A2_button_fr_clicked(bool check);

  void on_B0_button_l_clicked(bool check);
  void on_B1_button_stop_clicked(bool check);
  void on_B2_button_r_clicked(bool check);

  void on_C0_button_bl_clicked(bool check);
  void on_C1_button_b_clicked(bool check);
  void on_C2_button_br_clicked(bool check);

  void on_button_balance_on_clicked(bool check);
  void on_button_balance_off_clicked(bool check);
  void on_button_feedback_gain_apply_clicked(bool check);

  void on_A0_button_get_step_clicked(bool check);
  void on_A1_button_clear_step_clicked(bool check);
  void on_A2_button_go_walking_clicked(bool check);

  // Head Control
  void on_head_center_button_clicked(bool check);

  // Interactive Marker
  void on_dSpinBox_marker_pos_x_valueChanged(double value);
  void on_dSpinBox_marker_pos_y_valueChanged(double value);
  void on_dSpinBox_marker_pos_z_valueChanged(double value);

  void on_dSpinBox_marker_ori_r_valueChanged(double value);
  void on_dSpinBox_marker_ori_p_valueChanged(double value);
  void on_dSpinBox_marker_ori_y_valueChanged(double value);

  void on_button_marker_set_clicked();
  void on_button_marker_clear_clicked();

  // Demo
  void on_button_manipulation_demo_0_clicked(bool check);
  void on_button_manipulation_demo_1_clicked(bool check);
  void on_button_manipulation_demo_2_clicked(bool check);
  void on_button_manipulation_demo_3_clicked(bool check);
  void on_button_manipulation_demo_4_clicked(bool check);
  void on_button_manipulation_demo_5_clicked(bool check);
  void on_button_manipulation_demo_6_clicked(bool check);
  void on_button_manipulation_demo_7_clicked(bool check);

  void on_button_walking_demo_0_clicked(bool check);
  void on_button_walking_demo_1_clicked(bool check);
  void on_button_walking_demo_2_clicked(bool check);
  void on_button_walking_demo_3_clicked(bool check);
  void on_button_walking_demo_4_clicked(bool check);
  void on_button_walking_demo_5_clicked(bool check);
  void on_button_walking_demo_6_clicked(bool check);
  void on_button_walking_demo_7_clicked(bool check);

  void on_button_motion_demo_0_clicked(bool check);
  void on_button_motion_demo_1_clicked(bool check);

  void on_button_overload_count_reset_clicked(bool check);

  /******************************************
   ** Manual connections
   *******************************************/
  void updateLoggingView();  // no idea why this can't connect automatically
  void updatePresentJointModule(std::vector<int> mode);
  void enableModule(QString mode_name);
  void updateHeadJointsAngle(double pan, double tilt);

  // Manipulation
  void updateCurrJointSpinbox(double value);
  void updateCurrPosSpinbox(double x, double y, double z);
  void updateCurrOriSpinbox(double x, double y, double z, double w);
  void updateCurrOriSpinbox(double r, double p, double y);

  // demo
  void updatePointPanel(const geometry_msgs::Point point);
  void updatePosePanel(const geometry_msgs::Pose pose);

  // overload
  void updateOverloadStatus(int side, int overload_status, int warning_count, int error_count);

 protected Q_SLOTS:
  void setHeadJointsAngle();
  void playMotion(int motion_index);

 private:
  static const double GRIPPER_ON_ANGLE = 60;
  static const double GRIPPER_OFF_ANGLE = 5;
  static const double GRIPPER_TORQUE_LIMIT = 250;

  QString sytlesheet_overload_normal = QString("background-color: rgb(78, 154, 6); color: rgb(46, 52, 54);");
  QString sytlesheet_overload_warning = QString("background-color: rgb(196, 160, 0); color: rgb(85, 87, 83);");
  QString sytlesheet_overload_error = QString("background-color: rgb(204, 0, 0); color: rgb(211, 215, 207);");
  QString sytlesheet_overload_none = QString("background-color: rgba(10, 10, 10, 10); color: rgba(0, 0, 0, 0);");

  void setUserShortcut();
  void initModeUnit();
  void initMotionUnit();
  void updateModuleUI();
  void setHeadJointsAngle(double pan, double tilt);
  void sendWalkingCommand(const std::string &command);
  void setGripper(const double angle_deg, const double torque_limit, const std::string &arm_type);

  void makeInteractiveMarker();
  void updateInteractiveMarker();
  void clearMarkerPanel();
  void getPoseFromMarkerPanel(geometry_msgs::Pose &current);
  void setPoseToMarkerPanel(const geometry_msgs::Pose &current);
  void getPointFromMarkerPanel(geometry_msgs::Point &current);
  void setPointToMarkerPanel(const geometry_msgs::Point &current);

  void clearOverload();

  /******************************************
   ** Transformation
   *******************************************/
  Eigen::Vector3d rotation2rpy(const Eigen::MatrixXd &rotation);
  Eigen::MatrixXd rpy2rotation(const double &roll, const double &pitch, const double &yaw);
  Eigen::Quaterniond rpy2quaternion(const Eigen::Vector3d &euler);
  Eigen::Quaterniond rpy2quaternion(const double &roll, const double &pitch, const double &yaw);
  Eigen::Quaterniond rotation2quaternion(const Eigen::MatrixXd &rotation);
  Eigen::Vector3d quaternion2rpy(const Eigen::Quaterniond &quaternion);
  Eigen::Vector3d quaternion2rpy(const geometry_msgs::Quaternion &quaternion);
  Eigen::MatrixXd quaternion2rotation(const Eigen::Quaterniond &quaternion);
  Eigen::MatrixXd rotationX(const double &angle);
  Eigen::MatrixXd rotationY(const double &angle);
  Eigen::MatrixXd rotationZ(const double &angle);

  Ui::MainWindowDesign ui_;
  QNodeThor3 qnode_thor3_;
  bool debug_print_;
  bool demo_mode_;
  bool is_updating_;
  std::map<std::string, QList<QWidget *> > module_ui_table_;
};

template<typename T>
T deg2rad(T deg)
{
  return deg * M_PI / 180;
}

template<typename T>
T rad2deg(T rad)
{
  return rad * 180 / M_PI;
}

}  // namespace thormang3_demo

#endif // thormang3_demo_MAIN_WINDOW_H
