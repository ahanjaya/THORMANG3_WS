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
 ** Ifdefs
 *****************************************************************************/

#ifndef thormang3_offset_tuner_client_QNODE_HPP_
#define thormang3_offset_tuner_client_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN

#include <string>
#include <QThread>
#include <QStringListModel>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>

#include "thormang3_offset_tuner_msgs/JointOffsetData.h"
#include "thormang3_offset_tuner_msgs/JointOffsetPositionData.h"
#include "thormang3_offset_tuner_msgs/JointTorqueOnOff.h"
#include "thormang3_offset_tuner_msgs/JointTorqueOnOffArray.h"
#include "thormang3_offset_tuner_msgs/GetPresentJointOffsetData.h"

#endif  // Q_MOC_RUN

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace thormang3_offset_tuner_client
{

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNode : public QThread
{
Q_OBJECT
 public:
  enum LogLevel
  {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
  };
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();

  QStringListModel* loggingModel()
  {
    return &logging_model_;
  }
  void log(const LogLevel &level, const std::string &msg);

  void send_torque_enable_msg(thormang3_offset_tuner_msgs::JointTorqueOnOffArray msg);
  void send_joint_offset_data_msg(thormang3_offset_tuner_msgs::JointOffsetData msg);
  void send_command_msg(std_msgs::String msg);
  bool is_refresh()
  {
    return is_refresh_;
  }

  std::map<int, std::string> right_arm_offset_group;
  std::map<int, std::string> left_arm_offset_group;
  std::map<int, std::string> legs_offset_group;
  std::map<int, std::string> body_offset_group;

 public Q_SLOTS:
  void getPresentJointOffsetData(bool recalculate_offset = false);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

  void update_present_joint_offset_data(thormang3_offset_tuner_msgs::JointOffsetPositionData msg);

 private:
  void ParseOffsetGroup(const std::string &path);

  int init_argc_;
  char** init_argv_;
  bool is_refresh_;
  QStringListModel logging_model_;

  ros::Publisher joint_offset_data_pub_;
  ros::Publisher torque_enable_pub_;
  ros::Publisher command_pub_;

  ros::ServiceClient get_present_joint_offset_data_client_;
};

}  // namespace thormang3_offset_tuner_client

#endif /* thormang3_offset_tuner_client_QNODE_HPP_ */
