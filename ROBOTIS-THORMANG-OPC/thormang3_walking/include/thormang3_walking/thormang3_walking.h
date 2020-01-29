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
 * thormang3_walking.h
 *
 *  Edited on: 2019. 8. 15.
 *      Editor: Hanjaya Mandala
 */

#ifndef THORMANG3_WALKING_THORMANG3_WALKING_H_
#define THORMANG3_WALKING_THORMANG3_WALKING_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>

#include "robotis_controller_msgs/StatusMsg.h"
#include "thormang3_walking_module_msgs/RobotPose.h"
#include "thormang3_walking_module_msgs/GetReferenceStepData.h"
#include "thormang3_walking_module_msgs/AddStepDataArray.h"
#include "thormang3_walking_module_msgs/StartWalking.h"
#include "thormang3_walking_module_msgs/SetBalanceParam.h"
#include "thormang3_walking_module_msgs/SetJointFeedBackGain.h"
#include "thormang3_walking_module_msgs/IsRunning.h"
#include "thormang3_walking_module_msgs/RemoveExistingStepData.h"
#include "thormang3_foot_step_generator/FootStepCommand.h"

void initialize();

void moveToInitPose();
void moveToResetPose();

void setCtrlModule();

bool loadBalanceParam(thormang3_walking_module_msgs::SetBalanceParam& set_param);
bool loadFeedBackGain(thormang3_walking_module_msgs::SetJointFeedBackGain& set_gain);
void setBalanceOn();
void setBalanceOff();

void setWalkingCommand(thormang3_foot_step_generator::FootStepCommand msg);

template<typename T>
T deg2rad(T deg)
{
  return deg * M_PI / 180;
}

#endif /* THORMANG3_WALKING_THORMANG3_WALKING_H_ */
