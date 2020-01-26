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
 * message_callback.h
 *
 *  Created on: 2016. 2. 20.
 *      Author: Jay Song
 */

#ifndef THOMAMG3_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_
#define THOMAMG3_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_


#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "thormang3_foot_step_generator/FootStepCommand.h"
#include "thormang3_foot_step_generator/Step2DArray.h"

#include "robotis_controller_msgs/StatusMsg.h"
#include "thormang3_walking_module_msgs/RobotPose.h"
#include "thormang3_walking_module_msgs/GetReferenceStepData.h"
#include "thormang3_walking_module_msgs/AddStepDataArray.h"
#include "thormang3_walking_module_msgs/StartWalking.h"
#include "thormang3_walking_module_msgs/SetBalanceParam.h"
#include "thormang3_walking_module_msgs/IsRunning.h"
#include "thormang3_walking_module_msgs/RemoveExistingStepData.h"

#include "robotis_foot_step_generator.h"


void initialize(void);

void walkingModuleStatusMSGCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg);

void walkingCommandCallback(const thormang3_foot_step_generator::FootStepCommand::ConstPtr& msg);
void step2DArrayCallback(const thormang3_foot_step_generator::Step2DArray::ConstPtr& msg);

bool isRunning(void);


#endif /* THOMAMG3_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_ */
