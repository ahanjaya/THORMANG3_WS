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
 * robotis_foot_step_generator.h
 *
 *  Created on: 2016. 6. 10.
 *      Author: Jay Song
 */

#ifndef THORMANG3_FOOT_STEP_GENERATOR_ROBOTIS_FOOT_STEP_GENERATOR_H_
#define THORMANG3_FOOT_STEP_GENERATOR_ROBOTIS_FOOT_STEP_GENERATOR_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include "thormang3_walking_module_msgs/AddStepDataArray.h"
#include "thormang3_foot_step_generator/Step2DArray.h"

#define STOP_WALKING           (0)
#define FORWARD_WALKING        (1)
#define BACKWARD_WALKING       (2)
#define RIGHTWARD_WALKING      (3)
#define LEFTWARD_WALKING       (4)
#define LEFT_ROTATING_WALKING  (5)
#define RIGHT_ROTATING_WALKING (6)

#define MINIMUM_STEP_TIME_SEC  (0.4)

namespace thormang3
{

class FootStepGenerator
{
public:
  FootStepGenerator();
  ~FootStepGenerator();

  void initialize();

  void calcRightKickStep(thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const thormang3_walking_module_msgs::StepData& ref_step_data);
  void calcLeftKickStep(thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const thormang3_walking_module_msgs::StepData& ref_step_data);

  void getStepData(thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const thormang3_walking_module_msgs::StepData& ref_step_data,
      int desired_step_type);

  void getStepDataFromStepData2DArray(thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type* step_data_array,
      const thormang3_walking_module_msgs::StepData& ref_step_data,
      const thormang3_foot_step_generator::Step2DArray::ConstPtr& request_step_2d);

  int    num_of_step_;
  double fb_step_length_m_;
  double rl_step_length_m_;
  double rotate_step_angle_rad_;

  double step_time_sec_;
  double start_end_time_sec_;
  double dsp_ratio_;

  double foot_z_swap_m_;
  double body_z_swap_m_;

  double default_y_feet_offset_m_;

private:
  bool calcStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int previous_step_type,  int desired_step_type);

  void calcFBStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction);
  void calcRLStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction);
  void calcRoStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction);
  void calcStopStep(const thormang3_walking_module_msgs::StepData& ref_step_data, int direction);

  Eigen::MatrixXd getTransformationXYZRPY(double position_x, double position_y, double position_z, double roll, double pitch, double yaw);
  void getPosefromTransformMatrix(const Eigen::MatrixXd &matTransform, double *position_x, double *position_y, double *position_z, double *roll, double *pitch, double *yaw);
  thormang3_walking_module_msgs::PoseXYZRPY getPosefromTransformMatrix(const Eigen::MatrixXd &matTransform);
  Eigen::MatrixXd getInverseTransformation(Eigen::MatrixXd transform);

  thormang3_walking_module_msgs::AddStepDataArray::Request::_step_data_array_type step_data_array_;

  int previous_step_type_;

};


}



#endif /* THORMANG3_FOOT_STEP_GENERATOR_ROBOTIS_FOOT_STEP_GENERATOR_H_ */
