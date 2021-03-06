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
 * offset_tuner_server_node.cpp
 *
 *  Created on: 2016. 2. 15.
 *      Author: Jay SONG
 */

#include <ros/ros.h>

#include "thormang3_offset_tuner_server/thormang3_offset_tuner_server.h"

using namespace thormang3;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offset_tuner_server_node");

    OffsetTunerServer* server = OffsetTunerServer::getInstance();

    server->initialize();

    ros::spin();

	return 0;
}
