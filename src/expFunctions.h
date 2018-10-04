#ifndef TONNESFN_EXPERIMENTS_EXPFUNCTIONS_H
#define TONNESFN_EXPERIMENTS_EXPFUNCTIONS_H

#include <string>
#include "ros/ros.h"

#include "dyret_common/Configuration.h"
#include "dyret_common/Configure.h"
#include "dyret_common/Pose.h"
#include "dyret_common/State.h"

#include "dyret_controller/PositionCommand.h"
#include "dyret_controller/ActionMessage.h"
#include "dyret_controller/GetGaitEvaluation.h"
#include "dyret_controller/GetGaitControllerStatus.h"

std::string trim(std::string& str);

bool callServoConfigService(dyret_common::Configure givenCall, ros::ServiceClient givenServoConfigService);
bool sendServoTorqueMessage(bool enable, ros::ServiceClient givenServoConfigClient);
void sendActionMessage(bool sleep, ros::Publisher givenActionMessages_pub);

void enableServos(ros::Publisher givenActionMessages_pub);
void disableServos(ros::ServiceClient givenServoConfigClient, ros::Publisher givenActionMessages_pub);

bool startGaitRecording(ros::ServiceClient get_gait_evaluation_client);
bool resetGaitRecording(ros::ServiceClient get_gait_evaluation_client);

void sendRestPoseMessage(ros::Publisher givenActionMessages_pub);
void sendIdleMessage(ros::Publisher givenActionMessages_pub);
void sendContGaitMessage(double givenDirection, ros::Publisher givenActionMessages_pub);

#endif
