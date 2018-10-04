#include "expFunctions.h"

std::string trim(std::string& str){
    size_t first = str.find_first_not_of(' ');
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last-first+1));
}

bool callServoConfigService(dyret_common::Configure givenCall, ros::ServiceClient givenServoConfigService){
    if (givenServoConfigService.call(givenCall))  {
        switch(givenCall.response.status){
            case dyret_common::Configure::Response::STATUS_NOERROR:
                ROS_INFO("Configure servo service returned no error");
                break;
            case dyret_common::Configure::Response::STATUS_STATE:
                ROS_ERROR("State error from configure servo response");
                break;
            case dyret_common::Configure::Response::STATUS_PARAMETER:
                ROS_ERROR("Parameter error from configure servo response");
                break;
            default:
                ROS_ERROR("Unknown error from configure servo response");
                break;
        }

        if (givenCall.response.status == givenCall.response.STATUS_NOERROR) return true;

    } else {
        ROS_ERROR("Failed to call servo config service");
        return false;
    }
}

bool sendServoTorqueMessage(bool enable, ros::ServiceClient givenServoConfigClient){
    dyret_common::Configure srv;

    if (enable == true){
        srv.request.configuration.revolute.type =dyret_common::RevoluteConfig::TYPE_ENABLE_TORQUE;
    } else {
        srv.request.configuration.revolute.type =dyret_common::RevoluteConfig::TYPE_DISABLE_TORQUE;
    }

    return callServoConfigService(srv, givenServoConfigClient);
}

void sendActionMessage(bool sleep, ros::Publisher givenActionMessages_pub){
    dyret_controller::ActionMessage actionMessage;
    actionMessage.configuration = dyret_controller::ActionMessage::t_mammal;
    if (sleep == true) actionMessage.actionType = dyret_controller::ActionMessage::t_sleep; else actionMessage.actionType = dyret_controller::ActionMessage::t_restPose;
    actionMessage.speed = 0.0;
    actionMessage.direction = 0.0;
    givenActionMessages_pub.publish(actionMessage);
}

// This enables all servoes again. No need to send torque-message, as sending a position automatically enables torque
void enableServos(ros::Publisher givenActionMessages_pub){
    sendActionMessage(false, givenActionMessages_pub);
}

void disableServos(ros::ServiceClient givenServoConfigClient, ros::Publisher givenActionMessages_pub){
    sendActionMessage(true, givenActionMessages_pub);
    sleep(1);
    sendServoTorqueMessage(0, givenServoConfigClient);
}

bool startGaitRecording(ros::ServiceClient get_gait_evaluation_client){
    dyret_controller::GetGaitEvaluation srv;
    srv.request.givenCommand = dyret_controller::GetGaitEvaluation::Request::t_start;
    if (!get_gait_evaluation_client.call(srv)){
        ROS_ERROR("Error while calling GaitRecording service with t_start\n");
        return false;
    }

    return true;
}

bool resetGaitRecording(ros::ServiceClient get_gait_evaluation_client){
    dyret_controller::GetGaitEvaluation srv;
    srv.request.givenCommand = dyret_controller::GetGaitEvaluation::Request::t_resetStatistics;
    if (!get_gait_evaluation_client.call(srv)){
        ROS_ERROR("Error while calling GaitRecording service with t_resetStatistics\n");
        return false;
    }

    return true;

}

void sendRestPoseMessage(ros::Publisher givenActionMessages_pub){
    dyret_controller::ActionMessage actionMessage;
    actionMessage.configuration = dyret_controller::ActionMessage::t_mammal;
    actionMessage.actionType = dyret_controller::ActionMessage::t_restPose;
    actionMessage.speed = 0.0;
    actionMessage.direction = 0.0;
    givenActionMessages_pub.publish(actionMessage);
}

void sendIdleMessage(ros::Publisher givenActionMessages_pub){
    dyret_controller::ActionMessage actionMessage;
    actionMessage.configuration = dyret_controller::ActionMessage::t_mammal;
    actionMessage.actionType = dyret_controller::ActionMessage::t_idle;
    actionMessage.speed = 0.0;
    actionMessage.direction = 0.0;
    givenActionMessages_pub.publish(actionMessage);
}

void sendContGaitMessage(double givenDirection, ros::Publisher givenActionMessages_pub){
    dyret_controller::ActionMessage actionMessage;
    actionMessage.configuration = dyret_controller::ActionMessage::t_mammal;
    actionMessage.actionType = dyret_controller::ActionMessage::t_contGait;
    actionMessage.speed = 0.0;
    actionMessage.direction = givenDirection;
    givenActionMessages_pub.publish(actionMessage);
}

bool callConfigurationService(dyret_common::Configure givenCall, ros::ServiceClient givenServoConfigService){
    if (givenServoConfigService.call(givenCall))  {
        switch(givenCall.response.status){
            case dyret_common::Configure::Response::STATUS_NOERROR:
                ROS_INFO("Configuration service returned no error");
                break;
            case dyret_common::Configure::Response::STATUS_STATE:
                ROS_ERROR("State error from configuration service response");
                break;
            case dyret_common::Configure::Response::STATUS_PARAMETER:
                ROS_ERROR("Parameter error from configuration service response");
                break;
            default:
                ROS_ERROR("Unknown error from configuration service response");
                break;
        }

        if (givenCall.response.status == givenCall.response.STATUS_NOERROR) return true;

    } else {
        ROS_ERROR("Failed to call configuration service");
        return false;
    }

}

bool setServoPIDs(std::vector<double> givenPIDs, ros::ServiceClient givenConfigurationService){
    dyret_common::Configure msg;

    msg.request.configuration.revolute.ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; // Set all servos
    msg.request.configuration.revolute.type = msg.request.configuration.revolute.TYPE_SET_PID;

    for (int i = 0; i < 12; i++){

        if (i == 0 || i == 3 || i == 6 || i == 9){
            // Coxa
            msg.request.configuration.revolute.parameters.push_back(givenPIDs[0]);
            msg.request.configuration.revolute.parameters.push_back(givenPIDs[1]);
            msg.request.configuration.revolute.parameters.push_back(givenPIDs[2]);
        } else if (i == 1 || i == 4 || i == 7 || i == 10){
            // Femur
            msg.request.configuration.revolute.parameters.push_back(givenPIDs[3]);
            msg.request.configuration.revolute.parameters.push_back(givenPIDs[4]);
            msg.request.configuration.revolute.parameters.push_back(givenPIDs[5]);
        } else {
            // Tibia
            msg.request.configuration.revolute.parameters.push_back(givenPIDs[6]);
            msg.request.configuration.revolute.parameters.push_back(givenPIDs[7]);
            msg.request.configuration.revolute.parameters.push_back(givenPIDs[8]);
        }
    }

    callConfigurationService(msg, givenConfigurationService);

}