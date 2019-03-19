#ifndef TONNESFN_EXPERIMENTS_EXPFUNCTIONS_H
#define TONNESFN_EXPERIMENTS_EXPFUNCTIONS_H

#include <string>
#include "ros/ros.h"

#include <gazebo/transport/transport.hh>

#include "dyret_common/Configuration.h"
#include "dyret_common/Configure.h"
#include "dyret_common/Pose.h"
#include "dyret_common/State.h"

#include "dyret_controller/PositionCommand.h"
#include "dyret_controller/GetGaitEvaluation.h"
#include "dyret_controller/GetGaitControllerStatus.h"

#include "camera_recorder/Configure.h"
#include "camera_recorder/Record.h"

#include <std_srvs/Trigger.h>

std::string trim(std::string& str);
void printMap(std::map<std::string, double> givenMap, std::string givenLeadingString, FILE * givenDestination);

double getMapValue(const std::map<std::string, double> &givenMap, const std::string &givenValue);

bool callServoConfigService(dyret_common::Configure givenCall, ros::ServiceClient givenServoConfigService);
bool sendServoTorqueMessage(bool enable, ros::ServiceClient givenServoConfigClient);
bool enableServos(ros::ServiceClient givenServoConfigClient);
bool disableServos(ros::ServiceClient givenServoConfigClient);
bool restartServos(ros::ServiceClient givenServoConfigClient);
bool setServoSpeeds(float givenSpeed, ros::ServiceClient givenServoConfigClient);

bool startGaitRecording(ros::ServiceClient get_gait_evaluation_client);
bool resetGaitRecording(ros::ServiceClient get_gait_evaluation_client);
bool pauseGaitRecording(ros::ServiceClient get_gait_evaluation_client);

void pauseGazebo();
void unpauseGazebo();

void startVideo(std::string fileName);
void stopVideo();

namespace gazebo {
    class WorldConnection {
    private:
        // Global connection to Gazebo
        gazebo::transport::NodePtr node;
        // Connection to publish messages to simulation
        gazebo::transport::PublisherPtr pub;
        // Private constructor!
        WorldConnection();
        ~WorldConnection();

    public:
        // Send step message to Gazebo and let ROS code complete
        void step(const size_t steps = 1);

        // Reset all models in simulation to initial positions
        bool reset();

        // Delete copy constructor to avoid unintended copy!
        WorldConnection(const WorldConnection&) = delete;
        void operator=(const WorldConnection&)  = delete;

        // Create static method to retrieve instance
        static WorldConnection& instance() {
            static WorldConnection inst;
            return inst;
        }
    };
}

#endif
