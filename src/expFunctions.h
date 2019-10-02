#ifndef TONNESFN_EXPERIMENTS_EXPFUNCTIONS_H
#define TONNESFN_EXPERIMENTS_EXPFUNCTIONS_H

#include <string>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>

#include <std_srvs/Trigger.h>

#include <gazebo/transport/transport.hh>

#include <dyret_common/Pose.h>
#include <dyret_common/State.h>
#include <dyret_common/Configure.h>
#include <dyret_common/timeHandling.h>
#include <dyret_common/Configuration.h>

#include <dyret_controller/ConfigureGait.h>
#include <dyret_controller/LoggerCommand.h>
#include <dyret_controller/PositionCommand.h>
#include <dyret_controller/GetGaitEvaluation.h>
#include <dyret_controller/GetInferredPosition.h>
#include <dyret_controller/GetGaitControllerStatus.h>
#include <dyret_controller/GaitControllerCommandService.h>

#include <camera_recorder/Record.h>
#include <camera_recorder/Configure.h>

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

std::string trim(std::string& str);
void printMap(std::map<std::string, double> givenMap, std::string givenLeadingString, FILE* givenFileDescriptor = stdout);

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

void playSound(std::string soundName, int number = 1);

void startWalking(ros::ServiceClient gaitCommandService_client);
void stopWalking(ros::ServiceClient gaitCommandService_client);

bool initLog(std::string individual, std::string logDirectoryPath, ros::ServiceClient loggerCommandService_client);
bool startLogging(ros::ServiceClient loggerCommandService_client);
bool saveLog(ros::ServiceClient loggerCommandService_client);

float getInferredPosition(ros::ServiceClient inferredPositionClient);

std::string getInputFromTerminal(std::string output);

bool legsAtRest(const std::array<int, 8> prismaticActuatorStates);

void setLegLengths(std::vector<float> lengths, ros::Publisher poseCommand_pub);
void setLegLengths(float femurLengths, float tibiaLengths, ros::Publisher poseCommand_pub);
void setLegLengths(float lengths, ros::Publisher poseCommand_pub);

void zeroPrismaticActuators(bool runOutFirst, ros::Publisher poseCommand_pub, std::array<int, 8> prismaticActuatorStates);

void setRandomRawFitness(ros::ServiceClient get_gait_evaluation_client, std::vector<std::map<std::string, double>> &rawFitnesses);

std::map<std::string, double> getGaitResults(ros::ServiceClient get_gait_evaluation_client);

bool enableFitnessLog(ros::ServiceClient get_gait_evaluation_client);
bool disableFitnessLog(ros::ServiceClient get_gait_evaluation_client);

void sendAngleCommand(std::vector<float> angles, ros::Publisher poseCommand_pub);

float getMaxServoTemperature(std::array<float, 12> servoTemperatures, bool printAllTemperatures = false);

void adjustGaitPose(ros::ServiceClient gaitCommandService_client);
void adjustRestPose(ros::ServiceClient gaitCommandService_client);

void spinGaitControllerOnce(ros::ServiceClient gaitCommandService_client);

double mapNumber(double value, double start1, double stop1, double start2, double stop2);
double getPoint(double givenNumber, double givenMinValue, double givenMaxValue, double givenOffset, double givenMinRange, double difficultyLevel);
std::map<std::string, double> genToHighLevelSplineGaitPhen(std::vector<double> givenGenotype, float frequencyFactor);
std::map<std::string, double> genToLowLevelSplineGaitPhen(std::vector<double> givenGenotype, float frequencyFactor, float gaitDifficultyFactor);
std::map<std::string, double> genToLowLevelAdvancedSplineGaitPhen(std::vector<double> givenGenotype, float frequencyFactor, float gaitDifficultyFactor);

std::string getDateString(struct tm *givenTime);
std::string createExperimentDirectory(std::string prefix, struct tm *givenTime);
std::string getCommitHash(std::string packageName);
void writeVersionLog(std::string givenLogDirectory);

std::vector<double> getRandomIndividual();

void recordSensorData(std::string label, float femurLength, float tibiaLength, int secondsToRecord, int numberOfDataPoints, ros::ServiceClient loggerCommandService_client);

float getDifficultyFactor(std::vector<std::string> commandQueue);

bool directoryExists(std::string givenPath);

void cooldownServos(ros::ServiceClient servoConfigClient, std::array<float, 12> servoTemperatures, ros::Publisher poseCommand_pub, std::vector<float> restPose);

std::string makeSensorDataDirectories(std::string givenSurface, int givenFemurLength, int givenTibiaLength);

void runGaitControllerWithActionMessage(bool forward,
                                        int currentIndividual,
                                        ros::ServiceClient get_gait_evaluation_client,
                                        ros::ServiceClient loggerCommandService_client,
                                        ros::ServiceClient gaitCommandService_client,
                                        ros::ServiceClient inferredPositionClient,
                                        bool enableLogging,
                                        int evaluationTimeout,
                                        float evaluationDistance,
                                        std::string logDirectoryPath);

void runGaitWithServiceCalls(float evaluationDistance,
                             int evaluationTimeout,
                             gazebo::WorldConnection* gz,
                             ros::ServiceClient get_gait_evaluation_client,
                             ros::ServiceClient inferredPositionClient,
                             ros::ServiceClient gaitCommandService_client);

void setGaitParams(std::string gaitType,
                   std::string logFilePath,
                   bool directionForward,
                   bool prepareForGait,
                   bool liveUpdate,
                   std::vector<float> femurLengths,
                   std::vector<float> tibiaLengths,
                   std::vector<std::string> parameterNames,
                   std::vector<float> parameterValues,
                   ros::ServiceClient gaitConfiguration_client);

void setGaitParams(std::string gaitType,
                   std::string logFilePath,
                   bool directionForward,
                   bool prepareForGait,
                   bool liveUpdate,
                   std::vector<float> femurLengths,
                   std::vector<float> tibiaLengths,
                   std::map<std::string, double> phenoTypeMap,
                   ros::ServiceClient gaitConfiguration_client);

void resetSimulation(gazebo::WorldConnection* gz);

#endif
