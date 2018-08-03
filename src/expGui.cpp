#include <iostream>
#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <chrono>
#include <unistd.h>

#include "ros/ros.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include "dyret_common/Configuration.h"
#include "dyret_common/Configure.h"
#include "dyret_common/Pose.h"
#include "dyret_common/State.h"

#include "dyret_common/timeHandling.h"
#include "dyret_common/wait_for_ros.h"

#include "dyret_controller/PositionCommand.h"
#include "dyret_controller/ActionMessage.h"
#include "dyret_controller/Trajectory.h"
#include "dyret_controller/GetGaitEvaluation.h"
#include "dyret_controller/GetGaitControllerStatus.h"

#include "external/sferes/phen/parameters.hpp"
#include "external/sferes/gen/evo_float.hpp"
#include "external/sferes/ea/nsga2.hpp"
#include "external/sferes/eval/eval.hpp"
#include "external/sferes/stat/pareto_front.hpp"
#include "external/sferes/modif/dummy.hpp"
#include "external/sferes/run.hpp"
#include <boost/program_options.hpp>

#include <sstream>
#include <iostream>

#include "individuals.h"

#include "evoSettings.h"

ros::ServiceClient servoConfigClient;
ros::ServiceClient get_gait_evaluation_client;
ros::Publisher trajectoryMessage_pub;
ros::Publisher poseCommand_pub;
ros::ServiceClient gaitControllerStatus_client;
ros::Subscriber dyretState_sub;
ros::ServiceClient servoStatus_client;
ros::Publisher actionMessages_pub;
ros::Publisher positionCommand_pub;

FILE* logOutput = stdout;

double currentFemurLength = 0.0;
double currentTibiaLength = 0.0;
int evaluationTimeout = 15; // 15 sec max each direction
float evaluationDistance = 1500.0;
int currentIndividual;

std::string evoLogPath;

const int numberOfEvalsInTesting = 1;

constexpr float phen_maxStepLength = 300.0;
constexpr float phen_maxFrequency  = 2.0;

bool evolveMorph = true;
bool addDiversity = true;
bool instantFitness = false;

std::string morphology;

bool robotOnStand = false;

char **argv_g;

std::vector<std::string> rawFitnesses; // Used to store raw fitness string until log writing
std::vector<std::string> fitnessFunctions; // Used to specify which fitness functions to use
std::vector<std::string> commandQueue; // Used to store commands from the arguments temporarily
std::string fullCommand; // Used to store commands from the arguments permanently

bool automatedRun(){
  return !fullCommand.empty();
}

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

bool sendServoTorqueMessage(bool enable){
  dyret_common::Configure srv;

  if (enable == true){
    srv.request.configuration.revolute.type =dyret_common::RevoluteConfig::TYPE_ENABLE_TORQUE;
  } else {
    srv.request.configuration.revolute.type =dyret_common::RevoluteConfig::TYPE_DISABLE_TORQUE;
  }

  return callServoConfigService(srv, servoConfigClient);
}

void sendActionMessage(bool sleep){
  dyret_controller::ActionMessage actionMessage;
  actionMessage.configuration = dyret_controller::ActionMessage::t_mammal;
  if (sleep == true) actionMessage.actionType = dyret_controller::ActionMessage::t_sleep; else actionMessage.actionType = dyret_controller::ActionMessage::t_restPose;
  actionMessage.speed = 0.0;
  actionMessage.direction = 0.0;
  actionMessages_pub.publish(actionMessage);
}

// This enables all servoes again. No need to send torque-message, as sending a position automatically enables torque
void enableServos(){
  sendActionMessage(false);
}

void disableServos(){
  sendActionMessage(true);
  sleep(1);
  sendServoTorqueMessage(0);
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

std::vector<float> getGaitResults(ros::ServiceClient get_gait_evaluation_client){
  dyret_controller::GetGaitEvaluation srv;
  std::vector<float> vectorToReturn;

  srv.request.givenCommand = dyret_controller::GetGaitEvaluation::Request::t_getResults;

  if (get_gait_evaluation_client.call(srv)) {
    if (srv.response.descriptors.size() != srv.response.results.size()){
      ROS_ERROR("Result and descriptor sizes not equal. Cannot process results!");
      return std::vector<float>();
    }

    std::stringstream ss;

    ss  << "        {\n";
    for (int i = 0; i < srv.response.descriptors.size(); i++){
      ss <<  "          \"" << srv.response.descriptors[i] << "\": " << std::setprecision(4) << srv.response.results[i];
      if (i == srv.response.descriptors.size()-1) ss << "\n"; else ss << ",\n";
    }

    ss << "        }";

    rawFitnesses.push_back(ss.str());

    vectorToReturn = srv.response.results;
  } else {
    ROS_ERROR("Error while calling GaitRecording service with t_getResults!\n");
  }

  return vectorToReturn;
}

void setGaitParams(double givenStepLength,
                   double givenStepHeight,
                   double givenSmoothing,
                   double givenGaitFrequency,
                   double givenGaitSpeed,
                   double givenWagPhaseOffset,
                   double givenWagAmplitude_x,
                   double givenWagAmplitude_y,
                   double givenLiftDuration){

  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter param_stepLength;
  dynamic_reconfigure::DoubleParameter param_stepHeight;
  dynamic_reconfigure::DoubleParameter param_smoothing;
  dynamic_reconfigure::DoubleParameter param_gaitFrequency;
  dynamic_reconfigure::DoubleParameter param_gaitSpeed;
  dynamic_reconfigure::DoubleParameter param_wagAmplitude_x;
  dynamic_reconfigure::DoubleParameter param_wagAmplitude_y;
  dynamic_reconfigure::DoubleParameter param_wagPhase;
  dynamic_reconfigure::DoubleParameter param_liftDuration;
  dynamic_reconfigure::IntParameter param_cP;
  dynamic_reconfigure::IntParameter param_cI;
  dynamic_reconfigure::IntParameter param_cD;
  dynamic_reconfigure::Config conf;

  param_stepLength.name = "stepLength";
  param_stepLength.value = givenStepLength;
  param_stepHeight.name = "stepHeight";
  param_stepHeight.value = givenStepHeight;
  param_smoothing.name  = "smoothing";
  param_smoothing.value = givenSmoothing;
  param_gaitFrequency.name  = "gaitFrequency";
  param_gaitFrequency.value = givenGaitFrequency;
  param_gaitSpeed.name  = "gaitSpeed";
  param_gaitSpeed.value = givenGaitSpeed;
  param_wagAmplitude_x.name = "wagAmplitude_x";
  param_wagAmplitude_x.value = givenWagAmplitude_x;
  param_wagAmplitude_y.name = "wagAmplitude_y";
  param_wagAmplitude_y.value = givenWagAmplitude_y;
  param_wagPhase.name = "wagPhase";
  param_wagPhase.value = givenWagPhaseOffset;
  param_liftDuration.name = "liftDuration";
  param_liftDuration.value = givenLiftDuration;
  param_cP.name = "cP";
  param_cP.value = 10;
  param_cI.name = "cI";
  param_cI.value = 0;
  param_cD.name = "cD";
  param_cD.value = 0;
  conf.doubles.push_back(param_stepLength);
  conf.doubles.push_back(param_stepHeight);
  conf.doubles.push_back(param_smoothing);
  conf.doubles.push_back(param_gaitFrequency);
  conf.doubles.push_back(param_gaitSpeed);
  conf.doubles.push_back(param_wagAmplitude_x);
  conf.doubles.push_back(param_wagAmplitude_y);
  conf.doubles.push_back(param_wagPhase);
  conf.doubles.push_back(param_liftDuration);

  if (robotOnStand == true) {
    conf.ints.push_back(param_cP);
    conf.ints.push_back(param_cI);
    conf.ints.push_back(param_cD);
  }

  srv_req.config = conf;

  ros::service::call("/gaitController/set_parameters", srv_req, srv_resp);
}

bool gaitControllerDone(ros::ServiceClient gaitControllerStatus_client){
                        dyret_controller::GetGaitControllerStatus srv;

  if (gaitControllerStatus_client.call(srv)) {
      if (srv.response.gaitControllerStatus.actionType == srv.response.gaitControllerStatus.t_idle) return true;
  }

  return false;
}

void resetTrajectoryPos(ros::Publisher givenTrajectoryMsgPublisher){
  dyret_controller::Trajectory msg;
  msg.command = msg.t_resetPosition;
  givenTrajectoryMsgPublisher.publish(msg);
}

void sendTrajectories(std::vector<float> givenTrajectoryDistances, std::vector<float> givenTrajectoryAngles, std::vector<int> givenTrajectoryTimeouts, ros::Publisher givenTrajectoryMsgPublisher){
  dyret_controller::Trajectory msg;

  msg.trajectorySegments.resize(givenTrajectoryDistances.size());
  for (int i = 0; i < givenTrajectoryDistances.size(); i++){
      msg.trajectorySegments[i].distance = givenTrajectoryDistances[i];
      msg.trajectorySegments[i].angle = givenTrajectoryAngles[i];
      msg.trajectorySegments[i].timeoutInSec = givenTrajectoryTimeouts[i];
  }

  givenTrajectoryMsgPublisher.publish(msg);

}

bool isFitnessObjective(std::string givenString){
  return (std::find(fitnessFunctions.begin(), fitnessFunctions.end(), givenString) != fitnessFunctions.end());
}

void setLegLengths(float femurLengths, float tibiaLengths){
  dyret_common::Pose msg;

  msg.prismatic.resize(2);

  msg.prismatic[0] = femurLengths;
  msg.prismatic[1] = tibiaLengths;

  poseCommand_pub.publish(msg);
}

// Positions is a 12 length vector with positions for all 4 legs
void setLegPositions(std::vector<float> positions){
  assert(positions.size() == 12);
  dyret_controller::PositionCommand msg;

  for (int i = 0; i < msg.legPosition.size(); i++){
    msg.legPosition[i].x = positions[i*3];
    msg.legPosition[i].y = positions[(i*3)+1];
    msg.legPosition[i].z = positions[(i*3)+2];
  }

  positionCommand_pub.publish(msg);

}

void testLegPositionAll(std::vector<float> position){
  assert(position.size() == 3);
  std::vector<float> positions;
  for (int i = 0; i < 4; i++) positions.insert(positions.end(), position.begin(), position.end());

  setLegPositions(positions);
}

void dyretStateCallback(const dyret_common::State::ConstPtr& msg) {
  currentFemurLength = (msg->prismatic[0].position + msg->prismatic[2].position + msg->prismatic[4].position + msg->prismatic[6].position) / 4.0;
  currentTibiaLength = (msg->prismatic[1].position + msg->prismatic[3].position + msg->prismatic[5].position + msg->prismatic[7].position) / 4.0;
}

bool legsAreLength(float femurLengths, float tibiaLengths){
  return ( (fabs(femurLengths - currentFemurLength) < 3.0f) && (fabs(tibiaLengths - currentTibiaLength) < 3.0f) );
}

float getServoVoltage(){
/*
  dyret_common::GetServoStatuses  gssres;
  servoStatus_client.call(gssres);

  float voltages = 0.0;
  float counter = 0.0;

  for (int i = 0; i < gssres.response.servoStatuses.size(); i++){
    voltages += gssres.response.servoStatuses[i].voltage;
    counter += 1.0;
  }

  return voltages / counter;*/
  return 0.0;
}

float getMaxServoTemperature(bool printAllTemperatures = false){
/*  float maxTemp = -1.0;

  dyret_common::GetServoStatuses  gssres;
  servoStatus_client.call(gssres);

  if (printAllTemperatures) printf("Servo temperatures: ");
  for (int i = 0; i < gssres.response.servoStatuses.size(); i++){
    if (gssres.response.servoStatuses[i].temperature > maxTemp) maxTemp = gssres.response.servoStatuses[i].temperature;
    if (printAllTemperatures) printf("%.2f ",gssres.response.servoStatuses[i].temperature);
  }
  if (printAllTemperatures) printf("\n");

  return maxTemp;*/

  return 0.0;
}

std::vector<float> evaluateIndividual(std::vector<double> phenoType,
                                      bool speedAspectLocked,
                                      ros::ServiceClient gaitControllerStatus_client,
                                      ros::Publisher trajectoryMessage_pub,
                                      ros::ServiceClient get_gait_evaluation_client) {

  currentIndividual++;

  // (Return random fitness to test)
  if (instantFitness == true) {
    usleep(30000);

    rawFitnesses.clear();

    std::stringstream ss1;

    ss1  << "        {\n";
    for (int i = 0; i < 7; i++){
      ss1 <<  "          \"rawFitness_" << i << "\": " << std::setprecision(4) << (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
      if (i == 6) ss1 << "\n"; else ss1 << ",\n";
    }

    ss1 << "        }";

    rawFitnesses.push_back(ss1.str());

    std::stringstream ss2;

    ss2  << "        {\n";
    for (int i = 0; i < 7; i++){
      ss2 <<  "          \"rawFitness_" << i << "\": " << std::setprecision(4) << (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
      if (i == 6) ss2 << "\n"; else ss2 << ",\n";
    }

    ss2 << "        }";

    rawFitnesses.push_back(ss2.str());

    return std::vector<float>{static_cast <float> (rand()) / static_cast <float> (RAND_MAX),
                              static_cast <float> (rand()) / static_cast <float> (RAND_MAX)};
  }

  if (ros::Time::isSystemTime()) {
    // Code to stop for cooldown at the start of each new generation:
    if (currentIndividual == popSize) {
      currentIndividual = 0;
      getMaxServoTemperature(true);
      fprintf(logOutput, "Cooldown to 50C? (y/n) > ");

      char choice;
      scanf(" %c", &choice);

      if (choice == 'y') {
        disableServos();
        long long int currentTime = getMs();
        fprintf(logOutput, "00.0 ");
        while (getMaxServoTemperature(true) > 50) {
          sleep(10);
          fprintf(logOutput, "%3.1f: ", ((getMs() - currentTime) / 1000.0) / 60.0);
        }

        std::cout << "Press enter to enable servos";
        std::cin.ignore();
        std::cin.ignore();

        enableServos();

        std::cout << "Press enter to continue evolution";
        std::cin.ignore();
      }
    }
  }

  fprintf(logOutput, "%03u: Evaluating stepLength %.2f, "
             "stepHeight %.2f, "
             "smoothing %.2f, "
             "frequency: %.2f, "
             "speed: %.2f, "
             "wagPhase: %.2f, "
             "wagAmplitude_x: %.2f, "
             "wagAmplitude_y: %.2f,"
             "femurLength: %.2f,"
             "tibiaLength: %.2f,"
             "liftDuration: %.2f\n",
         currentIndividual,
         phenoType[0],
         phenoType[1],
         phenoType[2],
         phenoType[3],
         phenoType[4],
         phenoType[5],
         phenoType[6],
         phenoType[7],
         phenoType[8],
         phenoType[9],
         phenoType[10]);

  if (ros::Time::isSystemTime()) { // Only check temperature in real world
    // Check temperature - if its over the limit below, consider fitness invalid (due to DC motor characterics)
    if (getMaxServoTemperature() > 70.0) {
      fprintf(logOutput, "  Temperature is too high at %.1f\n", getMaxServoTemperature());
      return std::vector<float>();
    }
  }

  setGaitParams(phenoType[0], phenoType[1], phenoType[2], phenoType[3], phenoType[4], phenoType[5], phenoType[6], phenoType[7], phenoType[10]);

  // Set leg lengths and wait until they reach the correct length
  setLegLengths(phenoType[8], phenoType[9]);
  int secPassed = 0;
  while (!legsAreLength(phenoType[8], phenoType[9])) {
    sleep(1);
    if (secPassed++ > 60) return std::vector<float>(); // 1 min timeout
  }

  std::vector<float> trajectoryAngles(1);
  std::vector<float> trajectoryDistances(1);
  std::vector<int> trajectoryTimeouts(1);
  trajectoryDistances[0] = evaluationDistance;
  trajectoryTimeouts[0] = evaluationTimeout;
  resetTrajectoryPos(trajectoryMessage_pub); // Reset position before starting
  resetGaitRecording(get_gait_evaluation_client);
  sendTrajectories(trajectoryDistances, trajectoryAngles, trajectoryTimeouts, trajectoryMessage_pub);
  sleep(5);

  secPassed = 0;
  while (gaitControllerDone(gaitControllerStatus_client) == false) {
    sleep(1);
    if (secPassed++ > (evaluationTimeout + 60)) {
      fprintf(logOutput, "Timed out at first evaluation (%d seconds)\n", secPassed);
      return std::vector<float>();
    }
  }

  std::vector<float> gaitResultsForward = getGaitResults(get_gait_evaluation_client);

  if (gaitResultsForward.size() == 0) {
    fprintf(stderr, "GaitResultsForward.size() == 0!\n");
    return gaitResultsForward;
  }

  fprintf(logOutput, "  Res F: ");
  for (int i = 0; i < gaitResultsForward.size(); i++) {
    fprintf(logOutput, "%.5f", gaitResultsForward[i]);
    if (i != (gaitResultsForward.size() - 1)) fprintf(logOutput, ", "); else fprintf(logOutput, "\n");
  }

  trajectoryDistances[0] = 0.0;
  trajectoryTimeouts[0] = evaluationTimeout;

  //resetTrajectoryPos(trajectoryMessage_pub); // Reset position before starting
  resetGaitRecording(get_gait_evaluation_client);
  sendTrajectories(trajectoryDistances, trajectoryAngles, trajectoryTimeouts, trajectoryMessage_pub);
  sleep(5);

  secPassed = 0;
  while (gaitControllerDone(gaitControllerStatus_client) == false) {
    sleep(1);
    if (secPassed++ > (evaluationTimeout + 60)) {
      fprintf(logOutput, "Timed out at second evaluation (%d seconds)\n", secPassed);
      return std::vector<float>();
    }
  }

  std::vector<float> gaitResultsReverse = getGaitResults(get_gait_evaluation_client);

  if (gaitResultsReverse.size() == 0) {
    fprintf(stderr, "GaitResultsReverse.size() == 0!\n");
    return gaitResultsReverse;
  }

  fprintf(logOutput, "  Res R: ");
  for (int i = 0; i < gaitResultsReverse.size(); i++) {
    fprintf(logOutput, "%.5f", gaitResultsReverse[i]);
    if (i != (gaitResultsReverse.size() - 1)) fprintf(logOutput, ", "); else fprintf(logOutput, "\n");
  }

  std::vector<float> fitness(fitnessFunctions.size());
  int currentFitnessIndex = 0;

  float fitness_inferredSpeed = (gaitResultsForward[0] + gaitResultsReverse[0]) / 2.0;
  float fitness_current = (gaitResultsForward[4] + gaitResultsReverse[4]) / 2.0;
  float fitness_stability = (gaitResultsForward[6] + gaitResultsReverse[6]) / 2.0;
  float fitness_mocapSpeed = (gaitResultsForward[5] + gaitResultsReverse[5]) / 2.0;

  // Inferred speed:
  if (isFitnessObjective("InferredSpeed")) {
    fitness[currentFitnessIndex] = fitness_inferredSpeed;
    currentFitnessIndex++;
  }

  if (isFitnessObjective("Current")) {
    fitness[currentFitnessIndex] = fitness_current;
    currentFitnessIndex++;
  }

  if (isFitnessObjective("Stability")) {
    fitness[currentFitnessIndex] = fitness_stability;
    currentFitnessIndex++;
  }

  if (isFitnessObjective("MocapSpeed")) {
    fitness[currentFitnessIndex] = fitness_mocapSpeed;
    currentFitnessIndex++;
  }

  return fitness;

}

using namespace sferes;
using namespace sferes::gen::evo_float;

struct Params {
  struct evo_float {
    SFERES_CONST float cross_rate = 0.0f;
    SFERES_CONST float mutation_rate = 1.0f;
    SFERES_CONST float sigma = 1.0f/6.0f;
    SFERES_CONST mutation_t mutation_type = gaussian;
    SFERES_CONST cross_over_t cross_over_type = recombination;
  };
  struct pop {
    SFERES_CONST unsigned size     =   popSize;  // Population size
    SFERES_CONST unsigned nb_gen   =   generations-1;  // Number of generations
    SFERES_CONST int dump_period   =    1;  // How often to save
    SFERES_CONST int initial_aleat =    1;  // Individuals to be created during random generation process
  };
  struct parameters {
    SFERES_CONST float min = 0.0f;
    SFERES_CONST float max = 1.0f;
  };
};

std::vector<double> genToPhen(std::vector<double> givenGenotype){

  std::vector<double> phenotype = {givenGenotype[0] * phen_maxStepLength,  // 0: stepLength
                                   25.0 + (givenGenotype[1] * 50.0),       // 1: stepHeight        25 -> 75
                                   givenGenotype[2] * 50.0,                // 2: smoothing          0 -> 50
                                   givenGenotype[6] * phen_maxFrequency,   // 6: frequency
                                   NAN,                                    // 6: speed
                                   (givenGenotype[3] * 0.4) - 0.2,         // 3: wagPhase        -0.2 -> 0.2
                                   givenGenotype[4] * 50.0,                // 4: wagAmplitude_x     0 -> 50
                                   givenGenotype[5] * 50.0,                // 5: wagAmplitude_y     0 -> 50
                                   givenGenotype[7] * 25.0,                // 7: femurLength        0 -> 25
                                   givenGenotype[8] * 95.0,                // 8: tibiaLength        0 -> 95
                                   (givenGenotype[9] * 0.15) + 0.05        // 9: liftDuration    0.05 -> 0.20
  };

  return phenotype;
}

std::vector<double> phenToGen(std::vector<double> givenFenotype){

  std::vector<double> genotype = {givenFenotype[0] / phen_maxStepLength,  // 0: stepLength
                                  (givenFenotype[1] - 25) / 50.0,         // 1: stepHeight        25 -> 75
                                  givenFenotype[2] / 50.0,                // 2: smoothing          0 -> 50
                                  (givenFenotype[5] + 0.2) / 0.4,         // 3: wagPhase        -0.2 -> 0.2
                                  givenFenotype[6] / 50.0,                // 4: wagAmplitude_x     0 -> 50
                                  givenFenotype[7] / 50.0,                // 5: wagAmplitude_y     0 -> 50
                                  givenFenotype[3] / phen_maxFrequency,   // 6: frequency
                                  givenFenotype[8] / 25.0,                // 7: femurLength        0 -> 25
                                  givenFenotype[9] / 95.0,                // 8: tibiaLength        0 -> 95
                                  (givenFenotype[10] - 0.05) / 0.15       // 9: liftDuration    0.05 -> 0.20
  };

  return genotype;

}


SFERES_FITNESS(FitExp2MO, sferes::fit::Fitness) {
public:
  FitExp2MO()  {}
  template<typename Indiv>
  void eval(Indiv& ind) {

    rawFitnesses.clear(); // Clear any residual fitness strings from earlier experiments

    // Only add diversity if we are evolving morphology
    if (evolveMorph == true && addDiversity) {
      this->_objs.resize(fitnessFunctions.size() + 1);
    } else {
      this->_objs.resize(fitnessFunctions.size());
    }

    // Set length of individual if not evolving morphology:
    if (evolveMorph == false) {
      if (morphology == "small") {
        ind.gen().data(7, 0.0);
        ind.gen().data(8, 0.0);
      } else if (morphology == "medium"){
        ind.gen().data(7, 0.5);
        ind.gen().data(8, 0.5);
      } else if (morphology == "large") {
        ind.gen().data(7, 1.0);
        ind.gen().data(8, 1.0);
      } else {
        fprintf(stderr, "Morphology \"%s\" not recognized!\n", morphology.c_str());
      }
    }

    std::vector<double> individualData(10);

    for (int i = 0; i < individualData.size(); i++){
      individualData[i] = ind.gen().data(i);
    }

    std::vector<double> individualParameters = genToPhen(individualData);

    bool validSolution;
    std::vector<float> fitnessResult;

    int retryCounter = 0;

    do{
        validSolution = true;
        fitnessResult = evaluateIndividual(individualParameters, false, gaitControllerStatus_client, trajectoryMessage_pub, get_gait_evaluation_client);

        fprintf(logOutput, "    Fitness received: ");
        for (int i = 0; i < fitnessResult.size(); i++) fprintf(logOutput, "%.2f ", fitnessResult[i]);
        fprintf(logOutput, "\n");

        for (int i = 0; i < fitnessResult.size(); i++){
            if (std::isnan(fitnessResult[i]) == true){
                validSolution = false;
            }
        }

        if (fitnessResult.size() == 0) validSolution = false;

        // A valid solution could not be found:
        if ((validSolution == false) || (fitnessResult.size() == 0)){
          if (automatedRun){
            if (retryCounter < 3){
              fprintf(logOutput, "Retrying\n");
              retryCounter += 1;
              currentIndividual--;
              validSolution = false;
            } else {
              fprintf(logOutput, "ABORT - after three retries without results\n");
              exit(-1);
            }
          } else {
            fprintf(logOutput, "Got invalid fitness: choose action ((r)etry/(d)iscard/(c)ooldown): ");

            char choice;
            scanf(" %c", &choice);

            if (choice == 'd') { // Discard
              fitnessResult.resize(fitnessFunctions.size());
              for (int i = 0; i < fitnessResult.size(); i++) fitnessResult[i] = -1000;

              fprintf(logOutput, "Discarding\n");
              validSolution = true;
            } else if (choice == 'c') {
              disableServos();
              fprintf(logOutput, "Servos disabled\n");

              while (getMaxServoTemperature(true) > 50) { sleep(15); }

              enableServos();

              std::cout << "Press enter to continue evolution";
              std::cin.ignore();
              std::cin.ignore();

              currentIndividual--;
              validSolution = false;
            } else {
              fprintf(logOutput, "Retrying\n");
              currentIndividual--;
              validSolution = false;
            }
          }
        }

    } while (validSolution == false);

    for (int i = 0; i < fitnessResult.size(); i++){
        this->_objs[i] = fitnessResult[i];
    }

    FILE * genFile = fopen("generation", "r");

    FILE * evoLog = fopen(evoLogPath.c_str(), "a");
    if (evoLog == NULL){
      ROS_ERROR("evoLog couldnt be opened (err%d)\n", errno);
    }

    int currentGeneration;
    fscanf(genFile, "%d", &currentGeneration);
    fclose(genFile);
    fprintf(evoLog, "    {\n");
    fprintf(evoLog, "      \"id\": %d,\n", currentIndividual);
    fprintf(evoLog, "      \"generation\": %d,\n", currentGeneration);

    fprintf(evoLog, "      \"genotype\": [\n");
    for (int i = 0; i < individualData.size(); i++){
      fprintf(evoLog, "        %.2f", individualData[i]);
      if (i != individualData.size()-1) fprintf(evoLog, ",");
      fprintf(evoLog, "  \n");
    }
    fprintf(evoLog, "      ],\n");

    fprintf(evoLog, "      \"phenotype\": [\n");
    for (int i = 0; i < individualParameters.size(); i++){
      if (isnan(individualParameters[i])) fprintf(evoLog, "        \"NaN\""); else fprintf(evoLog, "        %.2f", individualParameters[i]);

      if (i != individualParameters.size()-1) fprintf(evoLog, ",");
      fprintf(evoLog, "  \n");
    }
    fprintf(evoLog, "      ],\n");

    fprintf(evoLog, "      \"fitness\": {\n");
    for (int i = 0; i < fitnessResult.size(); i++){
      fprintf(evoLog, "        \"%s\": %.2f", fitnessFunctions[i].c_str(), fitnessResult[i]);
      if (i != fitnessResult.size()-1) fprintf(evoLog, ",");
      fprintf(evoLog, "  \n");
    }
    fprintf(evoLog, "      },\n");

    fprintf(evoLog, "      \"raw_fitness\": [\n");
    for (int i = 0; i < rawFitnesses.size(); i++){
      fprintf(evoLog, "%s", rawFitnesses[i].c_str());
      if (i == rawFitnesses.size()-1) fprintf(evoLog, "\n"); else fprintf(evoLog, ",\n");
    }

    fprintf(evoLog, "      ]\n");

    fprintf(evoLog, "    }");

    if (currentIndividual == ((generations)*popSize)-1){ // If last individual
      fprintf(evoLog, "\n");
      fprintf(evoLog, "  ]\n");
      fprintf(evoLog, "}");
    } else {
      fprintf(evoLog, ",\n");
    }

    fclose(evoLog);

  }
};

namespace expGui {

// Setup evolutionary framework
  typedef gen::EvoFloat<10, Params> gen_t;   // Number of parameters in each individual:
  typedef phen::Parameters<gen_t, FitExp2MO<Params>, Params> phen_t;
  typedef eval::Eval<Params> eval_t;
  typedef boost::fusion::vector<stat::State<phen_t, Params> > stat_t;
  typedef modif::Dummy<> modifier_t;
  typedef ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
  ea_t ea;

}

void run_individual(std::vector<double> givenPhenotype){

  fitnessFunctions.clear();
  fitnessFunctions.emplace_back("MocapSpeed");
  fitnessFunctions.emplace_back("Stability");

  for (int i = 0; i < numberOfEvalsInTesting; i++) {

    std::vector<float> fitnessResult = evaluateIndividual(givenPhenotype, false,
                                                          gaitControllerStatus_client, trajectoryMessage_pub,
                                                          get_gait_evaluation_client);
    fprintf(logOutput, "Returned fitness (%lu): ", fitnessResult.size());
  }
}

void menu_demo(){

  std::string choice;

  std::cout << "  Please choose one demonstration: (enter to go back)\n";

  fprintf(logOutput, "    ss - Test small robot (small control)\n"
         "    ls - Test large robot (small control)\n"
         "    ll - Test large robot (large control)\n"
         "    ms - Request small morphology\n"
         "    mm - Request medium morphology\n"
         "    ml - Request large morphology\n");
  fprintf(logOutput, "\n> ");

  if (commandQueue.empty()) {
    getline(std::cin, choice);
  } else {
    choice = commandQueue[0];
    fprintf(logOutput, "*%s*\n", choice.c_str());
    commandQueue.erase(commandQueue.begin());
  }

  if (choice.empty() == false){
    if (choice == "ss"){
      run_individual(individuals::smallRobotSmallControl);
    } else if (choice == "ls"){
      run_individual(individuals::largeRobotSmallControl);
    } else if (choice == "ll"){
      run_individual(individuals::largeRobotLargeControl);
    } else if (choice == "ms"){
      setLegLengths(0.0,0.0);
      fprintf(logOutput, "Small morphology requested\n");
    } else if (choice == "mm"){
      setLegLengths(12.5,47.5);
      fprintf(logOutput, "Medium morphology requested\n");
    } else if (choice == "ml"){
      setLegLengths(25.0,95.0);
      fprintf(logOutput, "Large morphology requested\n");
    }
  }
};

std::string getDateString(struct tm * givenTime){
  std::stringstream ss;

  ss << givenTime->tm_year+1900
     << std::setw(2) << std::setfill('0') << givenTime->tm_mon+1
     << std::setw(2) << std::setfill('0') << givenTime->tm_mday
     << std::setw(2) << std::setfill('0') << givenTime->tm_hour
     << std::setw(2) << std::setfill('0') << givenTime->tm_min
     << std::setw(2) << std::setfill('0') << givenTime->tm_sec;

  return ss.str();

}

std::string createExperimentDirectory(std::string prefix, struct tm * givenTime){
  std::stringstream ss;
  ss << getenv("HOME") << "/catkin_ws/experimentResults/";
  mkdir(ss.str().c_str(), 0700);
  ss.str(std::string());

  ss << getenv("HOME") << "/catkin_ws/experimentResults/" << getDateString(givenTime) << "_" << prefix << "/";

  mkdir(ss.str().c_str(), 0700);
  return ss.str();
}

void experiments_evolve(const std::string givenMorphology, bool evolveMorphology, bool givenAddDiversity){
  //assert(popSize == 8);
  addDiversity = givenAddDiversity;
  evolveMorph = evolveMorphology;
  morphology = givenMorphology;
  currentIndividual = -1;
  fitnessFunctions.clear();
  fitnessFunctions.emplace_back("MocapSpeed");
  fitnessFunctions.emplace_back("Stability");

  std::cout << "How many runs do you want to do? >";

  int numberOfTests;

  if (commandQueue.empty()) {
    std::cin >> numberOfTests;
    std::cin.ignore(10000, '\n');
  } else {
    numberOfTests = std::stoi(commandQueue[0]);
    fprintf(logOutput, "*%d*\n", numberOfTests);
    commandQueue.erase(commandQueue.begin());
  }

  for (int i = 0; i < numberOfTests; i++){
    currentIndividual = -1;

    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );

    std::string experimentDirectory = createExperimentDirectory("evo", now);

    std::stringstream ss;
    ss << experimentDirectory.c_str() << getDateString(now) << "_evo.json";

    evoLogPath = ss.str();

    FILE * evoLog = fopen(evoLogPath.c_str(), "a");
    if (evoLog == NULL){
      ROS_ERROR("evoLog could not be opened (err%d)\n", errno);
    }

    char hostname[1024];
    gethostname(hostname, 1024);

    fprintf(evoLog, "{\n");
    fprintf(evoLog, "  \"experiment_info\": {\n");
    fprintf(evoLog, "    \"time\": \"%s\",\n", getDateString(now).c_str());
    if (fullCommand.size() != 0) fprintf(evoLog, "    \"command\": \"%s\",\n", trim(fullCommand).c_str());
    fprintf(evoLog, "    \"type\": \"evolution\",\n");
    fprintf(evoLog, "    \"machine\": \"%s\",\n", hostname);
    fprintf(evoLog, "    \"user\": \"%s\",\n", getenv("USER"));

    if (ros::Time::isSimTime()) fprintf(evoLog, "    \"platform\": \"simulation\",\n"); else fprintf(evoLog, "    \"platform\": \"hardware\",\n");
    fprintf(evoLog, "    \"generations\": %d,\n", generations);
    fprintf(evoLog, "    \"population\": %d,\n", popSize);

    if (evolveMorphology) fprintf(evoLog, "    \"morphology\": \"*evolved*\",\n"); else fprintf(evoLog, "    \"morphology\": \"%s\",\n", givenMorphology.c_str());

    fprintf(evoLog, "    \"fitness\": [\n");
    if (instantFitness == false) {
      for (int i = 0; i < fitnessFunctions.size(); i++) {
        fprintf(evoLog, "      \"%s\"", fitnessFunctions[i].c_str());
        if (i != fitnessFunctions.size() - 1) fprintf(evoLog, ",\n");
      }
    } else fprintf(evoLog, "      \"*INSTANT*\"");
    if (givenAddDiversity) fprintf(evoLog, ", \"Diversity\"\n");
    fprintf(evoLog, "\n");
    fprintf(evoLog, "    ]\n");

    fprintf(evoLog, "  },\n");

    fprintf(evoLog, "  \"individuals\": [\n");

    fclose(evoLog);

    // Add directory command to argc and argv going into sferes:
    int argc_tmp = 2;
    char *argv_tmp[3];
    argv_tmp[0] = argv_g[0]; // Copy the first reference
    argv_tmp[2] = 0;

    mkdir(std::string(experimentDirectory + "sferes").c_str(), 0700);

    std::string commString = "-d" + experimentDirectory + "sferes";
    const char *sferesCommand = commString.c_str();
    argv_tmp[1] = const_cast<char*>(sferesCommand);

    fprintf(logOutput, "%s, %s\n", argv_tmp[0], argv_tmp[1]);

    run_ea(argc_tmp, argv_tmp, expGui::ea, evoLogPath);
    evoLogPath.clear();

    fprintf(logOutput, "Experiment finished. Log written to:\n  %s\n", ss.str().c_str());
  }

};

void experiments_verifyFitness(){
  currentIndividual = 0;

  std::cout << "How many individuals do you want to test? >";

  int numberOfTests;
  std::cin >> numberOfTests;
  std::cin.ignore(10000, '\n');

  time_t t = time(0);   // get time now
  struct tm * now = localtime( & t );

  std::string logDirectory = createExperimentDirectory("ver",now);

  std::stringstream ss;
  ss << logDirectory << getDateString(now) << "_ver.txt";

  std::string verifyLogPath = ss.str();

  FILE * verifyLog = fopen(verifyLogPath.c_str(), "a");
  if (verifyLog == NULL){
    ROS_ERROR("verifyLog couldnt be opened (err%d)\n", errno);
  }
  fclose(verifyLog);

  fitnessFunctions.clear();
  fitnessFunctions.emplace_back("MocapSpeed");
  fitnessFunctions.emplace_back("Stability");

  for (int i = 0; i < numberOfTests; i++) {

        std::vector<float> fitnessResult = evaluateIndividual(individuals::smallRobotSmallControl, false,
                                                          gaitControllerStatus_client, trajectoryMessage_pub,
                                                          get_gait_evaluation_client);

    fprintf(logOutput, "Returned fitness (%lu): ", fitnessResult.size());
    FILE * verifyLog = fopen(verifyLogPath.c_str(), "a");

    if (i > 0) fprintf(verifyLog, "\n");

    for (int j = 0; j < fitnessResult.size(); j++) {
      fprintf(logOutput, "%.2f ", fitnessResult[j]);

      if (j > 0) fprintf(verifyLog, ", ");
      fprintf(verifyLog, "%f",fitnessResult[j]);
    }

    fclose(verifyLog);
    fprintf(logOutput, "\n");

  }
}

void experiments_fitnessNoise(){
  for (int i = 0; i < 100; i++){
    resetGaitRecording(get_gait_evaluation_client);
    startGaitRecording(get_gait_evaluation_client);

    sleep(30);

    std::vector<float> gaitResults = getGaitResults(get_gait_evaluation_client);
    fprintf(logOutput, "  Res: ");
    for (int i = 0; i < gaitResults.size(); i++){
      fprintf(logOutput, "%.5f", gaitResults[i]);
      if (i != (gaitResults.size()-1)) fprintf(logOutput, ", "); else fprintf(logOutput, "\n");
    }
  }
}

std::vector<double> getRandomIndividual(){
  std::vector<double> individual(10);

  do{
    for (int j = 0; j < individual.size(); j++) individual[j] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  } while (((((individual[0]*300.0) * ((individual[6]*2.0)*60.0)) / 1000.0) > 10.0) // Speed has to be below 10m/min
          || ((individual[0]*300.0) < 5.0) // StepLength has to be above 5mm
          || ((individual[6]*2.0) < 0.1)); // Frequency has to be above 0.1

  return individual;
}

void experiments_randomSearch(){

  currentIndividual = 0;

  std::cout << "How many runs do you want to do? >";

  int numberOfTests;

  if (commandQueue.empty()) {
    std::cin >> numberOfTests;
    std::cin.ignore(10000, '\n');
  } else {
    numberOfTests = std::stoi(commandQueue[0]);
    fprintf(logOutput, "*%d*\n", numberOfTests);
    commandQueue.erase(commandQueue.begin());
  }

  fitnessFunctions.clear();
  fitnessFunctions.emplace_back("MocapSpeed");
  fitnessFunctions.emplace_back("Stability");

  for (int i = 0; i < numberOfTests; i++){

    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );

    std::string logDirectory = createExperimentDirectory("rand", now);

    std::stringstream ss;
    ss << logDirectory << getDateString(now) << "_rand.json";

    FILE * randomSearchLog = fopen(ss.str().c_str(), "a");
    if (randomSearchLog == NULL){
      ROS_ERROR("randomSearchLog couldnt be opened (err%d)\n", errno);
    }

    char hostname[1024];
    gethostname(hostname, 1024);

    fprintf(randomSearchLog, "{\n");
    fprintf(randomSearchLog, "  \"experiment_info\": {\n");
    fprintf(randomSearchLog, "    \"time\": \"%s\",\n", getDateString(now).c_str());
    fprintf(randomSearchLog, "    \"command\": \"%s\",\n", trim(fullCommand).c_str());
    fprintf(randomSearchLog, "    \"type\": \"random\",\n");
    fprintf(randomSearchLog, "    \"machine\": \"%s\",\n", hostname);
    fprintf(randomSearchLog, "    \"user\": \"%s\",\n", getenv("USER"));
    if (ros::Time::isSimTime()) fprintf(randomSearchLog, "    \"platform\": \"simulation\",\n"); else fprintf(randomSearchLog, "    \"platform\": \"hardware\",\n");
    fprintf(randomSearchLog, "    \"generations\": %d,\n", generations);
    fprintf(randomSearchLog, "    \"population\": %d,\n", popSize);

    fprintf(randomSearchLog, "    \"fitness\": [\n");
    if (instantFitness == false) {
      for (int i = 0; i < fitnessFunctions.size(); i++) {
        fprintf(randomSearchLog, "      \"%s\"", fitnessFunctions[i].c_str());
        if (i != fitnessFunctions.size() - 1) fprintf(randomSearchLog, ",\n");
      }
    } else fprintf(randomSearchLog, "      \"*INSTANT*\"");
    fprintf(randomSearchLog, "\n");
    fprintf(randomSearchLog, "    ]\n");

    fprintf(randomSearchLog, "  },\n");

    fprintf(randomSearchLog, "  \"individuals\": [\n");

    fclose(randomSearchLog);

    currentIndividual = 0;

    for (int j = 0; j < popSize*(generations); j++) {
      rawFitnesses.clear();

      // Generate random individual:
      std::vector<double> randomIndividual = getRandomIndividual();

      // Log individual
      randomSearchLog = fopen(ss.str().c_str(), "a");
      fprintf(randomSearchLog, "    {\n");
      fprintf(randomSearchLog, "      \"id\": %d,\n", currentIndividual);

      fprintf(randomSearchLog, "      \"genotype\": [\n");
      for (int k = 0; k < randomIndividual.size(); k++) {
        fprintf(randomSearchLog, "        %f", randomIndividual[k]);
        if (k == randomIndividual.size()-1) fprintf(randomSearchLog, "\n"); else fprintf(randomSearchLog, ",\n");
      }
      fprintf(randomSearchLog, "      ],\n");

      std::vector<double> individualParameters = genToPhen(randomIndividual);
      fprintf(randomSearchLog, "      \"phenotype\": [\n");
      for (int k = 0; k < individualParameters.size(); k++) {
        if (isnan(individualParameters[k])) fprintf(randomSearchLog, "        \"NaN\""); else fprintf(randomSearchLog, "        %.2f", individualParameters[k]);
        if (k == individualParameters.size()-1) fprintf(randomSearchLog, "\n"); else fprintf(randomSearchLog, ",\n");
      }
      fprintf(randomSearchLog, "      ],\n");

      fclose(randomSearchLog);

      std::vector<float> fitnessResult = evaluateIndividual(genToPhen(randomIndividual), false,
                                                            gaitControllerStatus_client, trajectoryMessage_pub,
                                                            get_gait_evaluation_client);

      fprintf(logOutput, "Returned fitness (%lu): ", fitnessResult.size());

      randomSearchLog = fopen(ss.str().c_str(), "a");
      fprintf(randomSearchLog, "      \"fitness\": {\n");

      for (int k = 0; k < fitnessResult.size(); k++) {
        fprintf(randomSearchLog, "        \"%s\": %.3f", fitnessFunctions[k].c_str(), fitnessResult[k]);
        if (k == fitnessResult.size()-1) fprintf(randomSearchLog, "\n"); else fprintf(randomSearchLog, ",\n");

        fprintf(logOutput, "%.2f ",fitnessResult[k]);

      }
      fprintf(logOutput, "\n");

      fprintf(randomSearchLog, "      },\n");

      fprintf(randomSearchLog, "      \"raw_fitness\": [\n");

      for (int k = 0; k < rawFitnesses.size(); k++) {
        fprintf(randomSearchLog, "%s", rawFitnesses[k].c_str());
        if (k == fitnessResult.size()-1) fprintf(randomSearchLog, "\n"); else fprintf(randomSearchLog, ",\n");

      }
      fprintf(randomSearchLog, "      ]\n");

      if (currentIndividual == ((generations)*popSize)){ // If last individual
        fprintf(randomSearchLog, "    }\n");
        fprintf(randomSearchLog, "  ]\n");
        fprintf(randomSearchLog, "}");
      } else {
        fprintf(randomSearchLog, "    },\n");
      }



      fclose(randomSearchLog);
    }

    fprintf(logOutput, "Experiment finished. Log written to:\n  %s\n", ss.str().c_str());
  }
}

void menu_experiments() {
  std::string choice;

  std::cout << "  Please choose one experiment: (enter to go back)\n";

  fprintf(logOutput, "    cs - evolve control, small morphology\n"
         "    cm - evolve control, medium morphology\n"
         "    cl - evolve control, large morphology\n"
         "    my - evolve cont+morph, with diversity\n"
         "    mn - evolve cont+morph, w/o diversity\n"
         "    ra - random search\n"
         "    vf - verify fitness on single individual\n"
         "    vn - check noise in stability fitness\n");
  fprintf(logOutput, "\n> ");

  if (commandQueue.empty()) {
    getline(std::cin, choice);
  } else {
    choice = commandQueue[0];
    fprintf(logOutput, "*%s*\n", choice.c_str());
    commandQueue.erase(commandQueue.begin());
  }

  if (choice.empty() != true){
    if (choice == "cs") {
      experiments_evolve("small", false, false);
    } else if (choice == "cm") {
      experiments_evolve("medium", false, false);
    } else if (choice == "cl") {
      experiments_evolve("large", false, false);
    } else if (choice == "my") {
      experiments_evolve("", true, true);
    } else if (choice == "mn") {
      experiments_evolve("", true, false);
    } else if (choice == "vf"){
      experiments_verifyFitness();
    } else if (choice == "vn"){
      experiments_fitnessNoise();
    } else if (choice ==  "ra"){
      experiments_randomSearch();
    }
  }
}

void menu_configure() {
  std::string choice;

  std::cout << "  Please choose a setting to change: (enter to go back)\n";

  fprintf(logOutput, "    i - enable/disable instant fitness\n");
  fprintf(logOutput, "    s - enable/disable stand testing\n");
  fprintf(logOutput, "    e - enable servo torques\n");
  fprintf(logOutput, "    d - disable servo torques\n");
  fprintf(logOutput, "\n> ");

  if (commandQueue.empty()) {
    getline(std::cin, choice);
  } else {
    choice = commandQueue[0];
    fprintf(logOutput, "*%s*\n", choice.c_str());
    commandQueue.erase(commandQueue.begin());
  }

  if (choice.empty() != true) {
    if (choice == "i") {
      instantFitness = !instantFitness;
      if (instantFitness == true) fprintf(logOutput, "Instant fitness evaluation now enabled!\n");
      else
        fprintf(logOutput, "Instant fitness evaluation now disabled!\n");
    } else if (choice == "s"){
      if (robotOnStand == true){
        fprintf(logOutput, "   RobotOnStand now disabled!\n");
      } else {
        fprintf(logOutput, "   RobotOnStand now enabled!\n");
      }

      robotOnStand = !robotOnStand;
    } else if (choice == "e"){
      enableServos();
      fprintf(logOutput, "Servos enabled!\n");
    } else if (choice == "d"){
      disableServos();
      fprintf(logOutput, "Servos disabled!\n");
    }
  }
  fprintf(logOutput, "\n");
}

void testInverseKinematics(){
  fprintf(logOutput, "Testing inverse kinematics:\n");

  std::vector<float> position;

  testLegPositionAll(std::vector<float>{ 0.0f,  0.0f, -450.0f});
  sleep(3);
  testLegPositionAll(std::vector<float>{80.0f,  0.0f, -450.0f}); // Test X
  sleep(3);
  testLegPositionAll(std::vector<float>{ 0.0f,  0.0f, -450.0f});
  sleep(3);
  testLegPositionAll(std::vector<float>{ 0.0f, 80.0f, -450.0f}); // Test Y
  sleep(3);
  testLegPositionAll(std::vector<float>{ 0.0f,  0.0f, -450.0f});
  sleep(3);
  testLegPositionAll(std::vector<float>{ 0.0f,  0.0f, -420.0f}); // Test Z
  sleep(3);
  testLegPositionAll(std::vector<float>{ 0.0f,  0.0f, -450.0f});
  sleep(3);
  setLegPositions(std::vector<float>{-50.0f,  50.0f, -450.0f,
                                     0.0f,   0.0f, -450.0f,
                                     0.0f,   0.0f, -450.0f,
                                     0.0f,   0.0f, -450.0f});
  sleep(3);
  testLegPositionAll(std::vector<float>{ 0.0f,  0.0f, -450.0f});
  sleep(3);
  setLegPositions(std::vector<float>{  0.0f,   0.0f, -450.0f,
                                       50.0f,  50.0f, -450.0f,
                                       0.0f,   0.0f, -450.0f,
                                       0.0f,   0.0f, -450.0f});
  sleep(3);
  testLegPositionAll(std::vector<float>{ 0.0f,  0.0f, -450.0f});
  sleep(3);
}

void menu_test(){
  std::string choice;
  for(;;) {
    std::cout << "  Please choose one test: (enter to go back)\n";

    fprintf(logOutput, "    inv - test inverseKinematics\n");
    fprintf(logOutput, "\n> ");

    if (commandQueue.empty()) {
      getline(std::cin, choice);
    } else {
      choice = commandQueue[0];
      fprintf(logOutput, "*%s*\n", choice.c_str());
      commandQueue.erase(commandQueue.begin());
    }

    if (choice.empty() == true){
      break;
    } else {
      if (choice == "inv"){
        testInverseKinematics();
      }
    }
  }
};

int main(int argc, char **argv){

  argv_g = argv;

  fullCommand = "";
  if (argc > 1){
    for (int i = 1; i < argc; i++) {
      commandQueue.emplace_back(argv[i]);
      fullCommand.append(argv[i]);
      fullCommand.append(" ");
    }
  }

  FILE *fp = fopen("generation", "w");
  fprintf(fp,"%d",0);
  fclose(fp);

  fitnessFunctions.emplace_back("Speed");
  fitnessFunctions.emplace_back("Efficiency");

  ros::init(argc, argv, "exp2Gui");
  ros::NodeHandle rch;

  actionMessages_pub = rch.advertise<dyret_controller::ActionMessage>("/dyret/dyret_controller/actionMessages", 10);
  positionCommand_pub = rch.advertise<dyret_controller::PositionCommand>("/dyret/dyret_controller/positionCommand", 1);

  servoConfigClient = rch.serviceClient<dyret_common::Configure>("/dyret/configuration");
  get_gait_evaluation_client = rch.serviceClient<dyret_controller::GetGaitEvaluation>("get_gait_evaluation");
  gaitControllerStatus_client = rch.serviceClient<dyret_controller::GetGaitControllerStatus>("get_gait_controller_status");
  trajectoryMessage_pub = rch.advertise<dyret_controller::Trajectory>("/dyret/dyret_controller/trajectoryMessages", 1000);
  poseCommand_pub = rch.advertise<dyret_common::Pose>("/dyret/command", 10);
  dyretState_sub = rch.subscribe("/dyret/state", 1, dyretStateCallback);

  waitForRosInit(get_gait_evaluation_client, "get_gait_evaluation");
  waitForRosInit(gaitControllerStatus_client, "gaitControllerStatus");
  waitForRosInit(trajectoryMessage_pub, "/dyret/dyret_controller/trajectoryMessage");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string evoInfo = "testInfo";

  resetTrajectoryPos(trajectoryMessage_pub);
  if (resetGaitRecording(get_gait_evaluation_client) == false){
      spinner.stop();
      ros::shutdown();
      exit(0);
  }

  addDiversity = true;
  currentIndividual = -1;

  if (ros::Time::isSimTime()){
    fprintf(logOutput, "Currently running in simulation mode\n");
  } else {
    fprintf(logOutput, "Currently running in hardware mode\n");
  }

  std::map< std::string, boost::function<void()> > menu;
  menu["demo"] = &menu_demo;
  menu["experiments"] = &menu_experiments;
  menu["configure"] = &menu_configure;
  menu["test"] = &menu_test;

  std::string choice;
  for(;;) {
    std::cout << "Please choose: (enter to quit)\n";
    std::map<std::string, boost::function<void()> >::iterator it = menu.begin();
    while(it != menu.end()) {
      std::cout << "  " << (it++)->first << std::endl;
    }
    fprintf(logOutput, "\n> ");

    if (commandQueue.empty() && !automatedRun()) {
      getline(std::cin, choice);
    } else if (commandQueue.empty() && automatedRun) {
      choice = "exit";
    } else {
      choice = commandQueue[0];
      fprintf(logOutput, "*%s*\n", choice.c_str());
      commandQueue.erase(commandQueue.begin());
    }

    if (choice.empty() == true || choice == "exit"){
      spinner.stop();
      ros::shutdown();
      exit(0);
    } else if(menu.find(choice) == menu.end()) {
      fprintf(logOutput, "Unknown choice!\n");
      continue;
    }

    menu[choice]();
  }

}
