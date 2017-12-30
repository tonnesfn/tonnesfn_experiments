#include <iostream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <algorithm>

#include "ros/ros.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include "dyret_common/GetGaitControllerStatus.h"
#include "dyret_common/ActionMessage.h"
#include "dyret_common/Trajectory.h"
#include "dyret_common/Configuration.h"
#include "dyret_common/GetGaitEvaluation.h"
#include "dyret_common/GetServoStatuses.h"
#include "dyret_common/ServoConfig.h"
#include "dyret_common/ServoConfigArray.h"

#include "dyret_utils/timeHandling.h"
#include "dyret_utils/wait_for_ros.h"

#include "external/sferes/phen/parameters.hpp"
#include "external/sferes/gen/evo_float.hpp"
#include "external/sferes/ea/nsga2.hpp"
#include "external/sferes/eval/eval.hpp"
#include "external/sferes/stat/pareto_front.hpp"
#include "external/sferes/modif/dummy.hpp"
#include "external/sferes/run.hpp"
#include <boost/program_options.hpp>

#include "rosConnection.h"

#include <sstream>
#include <iostream>

ros::ServiceClient get_gait_evaluation_client;
ros::Publisher trajectoryMessage_pub;
ros::Publisher actuatorCommand_pub;
ros::ServiceClient gaitControllerStatus_client;
ros::Subscriber actuatorState_sub;
ros::ServiceClient servoStatus_client;
ros::Publisher servoConfig_pub;
ros::Publisher actionMessages_pub;

FILE * evoFitnessLog;
FILE * evoParamLog_gen;
FILE * evoParamLog_phen;
double globalSpeed;
float currentFemurLength = 0.0;
float currentTibiaLength = 0.0;
int currentIndividual;

const int individuals = 8;
const int generations = 18;

bool evolveMorph = true;
std::string morphology;

rosConnectionHandler_t* rch;

bool robotOnStand = false;

std::vector<std::string> fitnessFunctions;

FILE* getEvoPathFileHandle(std::string fileName, std::string givenHeader = ""){
  std::string line;
  std::ifstream myfile ("currentEvoDir");
  if (myfile.is_open()) {
    getline (myfile,line);
    myfile.close();
  } else {
      printf("Unable to open currentEvoDir file");
      return fopen(fileName.c_str(), "w+");
  }

  line.append("/");
  line.append(fileName.c_str());

  std::ifstream infile(line.c_str());

  FILE * handleToReturn = fopen(line.c_str(), "a+");

  if (!infile.good() && !givenHeader.empty()){
    fprintf(handleToReturn, "%s\n", givenHeader.c_str());
  }

  return handleToReturn;
}

void sendServoTorqueMessage(int givenValue){
  dyret_common::ServoConfigArray msg;
  std::vector<dyret_common::ServoConfig> msgContents(12);

  for (int i = 0; i < 12; i++) {
    msgContents[i].servoId = i;

    if (givenValue == 0) msgContents[i].configType = dyret_common::ServoConfig::t_disableTorque;
    if (givenValue == 1) msgContents[i].configType = dyret_common::ServoConfig::t_enableTorque;
  }

  msg.servoConfigs = msgContents;
  servoConfig_pub.publish(msg);

}

void sendActionMessage(bool sleep){
  dyret_common::ActionMessage actionMessage;
  actionMessage.configuration = dyret_common::ActionMessage::t_mammal;
  if (sleep == true) actionMessage.actionType = dyret_common::ActionMessage::t_sleep; else actionMessage.actionType = dyret_common::ActionMessage::t_restPose;
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

void startGaitRecording(ros::ServiceClient get_gait_evaluation_client){
  dyret_common::GetGaitEvaluation srv;
  srv.request.givenCommand = dyret_common::GetGaitEvaluation::Request::t_start;
  if (!get_gait_evaluation_client.call(srv)) printf("Error while calling GaitRecording service with t_start\n");
}

void resetGaitRecording(ros::ServiceClient get_gait_evaluation_client){
  dyret_common::GetGaitEvaluation srv;
  srv.request.givenCommand = dyret_common::GetGaitEvaluation::Request::t_resetStatistics;
  if (!get_gait_evaluation_client.call(srv)) printf("Error while calling GaitRecording service with t_resetStatistics\n");
}

std::vector<float> getGaitResults(ros::ServiceClient get_gait_evaluation_client){
  dyret_common::GetGaitEvaluation srv;
  std::vector<float> vectorToReturn;

  srv.request.givenCommand = dyret_common::GetGaitEvaluation::Request::t_getResults;

  if (get_gait_evaluation_client.call(srv)) {
    vectorToReturn = srv.response.results;
  } else {
      printf("Error while calling GaitRecording service with t_getResults!\n");
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
  dyret_common::GetGaitControllerStatus srv;

  if (gaitControllerStatus_client.call(srv)) {
      if (srv.response.gaitControllerStatus.actionType == srv.response.gaitControllerStatus.t_idle) return true;
  }

  return false;
}

void resetTrajectoryPos(ros::Publisher givenTrajectoryMsgPublisher){
  dyret_common::Trajectory msg;
  msg.command = msg.t_resetPosition;
  givenTrajectoryMsgPublisher.publish(msg);
}

void sendTrajectories(std::vector<float> givenTrajectoryDistances, std::vector<float> givenTrajectoryAngles, std::vector<int> givenTrajectoryTimeouts, ros::Publisher givenTrajectoryMsgPublisher){
  dyret_common::Trajectory msg;

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
  dyret_common::Configuration msg;

  msg.id.resize(0);
  msg.distance.resize(2);

  msg.distance[0] = femurLengths;
  msg.distance[1] = tibiaLengths;

  waitForRosInit(actuatorCommand_pub, "actuatorCommand_pub");

  actuatorCommand_pub.publish(msg);
}

void actuatorStateCallback(const dyret_common::Configuration::ConstPtr& msg) {
  if (msg->distance.size() == !8){ ROS_ERROR("Distance array length is wrong, it is %lu!", msg->distance.size()); }

  currentFemurLength = (msg->distance[0] + msg->distance[2] + msg->distance[4] + msg->distance[6]) / 4.0;
  currentTibiaLength = (msg->distance[1] + msg->distance[3] + msg->distance[5] + msg->distance[7]) / 4.0;
}

bool legsAreLength(float femurLengths, float tibiaLengths){
  if ( (fabs(femurLengths - currentFemurLength) < 0.5) && (fabs(tibiaLengths - currentTibiaLength) < 0.5) ){
    return true;
  } else {
    return false;
  }
}

float getMaxServoTemperature(bool printAllTemperatures = false){
  float maxTemp = -1.0;

  dyret_common::GetServoStatuses  gssres;
  servoStatus_client.call(gssres);

  if (printAllTemperatures) printf("Servo temperatures: ");
  for (int i = 0; i < gssres.response.servoStatuses.size(); i++){
    if (gssres.response.servoStatuses[i].temperature > maxTemp) maxTemp = gssres.response.servoStatuses[i].temperature;
    if (printAllTemperatures) printf("%.2f ",gssres.response.servoStatuses[i].temperature);
  }
  if (printAllTemperatures) printf("\n");

  return maxTemp;
}

std::vector<float> evaluateIndividual(std::vector<double> phenoType,
                                      std::string* fitnessString,
                                      bool speedAspectLocked,
                                      ros::ServiceClient gaitControllerStatus_client,
                                      ros::Publisher trajectoryMessage_pub,
                                      ros::ServiceClient get_gait_evaluation_client) {

  currentIndividual++;

  // (Return empty fitness to test)
  //return std::vector<float>{static_cast <float> (rand()) / static_cast <float> (RAND_MAX), static_cast <float> (rand()) / static_cast <float> (RAND_MAX)};

  if (currentIndividual == individuals){
    currentIndividual = 0;
    getMaxServoTemperature(true);
    printf("Cooldown to 50C? (y/n) > ");

    char choice;
    scanf(" %c", &choice);

    if (choice == 'y'){
      disableServos();
      while (getMaxServoTemperature(true) > 50){ sleep(10); }

      std::cout << "Press enter to enable servos";
      std::cin.ignore();
      std::cin.ignore();

      enableServos();

      std::cout << "Press enter to continue evolution";
      std::cin.ignore();
    }

  }

  printf("%03u: Evaluating stepLength %.2f, "
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

  // Check temperature - if its over the limit below, consider fitness invalid (due to DC motor characterics)
  if (getMaxServoTemperature() > 70.0){
    printf("  Temperature is too high at %.1f\n", getMaxServoTemperature());
    return std::vector<float>();
  }

  setGaitParams(phenoType[0], phenoType[1], phenoType[2], phenoType[3], phenoType[4], phenoType[5], phenoType[6], phenoType[7], phenoType[10]);

  setLegLengths(phenoType[8], phenoType[9]);
  int secPassed = 0;
  while (!legsAreLength(phenoType[8], phenoType[9])) {
    sleep(1);
    if (secPassed++ > 120) return std::vector<float>(); // 2 min timeout
  }

  std::vector<float> trajectoryAngles(1);
  std::vector<float> trajectoryDistances(1);
  std::vector<int> trajectoryTimeouts(1);
  trajectoryDistances[0] = 1500.0;
  trajectoryTimeouts[0] = 10.0; // 10 sec timeout

  resetTrajectoryPos(trajectoryMessage_pub); // Reset position before starting
  resetGaitRecording(get_gait_evaluation_client);
  sendTrajectories(trajectoryDistances, trajectoryAngles, trajectoryTimeouts, trajectoryMessage_pub);
  sleep(5);

  secPassed = 0;
  while (gaitControllerDone(gaitControllerStatus_client) == false) {
    sleep(1);
    if (secPassed++ > 30) {  // 30 sec timeout
      return std::vector<float>();
    }
  }

  std::vector<float> gaitResultsForward = getGaitResults(get_gait_evaluation_client);

  if (gaitResultsForward.size() == 0) {
    fprintf(stderr, "GaitResultsForward.size() == 0!\n");
    return gaitResultsForward;
  }

  printf("\tRes F: ");
  for (int i = 0; i < gaitResultsForward.size(); i++) {
    printf("%.5f", gaitResultsForward[i]);
    if (i != (gaitResultsForward.size() - 1)) printf(", "); else printf("\n");
  }

  trajectoryDistances[0] = 0.0;
  trajectoryTimeouts[0] = 10.0; // 10 sec timeout

  //resetTrajectoryPos(trajectoryMessage_pub); // Reset position before starting
  resetGaitRecording(get_gait_evaluation_client);
  sendTrajectories(trajectoryDistances, trajectoryAngles, trajectoryTimeouts, trajectoryMessage_pub);
  sleep(5);

  secPassed = 0;
  while (gaitControllerDone(gaitControllerStatus_client) == false) {
    sleep(1);
    if (secPassed++ > 35) return std::vector<float>(); // 35 sec timeout
  }

  std::vector<float> gaitResultsReverse = getGaitResults(get_gait_evaluation_client);

  if (gaitResultsReverse.size() == 0) {
    fprintf(stderr, "GaitResultsReverse.size() == 0!\n");
    return gaitResultsReverse;
  }

  printf("\tRes R: ");
  for (int i = 0; i < gaitResultsReverse.size(); i++) {
    printf("%.5f", gaitResultsReverse[i]);
    if (i != (gaitResultsReverse.size() - 1)) printf(", "); else printf("\n");
  }


  std::vector<float> fitness(fitnessFunctions.size());
  int currentFitnessIndex = 0;

  float fitness_inferredSpeed = (gaitResultsForward[0] + gaitResultsReverse[0]) / 2.0;
  float fitness_current = (gaitResultsForward[4] + gaitResultsReverse[4]) / 2.0;
  float fitness_stability = (gaitResultsForward[6] + gaitResultsReverse[6]) / 2.0;
  float fitness_mocapSpeed = (gaitResultsForward[5] + gaitResultsReverse[5]) / 2.0;
/*
  float fitness_inferredSpeed = gaitResultsForward[0];
  float fitness_current = gaitResultsForward[4];
  float fitness_stability = gaitResultsForward[3] + (gaitResultsForward[2] / 50.0);
  float fitness_mocapSpeed = gaitResultsForward[5];
*/

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

  std::stringstream ss;

  ss << currentIndividual << ",";

  for (int i = 0; i < gaitResultsForward.size(); i++) {
    ss << gaitResultsForward[i] << "," << gaitResultsReverse[i] << ",";
    //ss << gaitResultsForward[i] << ",";
  }

  ss << fitness_mocapSpeed << "," << fitness_stability << "," << fitness_current;
  *fitnessString = ss.str();

  return fitness;

}

void rosConnect(){
  int argc = 0;
  char **argv;

  if (rch != NULL) {
    delete rch;

    get_gait_evaluation_client.shutdown();
    gaitControllerStatus_client.shutdown();
    trajectoryMessage_pub.shutdown();
    actuatorCommand_pub.shutdown();
    actuatorState_sub.shutdown();
  }

  rch = new rosConnectionHandler_t(argc, argv);

  actionMessages_pub = rch->nodeHandle()->advertise<dyret_common::ActionMessage>("actionMessages", 10);
  servoStatus_client = rch->nodeHandle()->serviceClient<dyret_common::GetServoStatuses>("/dyret/servoStatuses");

  servoConfig_pub = rch->nodeHandle()->advertise<dyret_common::ServoConfigArray>("/dyret/servoConfigs", 1);
  get_gait_evaluation_client = rch->nodeHandle()->serviceClient<dyret_common::GetGaitEvaluation>("get_gait_evaluation");
  gaitControllerStatus_client = rch->nodeHandle()->serviceClient<dyret_common::GetGaitControllerStatus>("get_gait_controller_status");
  trajectoryMessage_pub = rch->nodeHandle()->advertise<dyret_common::Trajectory>("trajectoryMessages", 1000);
  actuatorCommand_pub = rch->nodeHandle()->advertise<dyret_common::Configuration>("actuatorCommands", 10);
  actuatorState_sub = rch->nodeHandle()->subscribe("/actuatorStates", 1, actuatorStateCallback);

  waitForRosInit(get_gait_evaluation_client, "get_gait_evaluation");
  waitForRosInit(gaitControllerStatus_client, "gaitControllerStatus");
  waitForRosInit(trajectoryMessage_pub, "trajectoryMessage");

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
    SFERES_CONST unsigned size     =   individuals;  // Population size
    SFERES_CONST unsigned nb_gen   =   generations;  // Number of generations
    SFERES_CONST int dump_period   =    1;  // How often to save
    SFERES_CONST int initial_aleat =    1;  // Individuals to be created during random generation process
  };
  struct parameters {
    SFERES_CONST float min = 0.0f;
    SFERES_CONST float max = 1.0f;
  };
};

std::vector<double> genToPhen(std::vector<double> givenGenotype){

  std::vector<double> phenotype = {50.0 + (givenGenotype[0] * 100.0), // 0: stepLength        50 -> 150
                                   25.0 + (givenGenotype[1] * 50.0),  // 1: stepHeight        25 -> 75
                                   givenGenotype[2] * 50.0,           // 2: smoothing          0 -> 50
                                   givenGenotype[6],                  // 6: frequency          0 -> 1
                                   NAN,                               // 6: speed
                                   (givenGenotype[3] * 0.4) - 0.2,    // 3: wagPhase        -0.2 -> 0.2
                                   givenGenotype[4] * 50.0,           // 4: wagAmplitude_x     0 -> 50
                                   givenGenotype[5] * 50.0,           // 5: wagAmplitude_y     0 -> 50
                                   givenGenotype[7] * 25.0,           // 7: femurLength        0 -> 25
                                   givenGenotype[8] * 50.0,           // 8: tibiaLength        0 -> 50
                                   (givenGenotype[9] * 0.15) + 0.05   // 9: liftDuration    0.05 -> 0.20
  };

  return phenotype;
}

std::vector<double> phenToGen(std::vector<double> givenFenotype){

  std::vector<double> genotype = {(givenFenotype[0] - 50) / 100.0,   // 0: stepLength        50 -> 150
                                  (givenFenotype[1] - 25) / 50.0,    // 1: stepHeight        25 -> 75
                                  givenFenotype[2] / 50.0,           // 2: smoothing          0 -> 50
                                  (givenFenotype[5] + 0.2) / 0.4,    // 3: wagPhase        -0.2 -> 0.2
                                  givenFenotype[6] / 50.0,           // 4: wagAmplitude_x     0 -> 50
                                  givenFenotype[7] / 50.0,           // 5: wagAmplitude_y     0 -> 50
                                  givenFenotype[3],                  // 6: frequency          0 -> 1
                                  givenFenotype[8] / 25.0,           // 7: femurLength        0 -> 25
                                  givenFenotype[9] / 50.0,           // 8: tibiaLength        0 -> 50
                                  (givenFenotype[10] - 0.05) / 0.15  // 9: liftDuration    0.05 -> 0.20
  };

  return genotype;

}


SFERES_FITNESS(FitExp2MO, sferes::fit::Fitness) {
public:
  FitExp2MO()  {}
  template<typename Indiv>
  void eval(Indiv& ind) {

    // Only add diversity if we are evolving morphology
    if (evolveMorph == true) {
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
      printf("%.2f ", ind.gen().data(i));
    }
    printf("\n");

    std::vector<double> individualParameters = genToPhen(individualData);

    std::string fitnessDescription_gen  = "id,stepLength,stepHeight,smoothing,wagPhase,wagAmplitude_x,wagAmplitude_y,frequency,femurLength,tibiaLength,liftDuration";
    std::string fitnessDescription_phen = "id,stepLength,stepHeight,smoothing,frequency,speed,wagPhase,wagAmplitude_x,wagAmplitude_y,femurLength,tibiaLength,liftDuration";

    bool validSolution;
    std::vector<float> fitnessResult;

    std::string fitnessString;

    do{
        validSolution = true;
        fitnessResult = evaluateIndividual(individualParameters, &fitnessString, false, gaitControllerStatus_client, trajectoryMessage_pub, get_gait_evaluation_client);

        printf("  Fitness received: ");
        for (int i = 0; i < fitnessResult.size(); i++) printf("%.2f ", fitnessResult[i]);
        printf("\n");

        for (int i = 0; i < fitnessResult.size(); i++){
            if (std::isnan(fitnessResult[i]) == true){
                validSolution = false;
            }
        }

        if (fitnessResult.size() == 0) validSolution = false;

        if ((validSolution == false) || (fitnessResult.size() == 0)){
          printf("Got invalid fitness: choose action ((r)etry/(d)iscard/r(e)connect/(c)ooldown): ");

          char choice;
          scanf(" %c", &choice);

          if (choice == 'd'){ // Discard
              fitnessResult.resize(fitnessFunctions.size());
              for (int i = 0; i < fitnessResult.size(); i++) fitnessResult[i] = -1000;

              printf("Discarding\n");
              validSolution = true;
          } else if (choice == 'e'){
            rosConnect();
            printf("Reconnected and retrying\n");
            sleep(3);
            currentIndividual--;
            validSolution = false;
          } else if (choice == 'c'){
            disableServos();
            printf("Servos disabled\n");

            while (getMaxServoTemperature(true) > 50){ sleep(15); }

            enableServos();

            std::cout << "Press enter to continue evolution";
            std::cin.ignore();
            std::cin.ignore();

            currentIndividual--;
            validSolution = false;
          } else {
              printf("Retrying\n");
              currentIndividual--;
              validSolution = false;
          }
        }

    } while (validSolution == false);

    for (int i = 0; i < fitnessResult.size(); i++){
        this->_objs[i] = fitnessResult[i];
    }

    // Do logging:
    if (evoParamLog_gen == NULL){
      evoParamLog_gen = getEvoPathFileHandle("evoParamLog_gen.csv", fitnessDescription_gen);
    }

    fprintf(evoParamLog_gen, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                currentIndividual,
                ind.data(0), ind.data(1), ind.data(2), ind.data(3),
                ind.data(4), ind.data(5), ind.data(6), ind.data(7),
                ind.data(8), ind.data(9));
    fflush(evoParamLog_gen);

    if (evoParamLog_phen == NULL){
        evoParamLog_phen = getEvoPathFileHandle("evoParamLog_phen.csv", fitnessDescription_phen);
    }

    fprintf(evoParamLog_phen, "%d,", currentIndividual);
    for (int i = 0; i < individualParameters.size(); i++) if(i != (individualParameters.size()-1)) fprintf(evoParamLog_phen, "%f,", individualParameters[i]); else fprintf(evoParamLog_phen, "%f\n", individualParameters[i]);
    fflush(evoParamLog_phen);

    std::string fitnessDescription = "Id,"
                                     "Speed_I (F),Speed_I (R),"
                                     "angVel (F),angVel (R),"
                                     "linAcc (F),linAcc(R),"
                                     "orientation (F),orientation(R),"
                                     "efficiency (F),efficiency (R),"
                                     "speed_m (F),speed_m (R),"
                                     "stability (F),stability (R),"
                                     "speed_m (T),"
                                     "stability (T),"
                                     "current (T)";
    //std::string fitnessDescription = "Id, Speed_I, angVel, linAcc, stability, efficiency, speed_m, speed_m (T), stability (T), current (T)";

    if (evoFitnessLog == NULL){
        evoFitnessLog = getEvoPathFileHandle("evoFitnessLog.csv", fitnessDescription);
    }

    fprintf(evoFitnessLog, "%s\n", fitnessString.c_str());
    fflush(evoFitnessLog);
  }
};

std::string getEvoInfoString(){

  std::ostringstream stringStream;
  stringStream.precision(2);

  stringStream << ("  Fitness: ");
  for (int i = 0; i < fitnessFunctions.size(); i++) stringStream << fitnessFunctions[i].c_str();
  stringStream << "\n";

  stringStream << "    Pop: " << Params::pop::size << ", Gen: " << Params::pop::nb_gen << "\n";
  if (Params::evo_float::mutation_type == gaussian) stringStream << "  Mutation type: Gaussian\n"; else stringStream << "Mutation type: Unknown\n";
  stringStream << "      Mut_p: " <<  Params::evo_float::mutation_rate << ", Mut_a: " << Params::evo_float::sigma << "\n";
  if (Params::evo_float::cross_over_type == recombination) stringStream << "  Crossover type: Recombination\n"; else stringStream << "Crossover type: Unknown\n";
  stringStream << "      C_p: " << Params::evo_float::cross_rate << "\n\n";

  printf("%s", stringStream.str().c_str());

  return stringStream.str();
}

void resetEvoDir(){
  if(remove( "currentevodir" ) != 0) printf("Removed currentevodir file\n");
}

int main(int argc, char **argv){

  resetEvoDir();

  fitnessFunctions.emplace_back("Speed");
  fitnessFunctions.emplace_back("Efficiency");

  globalSpeed = 5.0;

  evoFitnessLog = NULL;
  evoParamLog_gen = NULL;
  evoParamLog_phen = NULL;

  rosConnect();

  ros::AsyncSpinner spinner(2);
  spinner.start();

  typedef gen::EvoFloat<10, Params> gen_t;   // Number of parameters in each individual:
  typedef phen::Parameters<gen_t, FitExp2MO<Params>, Params> phen_t;
  typedef eval::Eval<Params> eval_t;
  typedef boost::fusion::vector<stat::State<phen_t, Params> >  stat_t;
  typedef modif::Dummy<> modifier_t;
  typedef ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
  ea_t ea;

  std::string evoInfo = "testInfo";

  resetTrajectoryPos(trajectoryMessage_pub);
  resetGaitRecording(get_gait_evaluation_client);

  int inputChar;
  do {
    printf("1 - Evo control - small\n"
           "2 - Evo control - medium\n"
           "3 - Evo control - large\n"
           "4 - CO-evo morph+cont\n"
           "s - Enable/disable stand testing\n"
           "v - Verify fitness\n"
           "n - Test fitness noise\n"
           "e - Enable servos\n"
           "d - Disable servos\n"
           "r - Reconnect to master\n"
           "0 - Exit\n> ");

    currentIndividual = individuals-1;

    inputChar = getchar();
    std::cin.ignore(1000,'\n');

    switch(inputChar){
      // Enable/disable stand testing:
      case 's':
        if (robotOnStand == true){
          printf("   RobotOnStand now disabled!\n");
        } else {
          printf("   RobotOnStand now enabled!\n");
        }

        robotOnStand = !robotOnStand;
        break;

      // Verify fitness:
      case 'v':
      {
        for (int i = 0; i < 1; i++){

          /*
           0.01112037897, 0.7483010888, 0.8399485350, 0.9170328379, 0.2148616016, 0.8183637857, 0.8674162626, 0.3620291352, 0.03699165583

           <item>-3.215271831e-01</item>
           <item>8.403041840e+00</item>
           <item>1.645827740e-01</item>
          */

          std::vector<double> givenIndividual = {0.01112037897,
                                                 0.7483010888,
                                                 0.8399485350,
                                                 0.9170328379,
                                                 0.2148616016,
                                                 0.8183637857,
                                                 0.4,
                                                 0.3620291352,
                                                 0.03699165583,
                                                 0.5};
          //std::vector<double> givenIndividual = {0.01112037897, 0.7483010888, 0.8399485350, 0.9170328379, 0.2148616016, 0.8183637857, 0.2674162626, 0.3620291352, 0.03699165583};

          std::vector<double> givenInd_phen = genToPhen(givenIndividual);
          std::vector<double> givenInd_gen = phenToGen(givenInd_phen);

          printf("  GivenIndividual: ");
          for (int i = 0; i < givenIndividual.size(); i++){
            printf("%.4f, ", givenIndividual[i]);
          }
          printf("\n");

          printf("  givenInd_phen: ");
          for (int i = 0; i < givenInd_phen.size(); i++){
            printf("%.4f, ", givenInd_phen[i]);
          }
          printf("\n");

          fitnessFunctions.clear();
          fitnessFunctions.emplace_back("MocapSpeed");
          fitnessFunctions.emplace_back("Stability");

          for (int i = 0; i < 10; i++) {

            std::string fitnessString;
            std::vector<float> fitnessResult = evaluateIndividual(genToPhen(givenIndividual), &fitnessString, false,
                                                                  gaitControllerStatus_client, trajectoryMessage_pub,
                                                                  get_gait_evaluation_client);
            printf("%s\n", fitnessString.c_str());
            printf("Returned fitness (%lu): ", fitnessResult.size());
            for (int j = 0; j < fitnessResult.size(); j++) {
              printf("%.2f ", fitnessResult[j]);
            }
            printf("\n");

          }
        }
        break;
      }

      // Evo control - small:
      case '1':
        assert(individuals == 8);
        evolveMorph = false; // Disable morphology evolution
        morphology = "small";
        currentIndividual = individuals-1;
        fitnessFunctions.clear();
        fitnessFunctions.emplace_back("MocapSpeed");
        fitnessFunctions.emplace_back("Stability");
        run_ea(argc, argv, ea, getEvoInfoString());
        break;

      // Evo control - medium:
      case '2':
        assert(individuals == 8);
        evolveMorph = false;
        morphology = "medium";
        currentIndividual = individuals-1;
        fitnessFunctions.clear();
        fitnessFunctions.emplace_back("MocapSpeed");
        fitnessFunctions.emplace_back("Stability");
        run_ea(argc, argv, ea, getEvoInfoString());
        break;

      // Evo control - large:
      case '3':
        assert(individuals == 8);
        evolveMorph = false;
        morphology = "large";
        currentIndividual = individuals-1;
        fitnessFunctions.clear();
        fitnessFunctions.emplace_back("MocapSpeed");
        fitnessFunctions.emplace_back("Stability");
        run_ea(argc, argv, ea, getEvoInfoString());
        break;

      // CO-evo morph+cont:
      case '4':
        assert(individuals == 16);
        evolveMorph = true;
        currentIndividual = individuals-1;
        fitnessFunctions.clear();
        fitnessFunctions.emplace_back("MocapSpeed");
        fitnessFunctions.emplace_back("Stability");
        run_ea(argc, argv, ea, getEvoInfoString());
        break;

      // Reconnect to master:
      case 'r':
        printf("Reconnecting!\n");
        rosConnect();
        printf("Reconnected!\n");
        break;

      // Test fitness noise:
      case 'n':
        {

          for (int i = 0; i < 100; i++){
              resetGaitRecording(get_gait_evaluation_client);
              startGaitRecording(get_gait_evaluation_client);

              sleep(30);

              std::vector<float> gaitResults = getGaitResults(get_gait_evaluation_client);
              printf("\tRes: ");
              for (int i = 0; i < gaitResults.size(); i++){
                  printf("%.5f", gaitResults[i]);
                  if (i != (gaitResults.size()-1)) printf(", "); else printf("\n");
              }
          }

          break;
        }

      // Enable servos:
      case 'e':
        enableServos();
        printf("Servos enabled!\n");
        break;

      // Disable servos:
      case 'd':
        disableServos();
        printf("Servos disabled!\n");
        break;

      // Exit:
      case '0':
        printf("\tExiting program\n");
        break;
      default:
        printf("\tUndefined choice\n");
        break;
    };

    printf("\n");

  } while (inputChar != '0');

  if (evoFitnessLog != NULL) fclose(evoFitnessLog);
  if (evoParamLog_gen != NULL) fclose(evoParamLog_gen);
  if (evoParamLog_phen != NULL) fclose(evoParamLog_phen);

  return 0;
}
