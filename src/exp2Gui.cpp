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

ros::ServiceClient get_gait_evaluation_client;
ros::Publisher trajectoryMessage_pub;
ros::Publisher actuatorCommand_pub;
ros::ServiceClient gaitControllerStatus_client;
ros::Subscriber actuatorState_sub;

FILE * evoFitnessLog;
FILE * evoParamLog_gen;
FILE * evoParamLog_phen;
double globalSpeed;
float currentFemurLength = 0.0;
float currentTibiaLength = 0.0;
int currentIndividual;

bool robotOnStand = false;

std::vector<std::string> fitnessFunctions;

FILE* getEvoPathFileHandle(std::string fileName){
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

  FILE * handleToReturn = fopen(line.c_str(), "w+");

  return handleToReturn;
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
                   double givenWagAmplitude_y){

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

void servoConfigsCallback(const dyret_common::Configuration::ConstPtr& msg) {
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

std::vector<float> evaluateIndividual(std::vector<double> parameters, std::string* fitnessString, bool speedAspectLocked, ros::ServiceClient gaitControllerStatus_client, ros::Publisher trajectoryMessage_pub, ros::ServiceClient get_gait_evaluation_client){

  printf("%03u: Evaluating stepLength %.2f, "
         "stepHeight %.2f, "
         "smoothing %.2f, "
         "frequency: %.2f, "
         "speed: %.2f, "
         "wagPhase: %.2f, "
         "wagAmplitude_x: %.2f, "
         "wagAmplitude_y: %.2f,"
         "femurLength: %.2f,"
         "tibiaLength: %.2f\n",
         currentIndividual,
         parameters[0],
         parameters[1],
         parameters[2],
         parameters[3],
         parameters[4],
         parameters[5],
         parameters[6],
         parameters[7],
         parameters[8],
         parameters[9]);

  currentIndividual++;

  setGaitParams(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5], parameters[6], parameters[7]);

  setLegLengths(parameters[8], parameters[9]);
  while(!legsAreLength(parameters[8], parameters[9])){
    sleep(0.01);
  }

  std::vector<float> trajectoryAngles(1);
  std::vector<float> trajectoryDistances(1);
  std::vector<int>   trajectoryTimeouts(1);
  trajectoryDistances[0] = 1000.0;
  trajectoryTimeouts[0] = 10.0; // 10 sec timeout

  resetTrajectoryPos(trajectoryMessage_pub); // Reset position before starting
  resetGaitRecording(get_gait_evaluation_client);
  sendTrajectories(trajectoryDistances, trajectoryAngles, trajectoryTimeouts, trajectoryMessage_pub);
  sleep(5);

  // Todo: do this check intelligently
  while(gaitControllerDone(gaitControllerStatus_client) == false){ sleep(1);};

  std::vector<float> gaitResultsForward = getGaitResults(get_gait_evaluation_client);

  if (gaitResultsForward.size() == 0){
      return gaitResultsForward;
  }

  printf("\tRes: ");
  for (int i = 0; i < gaitResultsForward.size(); i++){
      printf("%.5f", gaitResultsForward[i]);
      if (i != (gaitResultsForward.size()-1)) printf(", "); else printf("\n");
  }

  trajectoryDistances[0] = 0.0;
  trajectoryTimeouts[0] = 10.0; // 10 sec timeout

  resetGaitRecording(get_gait_evaluation_client);
  sendTrajectories(trajectoryDistances, trajectoryAngles, trajectoryTimeouts, trajectoryMessage_pub);
  sleep(5);

  while(gaitControllerDone(gaitControllerStatus_client) == false){ sleep(1);}

  std::vector<float> gaitResultsReverse = getGaitResults(get_gait_evaluation_client);

  if (gaitResultsReverse.size() == 0){
      return gaitResultsReverse;
    }

  printf("\tRes: ");
  for (int i = 0; i < gaitResultsReverse.size(); i++){
      printf("%.5f", gaitResultsReverse[i]);
      if (i != (gaitResultsReverse.size()-1)) printf(", "); else printf("\n");
  }

  std::vector<float> fitness(fitnessFunctions.size());
  int currentFitnessIndex = 0;

  float fitness_inferredSpeed = (gaitResultsForward[0] + gaitResultsReverse[0]) / 2.0;
  float fitness_current = (gaitResultsForward[4] + gaitResultsReverse[4]) / 2.0;
  float fitness_stability = (gaitResultsForward[3] + gaitResultsReverse[3] + ((gaitResultsForward[2] + gaitResultsReverse[2]) / 50.0)) / 2.0;
  float fitness_mocapSpeed = (gaitResultsForward[5] + gaitResultsReverse[5]) / 2.0;

  // Inferred speed:
  if(isFitnessObjective("InferredSpeed")){
        fitness[currentFitnessIndex] = fitness_inferredSpeed;
        currentFitnessIndex++;
  }

  if(isFitnessObjective("Current")){
        fitness[currentFitnessIndex] = fitness_current;
        currentFitnessIndex++;
  }

  if(isFitnessObjective("Stability")){
        fitness[currentFitnessIndex] = fitness_stability;
        currentFitnessIndex++;
  }

  if (isFitnessObjective("MocapSpeed")){
      fitness[currentFitnessIndex] = fitness_mocapSpeed;
      currentFitnessIndex++;
  }

  std::stringstream ss;

  ss << fitness_inferredSpeed << ", " << fitness_current << ", " << fitness_stability << ", " << fitness_mocapSpeed;
  *fitnessString = ss.str();

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
    SFERES_CONST unsigned size     =    4;  // Population size
    SFERES_CONST unsigned nb_gen   =   16;  // Number of generations
    SFERES_CONST int dump_period   =    1;  // How often to save
    SFERES_CONST int initial_aleat =    1;  // Individuals to be created during random generation process
  };
  struct parameters {
    SFERES_CONST float min = 0.0f;
    SFERES_CONST float max = 1.0f;
  };
};

SFERES_FITNESS(FitExp2MO, sferes::fit::Fitness) {
public:
  FitExp2MO()  {}
  template<typename Indiv>
  void eval(Indiv& ind) {

    this->_objs.resize(fitnessFunctions.size());

    std::vector<double> individualParameters;

    double speed, frequency;


//   if (isFitnessObjective("Speed")){
        // If evolving for speed, set frequency from genome
        frequency = 0.2 + ind.data(6) * 1.3;
        speed = NAN;
/*    } else {
        // If not evolving from speed, set speed directly
       frequency = NAN;
        speed = globalSpeed;
    }
*/

    individualParameters = { 50 +  ind.data(0)*100.0, // 0: stepLength        50 -> 150
                             25 + (ind.data(1)*50.0), // 1: stepHeight        25 -> 75
                             ind.data(2)*50.0,        // 2: smoothing          0 -> 50
                             frequency,               // 6: frequency        0.2 -> 1.5
                             speed,                   // 6: speed
                             (ind.data(3)*0.4)-0.2,   // 3: wagPhase        -0.2 -> 0.2
                             ind.data(4)*50.0,        // 4: wagAmplitude_x     0 -> 50
                             ind.data(5)*50.0,        // 5: wagAmplitude_y     0 -> 50
                             ind.data(7)*25.0,        // 7: femurLength        0 -> 25
                             ind.data(8)*95.0         // 8: tibiaLength          0 -> 90
                           };
    std::string fitnessDescription_gen  = "stepLength, stepHeight, smoothing, wagPhase, wagAmplitude_x, wagAmplitude_y, frequency, femurLength, tibiaLength\n";
    std::string fitnessDescription_phen = "stepLength, stepHeight, smoothing, frequency, speed, wagPhase, wagAmplitude_x, wagAmplitude_y, femurLength, tibiaLength\n";

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

        if ((validSolution == false) || (fitnessResult.size() == 0)){
          printf("Got invalid fitness: reset the robot and choose action (r/d): ");

          char choice;
          scanf(" %c", &choice);

          if (choice == 'd'){ // Discard
              fitnessResult.resize(fitnessFunctions.size());
              for (int i = 0; i < fitnessResult.size(); i++) fitnessResult[i] = -1000;

              printf("Discarding\n");
              validSolution = true;
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
        evoParamLog_gen = getEvoPathFileHandle("evoParamLog_gen.csv");
        fprintf(evoParamLog_gen, "%s", fitnessDescription_gen.c_str());
    }

    fprintf(evoParamLog_gen, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                ind.data(0), ind.data(1), ind.data(2), ind.data(3),
                ind.data(4), ind.data(5), ind.data(6), ind.data(7), ind.data(8));
    fflush(evoParamLog_gen);

    if (evoParamLog_phen == NULL){
        evoParamLog_phen = getEvoPathFileHandle("evoParamLog_phen.csv");
        fprintf(evoParamLog_phen, "%s", fitnessDescription_phen.c_str());
    }

    for (int i = 0; i < individualParameters.size(); i++) if(i != (individualParameters.size()-1)) fprintf(evoParamLog_phen, "%f, ", individualParameters[i]); else fprintf(evoParamLog_phen, "%f\n", individualParameters[i]);
    fflush(evoParamLog_phen);

    if (evoFitnessLog == NULL){
        evoFitnessLog = getEvoPathFileHandle("evoFitnessLog.csv");
        fprintf(evoFitnessLog, "inferredSpeed, current, stability, mocapSpeed\n");
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

int main(int argc, char **argv){

  fitnessFunctions.emplace_back("Speed");
  fitnessFunctions.emplace_back("Efficiency");

  globalSpeed = 5.0;

  evoFitnessLog = NULL;
  evoParamLog_gen = NULL;
  evoParamLog_phen = NULL;

  ros::init(argc, argv, "exp2Gui");
  ros::NodeHandle n;

  get_gait_evaluation_client = n.serviceClient<dyret_common::GetGaitEvaluation>("get_gait_evaluation");
  gaitControllerStatus_client = n.serviceClient<dyret_common::GetGaitControllerStatus>("get_gait_controller_status");
  trajectoryMessage_pub = n.advertise<dyret_common::Trajectory>("trajectoryMessages", 1000);
  actuatorCommand_pub = n.advertise<dyret_common::Configuration>("actuatorCommands", 10);
  actuatorState_sub = n.subscribe("/actuatorStates", 1, servoConfigsCallback);

  waitForRosInit(get_gait_evaluation_client, "get_gait_evaluation");
  waitForRosInit(gaitControllerStatus_client, "gaitControllerStatus");
  waitForRosInit(trajectoryMessage_pub, "trajectoryMessage");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  typedef gen::EvoFloat<9, Params> gen_t;   // Number of parameters in each individual:
  typedef phen::Parameters<gen_t, FitExp2MO<Params>, Params> phen_t;
  typedef eval::Eval<Params> eval_t;
  typedef boost::fusion::vector<stat::ParetoFront<phen_t, Params> >  stat_t;
  typedef modif::Dummy<> modifier_t;
  typedef ea::Nsga2<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;
  ea_t ea;

  std::string evoInfo = "testInfo";

  resetTrajectoryPos(trajectoryMessage_pub);
  resetGaitRecording(get_gait_evaluation_client);

  int inputChar;
  do {
    printf("1 - Enable/disable stand testing\n"
           "2 - Test longest legs\n"
           "3 - Evo SO (mocapSpeed)\n"
           "4 - Evo SO (stability)\n"
           "5 - Evo MO (mocapSpeed+stability)\n"
           "9 - Test fitness noise (10min)\n"
           "0 - Exit\n> ");

    currentIndividual = 1;

    inputChar = getchar();
    std::cin.ignore(1000,'\n');

    switch(inputChar){
      case '1':
        if (robotOnStand == true){
          printf("   RobotOnStand now disabled!\n");
        } else {
          printf("   RobotOnStand now enabled!\n");
        }

        robotOnStand = !robotOnStand;
        break;
      case '2':
      {
        for (int i = 0; i < 1; i++){
          std::vector<double> individualParameters;

          // MO_speedStability_1_stable
          individualParameters = {80.00,  // stepLength
                                  50.00,  // stepHeight
                                  50.00,  // smoothing
                                   0.10,  // gaitFrequency
                                    NAN,  // speed
                                   0.08,  // wagPhase -0.2 -> 0.2
                                  35.00,  // wagAmplitude_x
                                  35.00,  // wagAmplitude_y
                                  25.00,  // femurLength
                                  95.00}; // tibiaLength


          fitnessFunctions.clear();
          fitnessFunctions.emplace_back("MocapSpeed");
          fitnessFunctions.emplace_back("Stability");

          std::string fitnessString;
          std::vector<float> fitnessResult = evaluateIndividual(individualParameters, &fitnessString, false, gaitControllerStatus_client, trajectoryMessage_pub, get_gait_evaluation_client);
          printf("Returned fitness (%lu): ", fitnessResult.size());
          for (int i = 0; i < fitnessResult.size(); i++){
            printf("%.2f ", fitnessResult[i]);
          }
          printf("\n");
        }
        break;
      }
      case '3':
        fitnessFunctions.clear();
        fitnessFunctions.emplace_back("MocapSpeed");
        run_ea(argc, argv, ea, getEvoInfoString());
        break;
      case '4':
        fitnessFunctions.clear();
        fitnessFunctions.emplace_back("Stability");
        run_ea(argc, argv, ea, getEvoInfoString());
        break;
      case '5':
        fitnessFunctions.clear();
        fitnessFunctions.emplace_back("MocapSpeed");
        fitnessFunctions.emplace_back("Stability");
        run_ea(argc, argv, ea, getEvoInfoString());
        break;
      case '9':
        {

          for (int i = 0; i < 10; i++){
              resetGaitRecording(get_gait_evaluation_client);
              startGaitRecording(get_gait_evaluation_client);

              sleep(60);

              std::vector<float> gaitResults = getGaitResults(get_gait_evaluation_client);
              printf("\tRes: ");
              for (int i = 0; i < gaitResults.size(); i++){
                  printf("%.5f", gaitResults[i]);
                  if (i != (gaitResults.size()-1)) printf(", "); else printf("\n");
              }
          }

          break;
        }
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
