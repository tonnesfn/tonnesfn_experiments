#include <iostream>
#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <chrono>
#include <unistd.h>
#include <stdio.h>

#include "ros/ros.h"

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

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
#include "dyret_controller/GetGaitEvaluation.h"
#include "dyret_controller/GetGaitControllerStatus.h"
#include "dyret_controller/GaitConfiguration.h"
#include "dyret_controller/DistAngMeasurement.h"

#include "external/sferes/phen/parameters.hpp"
#include "external/sferes/gen/evo_float.hpp"
#include "external/sferes/ea/nsga2.hpp"
#include "external/sferes/eval/eval.hpp"
#include "external/sferes/stat/best_fit.hpp"
#include "external/sferes/stat/pareto_front.hpp"
#include "external/sferes/modif/dummy.hpp"
#include "external/sferes/run.hpp"

#include "external/sferes_mapelites/map_elites.hpp"
#include "external/sferes_mapelites/fit_map.hpp"
#include "external/sferes_mapelites/stat_map.hpp"
#include "external/sferes_mapelites/stat_map_binary.hpp"

#include <boost/program_options.hpp>

#include <sstream>
#include <iostream>

#include "individuals.h"

#include "evoSettings.h"

#include "expFunctions.h"

ros::ServiceClient servoConfigClient;
ros::ServiceClient get_gait_evaluation_client;
ros::Publisher poseCommand_pub;
ros::ServiceClient gaitControllerStatus_client;
ros::ServiceClient servoStatus_client;
ros::Subscriber dyretState_sub;
ros::Subscriber gaitInferredPos_sub;
ros::Publisher actionMessages_pub;
ros::Publisher positionCommand_pub;
ros::Publisher gaitConfiguration_pub;

FILE *logOutput = stdout;

double currentFemurLength = 0.0;
double currentTibiaLength = 0.0;
int evaluationTimeout = 18; // 15 sec max each direction
float evaluationDistance = 1500.0;
int currentIndividual;

float gaitInferredPos = 0.0f;

std::string evoLogPath;

const int numberOfEvalsInTesting = 1;

bool evolveMorph = true;
bool instantFitness = false;

std::string morphology;
std::string gaitType;

bool robotOnStand = false;
float currentSpeed = 0.0;

char **argv_g;

std::vector<std::string> fitnessFunctions; // Used to specify which fitness functions to use
std::vector<std::string> commandQueue; // Used to store commands from the arguments temporarily
std::string fullCommand; // Used to store commands from the arguments permanently

bool automatedRun() {
    return !fullCommand.empty();
}

void resetSimulation(){

    std_srvs::Empty empty;

    // First pause physics:
    ros::service::call("/gazebo/pause_physics", empty);

    // Then reset world:
    ros::service::call("/gazebo/reset_world", empty);

    // Then reset DyRET:
    std_srvs::SetBool setBool;
    setBool.request.data = true; // Reset prismatic joints
    ros::service::call("/dyret/simulation/reset", setBool);

    // Unpause physics:
    ros::service::call("/gazebo/unpause_physics", empty);

    // Run a few ticks
    usleep(10000);

    ROS_INFO("Simulation reset");

}

void setRandomRawFitness(ros::ServiceClient get_gait_evaluation_client, std::vector<std::map<std::string, double>> &rawFitnesses) {
    dyret_controller::GetGaitEvaluation srv;
    std::vector<std::string> descriptorsToReturn;

    srv.request.givenCommand = dyret_controller::GetGaitEvaluation::Request::t_getDescriptors;

    if (get_gait_evaluation_client.call(srv)) {
        for (int i = 0; i <= 1; i++) {
            std::map<std::string, double> fitnessMap;

            for (int i = 0; i < srv.response.descriptors.size(); i++) {
                float randNumber = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                fitnessMap[srv.response.descriptors[i]] = randNumber;
            }

            rawFitnesses.push_back(fitnessMap);
        }
    } else {
        ROS_ERROR("Error while calling GaitRecording service with t_getDescriptors!\n");
    }

}

std::map<std::string, double> getGaitResults(ros::ServiceClient get_gait_evaluation_client){

    std::map<std::string, double> mapToReturn;

    dyret_controller::GetGaitEvaluation srv;
    srv.request.givenCommand = dyret_controller::GetGaitEvaluation::Request::t_getResults;

    if (get_gait_evaluation_client.call(srv)) {

        // Make sure the response is valid
        if (srv.response.descriptors.size() != srv.response.results.size()) {
            ROS_ERROR("Result (%lu) and descriptor (%lu) sizes not equal. Cannot process results!", srv.response.results.size(), srv.response.descriptors.size());
            return std::map<std::string, double>();
        }

        // Go through all responses and insert into map
        for (int i  = 0; i < srv.response.descriptors.size(); i++){
            mapToReturn[srv.response.descriptors[i]] = srv.response.results[i];
            //if (srv.response.descriptors[i] == "linAcc_z" && srv.response.results[i] < 2.0) *hasFallen = true;
        }
    } else {
        ROS_ERROR("Error while calling GaitRecording service with t_getResults!\n");
    }

    return mapToReturn;
}

void setGaitParams(std::string gaitName, std::vector<std::string> parameterNames, std::vector<float> parameterValues){

    dyret_controller::GaitConfiguration msg;

    msg.gaitName = gaitName;

    msg.parameterName = parameterNames;

    msg.parameterValue = parameterValues;

    gaitConfiguration_pub.publish(msg);
}

void setGaitParams(std::string gaitName, std::map<std::string, double> phenoTypeMap){
    std::vector<std::string> parameterNames;
    std::vector<float> parametervalues;

    for(auto elem : phenoTypeMap){
        parameterNames.push_back(elem.first);
        parametervalues.push_back((float) elem.second);
    }

    setGaitParams(gaitName, parameterNames, parametervalues);
}

bool gaitControllerDone(ros::ServiceClient gaitControllerStatus_client) {
    dyret_controller::GetGaitControllerStatus srv;

    if (gaitControllerStatus_client.call(srv)) {
        if (srv.response.gaitControllerStatus.actionType == srv.response.gaitControllerStatus.t_idle) return true;
    }

    return false;
}

void setLegLengths(float femurLengths, float tibiaLengths) {
    dyret_common::Pose msg;

    msg.prismatic.resize(2);

    msg.prismatic[0] = femurLengths;
    msg.prismatic[1] = tibiaLengths;

    poseCommand_pub.publish(msg);
}

void dyretStateCallback(const dyret_common::State::ConstPtr &msg) {
    currentFemurLength = (msg->prismatic[0].position + msg->prismatic[2].position + msg->prismatic[4].position +
                          msg->prismatic[6].position) / 4.0;
    currentTibiaLength = (msg->prismatic[1].position + msg->prismatic[3].position + msg->prismatic[5].position +
                          msg->prismatic[7].position) / 4.0;
}

void gaitInferredPosCallback(const dyret_controller::DistAngMeasurement::ConstPtr &msg) {
    gaitInferredPos = msg->distance;
}

bool legsAreLength(float femurLengths, float tibiaLengths) {
    return ((fabs(femurLengths - currentFemurLength) < 3.0f) && (fabs(tibiaLengths - currentTibiaLength) < 3.0f));
}

float getServoVoltage() {
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

float getMaxServoTemperature(bool printAllTemperatures = false) {
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

// This function takes in a phenotype, and returns the fitness for the individual
std::map<std::string, double> getFitness(std::map<std::string, double> phenoType,
                                         const ros::ServiceClient& get_gait_evaluation_client,
                                         std::vector<std::map<std::string, double>> &rawFitnesses) {

    std::map<std::string, double> mapToReturn;

    currentIndividual++;

    // Reset if we are in simulation
    if (ros::Time::isSimTime()) resetSimulation();

    // (Return random fitness to test)
    if (instantFitness) {
        usleep(30000);

        setRandomRawFitness(get_gait_evaluation_client, rawFitnesses);

        std::map<std::string, double> mapToReturn;

        for (int i = 0; i < fitnessFunctions.size(); i++){
            float randNumber = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            mapToReturn[fitnessFunctions[i]] = randNumber;
        }

        return mapToReturn;
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
                disableServos(servoConfigClient, actionMessages_pub);
                long long int currentTime = getMs();
                fprintf(logOutput, "00.0 ");
                while (getMaxServoTemperature(true) > 50) {
                    sleep(10);
                    fprintf(logOutput, "%3.1f: ", ((getMs() - currentTime) / 1000.0) / 60.0);
                }

                std::cout << "Press enter to enable servos";
                std::cin.ignore();
                std::cin.ignore();

                enableServos(actionMessages_pub);

                std::cout << "Press enter to continue evolution";
                std::cin.ignore();
            }
        }
    }

    fprintf(logOutput, "\n  %03u: Evaluating individual:\n", currentIndividual);
    printMap(phenoType, "    ", logOutput);

    if (ros::Time::isSystemTime()) { // Only check temperature in real world
        // Check temperature - if its over the limit below, consider fitness invalid (due to DC motor characterics)
        if (getMaxServoTemperature() > 70.0) {
            fprintf(logOutput, "  Temperature is too high at %.1f\n", getMaxServoTemperature());
            return std::map<std::string, double>();
        }
    }

    // Set gait parameters
    setGaitParams(gaitType, phenoType);

    // Set leg lengths and wait until they reach the correct length
    setLegLengths(phenoType.at("femurLength"), phenoType.at("tibiaLength"));
    int secPassed = 0;
    while (!legsAreLength(phenoType.at("femurLength"), phenoType.at("tibiaLength"))) {
        sleep(1);
        if (secPassed++ > 60) return std::map<std::string, double>(); // 1 min timeout to reconfigure
    }

    // Start the gait and wait for it to finish
    resetGaitRecording(get_gait_evaluation_client);
    sendContGaitMessage(0.0, actionMessages_pub);

    sleep(1);

    // Wait until the robot is done walking
    ros::Time startTime = ros::Time::now();
    while (gaitInferredPos < evaluationDistance) {
        usleep(1000);
        if ((ros::Time::now() - startTime).sec > (evaluationTimeout)) {
            printf("  Timed out forward at %ds\n", (ros::Time::now() - startTime).sec);
            break;
        }
    }

    sendIdleMessage(actionMessages_pub);
    sleep(1);

    std::map<std::string, double> gaitResultsForward = getGaitResults(get_gait_evaluation_client);

    if (gaitResultsForward.empty()) {
        ROS_ERROR("GaitResultsForward.size() == 0!\n");
        return std::map<std::string, double>();
    }

    // Print forward results
    fprintf(logOutput, "  Res F:\n");
    printMap(gaitResultsForward, "    ", logOutput);

    if (ros::Time::isSimTime()) resetSimulation();
    sleep(1);

    resetGaitRecording(get_gait_evaluation_client);
    sendContGaitMessage(M_PI, actionMessages_pub);
    sleep(1);

    startTime = ros::Time::now();
    while (gaitInferredPos > -evaluationDistance) { // Now walking backwards - negate evaluation distance
        usleep(1000);
        if ((ros::Time::now() - startTime).sec > (evaluationTimeout)) {
            printf("  Timed out reverse at %ds\n", (ros::Time::now() - startTime).sec);
            break;
        }
    }

    sendRestPoseMessage(actionMessages_pub);
    sleep(1);

    std::map<std::string, double> gaitResultsReverse = getGaitResults(get_gait_evaluation_client);

    if (gaitResultsReverse.empty()) {
        ROS_ERROR("GaitResultsReverse.size() == 0!\n");
        return std::map<std::string, double>();
    }

    // Print reverse results
    fprintf(logOutput, "  Res R:\n");
    printMap(gaitResultsReverse, "    ", logOutput);


    // Handle robots that fell
    if ((gaitResultsForward.at("linAcc_z") > 0) || (gaitResultsForward.at("linAcc_z") > 0)){
        mapToReturn["Stability"] = fmin(-1.0, (gaitResultsForward["combImuStab"] + gaitResultsReverse["combImuStab"]) / 2.0);

        if (ros::Time::isSimTime()){
            mapToReturn["MocapSpeed"] = (getMapValue(gaitResultsForward, "sensorSpeedForward") - getMapValue(gaitResultsReverse, "sensorSpeedForward")) / 2.0;
        } else {
            mapToReturn["MocapSpeed"] = (gaitResultsForward["sensorSpeed"] + gaitResultsReverse["sensorSpeed"]) / 2.0;
        }
    } else {
        ROS_WARN("Robot fell, discarding fitness");

        mapToReturn["Stability"] = -1;
        mapToReturn["MocapSpeed"] = 0;

    }


    // Print total results
    fprintf(logOutput, "  Res total: \n");
    printMap(mapToReturn, "    ", logOutput);

    fprintf(logOutput, "\n");

    // Add raw fitnesses to container
    rawFitnesses.push_back(gaitResultsForward);
    rawFitnesses.push_back(gaitResultsReverse);

    return mapToReturn;

}

std::map<std::string, double> genToHighLevelSplineGaitPhen(std::vector<double> givenGenotype) {

    assert(givenGenotype.size() >= 10);

    std::map<std::string, double> phenoType;

    phenoType["femurLength"]     = givenGenotype[0] * 25.0;          //  0    ->  25
    phenoType["tibiaLength"]     = givenGenotype[1] * 95.0;          //  0    ->  95
    phenoType["stepLength"]      = givenGenotype[2] * 300.0;         //  0    -> 300
    phenoType["stepHeight"]      = 25.0 + (givenGenotype[3] * 50.0); // 25    ->  75
    phenoType["smoothing"]       = givenGenotype[4] * 50.0;          //  0    ->  50
    phenoType["frequencyFactor"] = givenGenotype[5];                 //  0    ->   1.0
    phenoType["wagPhase"]        = (givenGenotype[6] * 0.4) - 0.2;   // -0.2  ->   0.2
    phenoType["wagAmplitude_x"]  = givenGenotype[7] * 50.0;          //  0    ->  50
    phenoType["wagAmplitude_y"]  = givenGenotype[8] * 50.0;          //  0    ->  50
    phenoType["liftDuration"]    = (givenGenotype[9] * 0.15) + 0.05; //  0.05 ->   0.20

    return phenoType;
}

std::map<std::string, double> genToLowLevelSplineGaitPhen(std::vector<double> givenGenotype) {

    if (givenGenotype.size() < 15){
        ROS_ERROR("givenGenotype.size() < 15: %lu", givenGenotype.size());
        exit(-1);
    }

    std::map<std::string, double> phenoType;

    phenoType["femurLength"]     = givenGenotype[0] * 25.0;          // 0    -> 25
    phenoType["tibiaLength"]     = givenGenotype[1] * 95.0;          // 0    -> 95
    phenoType["liftDuration"]    = (givenGenotype[2] * 0.15) + 0.05; // 0.05 ->  0.20
    phenoType["frequencyFactor"] = givenGenotype[3];

    phenoType["p0_x"] =   givenGenotype[3] * 0.0;
    phenoType["p0_y"] =  (givenGenotype[4] * 300.0) - 150.0; // -150 -> 150
    phenoType["p1_x"] =   givenGenotype[5] * 0.0;
    phenoType["p1_y"] =  (givenGenotype[6] * 300.0) - 150.0; // -150 -> 150;

    phenoType["p2_x"] =   givenGenotype[7] * 0.0;
    phenoType["p2_y"] =  (givenGenotype[8] * 300.0) - 150.0; // -150 -> 150;
    phenoType["p2_z"] =  (givenGenotype[9] * 70.0) + 10.0;   //   10 ->  80
    phenoType["p3_x"] =  givenGenotype[10] * 0.0;
    phenoType["p3_y"] = (givenGenotype[11] * 300.0) - 150.0; // -150 -> 150;
    phenoType["p3_z"] = (givenGenotype[12]* 70.0) + 10.0;;
    phenoType["p4_x"] =  givenGenotype[13] * 0.0;
    phenoType["p4_y"] = (givenGenotype[14] * 300.0) - 150.0; // -150 -> 150;
    phenoType["p4_z"] = (givenGenotype[15] * 70.0) + 10.0;   //   10 ->  80

    return phenoType;
}

std::map<std::string, double> evaluateIndividual(std::vector<double> givenIndividualGenotype) {

    // Set length of individual if not evolving morphology:
    if (evolveMorph == false) {
        if (morphology == "small") {
            givenIndividualGenotype[0] = 0.0;
            givenIndividualGenotype[8] = 0.0;
        } else if (morphology == "medium") {
            givenIndividualGenotype[0] = 0.5;
            givenIndividualGenotype[1] = 0.5;
        } else if (morphology == "large") {
            givenIndividualGenotype[0] = 1.0;
            givenIndividualGenotype[1] = 1.0;
        } else {
            ROS_ERROR("Morphology \"%s\" not recognized!\n", morphology.c_str());
        }
    }

    std::map<std::string, double> individualParameters;
    if (gaitType == "highLevelSplineGait") {
        individualParameters = genToHighLevelSplineGaitPhen(givenIndividualGenotype);
    } else if (gaitType == "lowLevelSplineGait"){
        individualParameters = genToLowLevelSplineGaitPhen(givenIndividualGenotype);
    } else {
        ROS_FATAL("Unknown controller: %s", gaitType.c_str());
        exit(-1);
    }

    bool validSolution;
    std::map<std::string, double> fitnessResult;

    int retryCounter = 0;
    std::vector<std::map<std::string, double>> rawFitness;

    do {
        validSolution = true;

        rawFitness.clear();
        fitnessResult = getFitness(individualParameters, get_gait_evaluation_client, rawFitness);

        if (fitnessResult.empty()) validSolution = false;

        // A valid solution could not be found:
        if ((validSolution == false) || (fitnessResult.size() == 0)) {
            if (automatedRun) {
                if (retryCounter < 3) {
                    fprintf(logOutput, "Retrying\n");
                    if (ros::Time::isSimTime()) resetSimulation();
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
                    fitnessResult.clear();

                    fprintf(logOutput, "Discarding\n");
                    validSolution = true;
                } else if (choice == 'c') { // Cooldown
                    disableServos(servoConfigClient, actionMessages_pub);
                    fprintf(logOutput, "Servos disabled\n");

                    while (getMaxServoTemperature(true) > 50) { sleep(15); }

                    enableServos(actionMessages_pub);

                    std::cout << "Press enter to continue evolution";
                    std::cin.ignore();
                    std::cin.ignore();

                    currentIndividual--;
                    validSolution = false;
                } else { // Retry
                    if (ros::Time::isSimTime()) resetSimulation();

                    fprintf(logOutput, "Retrying\n");
                    currentIndividual--;
                    validSolution = false;
                }
            }
        }

    } while (validSolution == false);

    FILE *genFile = fopen("generation", "r");

    FILE *evoLog = fopen(evoLogPath.c_str(), "a");
    if (evoLog == NULL) {
        ROS_ERROR("evoLog couldnt be opened (err%d)\n", errno);
    }

    int currentGeneration;
    fscanf(genFile, "%d", &currentGeneration);
    fclose(genFile);
    fprintf(evoLog, "    {\n");
    fprintf(evoLog, "      \"id\": %d,\n", currentIndividual);
    fprintf(evoLog, "      \"generation\": %d,\n", currentGeneration);

    fprintf(evoLog, "      \"genotype\": [\n");
    for (int i = 0; i < givenIndividualGenotype.size(); i++) {
        fprintf(evoLog, "        %f", givenIndividualGenotype[i]);
        if (i != givenIndividualGenotype.size() - 1) fprintf(evoLog, ",");
        fprintf(evoLog, "  \n");
    }
    fprintf(evoLog, "      ],\n");

    fprintf(evoLog, "      \"phenotype\": {\n");
    int i = 0;
    for(auto elem : individualParameters){
        fprintf(evoLog, "        \"%s\": %.3f", elem.first.c_str(), elem.second);
        if (i != individualParameters.size()-1) fprintf(evoLog, ",\n"); else fprintf(evoLog, "\n");
        i++;
    }
    fprintf(evoLog, "      },\n");

    fprintf(evoLog, "      \"fitness\": {\n");
    i = 0;
    for(auto elem : fitnessResult){
        fprintf(evoLog, "        \"%s\": %.3f", elem.first.c_str(), elem.second);
        if (i != fitnessResult.size()-1) fprintf(evoLog, ",\n"); else fprintf(evoLog, "\n");
        i++;
    }
    fprintf(evoLog, "      },\n");


    fprintf(evoLog, "      \"raw_fitness\": [\n");
    fprintf(evoLog, "        {\n");
    printMap(rawFitness[0], "          ", evoLog);
    fprintf(evoLog, "        },{\n");
    printMap(rawFitness[1], "          ", evoLog);
    fprintf(evoLog, "        }\n");

    fprintf(evoLog, "      ]\n");

    fprintf(evoLog, "    }");

    if (currentIndividual == ((generations) * popSize) - 1) { // If last individual
        fprintf(evoLog, "\n");
        fprintf(evoLog, "  ]\n");
        fprintf(evoLog, "}");
    } else {
        fprintf(evoLog, ",\n");
    }

    fclose(evoLog);

    return fitnessResult;

}

SFERES_FITNESS (FitExp2MO, sferes::fit::Fitness)
{
public:
    FitExp2MO() {}

    template<typename Indiv>
    void eval(Indiv &ind) {

        this->_objs.resize(fitnessFunctions.size());

        std::vector<double> individualData(ind.gen().size());

        for (int i = 0; i < individualData.size(); i++) individualData[i] = ind.gen().data(i);

        std::map<std::string, double> fitnessResult = evaluateIndividual(individualData);

        for (int i = 0; i < fitnessFunctions.size(); i++){
            this->_objs[i] = fitnessResult[fitnessFunctions[i]];
        }

    }
};

// Sferes
FIT_MAP(FitMapElites)
{
public:
    FitMapElites() {}

    template<typename Indiv>
    void eval(Indiv &ind) {

        this->_objs.resize(fitnessFunctions.size());

        std::vector<double> individualData(10);

        for (int i = 0; i < individualData.size(); i++) individualData[i] = ind.gen().data(i);

        std::map<std::string, double> fitnessResult = evaluateIndividual(individualData);

        for (int i = 0; i < fitnessFunctions.size(); i++){
            this->_objs[i] = (float) fitnessResult.at(fitnessFunctions[i]);
        }

        std::vector<float> data;
        data.push_back(fmin(fitnessResult.at("mocapSpeed") / 10.0f, 1.0));
        data.push_back((individualData[7] * 25.0f + individualData[8] * 95.0f) / 120.0f); // Total leg height

        this->set_desc(data);

    }

    bool dead() {
        return false;
    };
};

#include "sferesExperiments.h"

void run_individual(std::string gaitName, std::map<std::string, double> phenoTypeMap) {

    gaitType = gaitName;

    fitnessFunctions.clear();
    fitnessFunctions.emplace_back("MocapSpeed");
    fitnessFunctions.emplace_back("Stability");

    for (int i = 0; i < numberOfEvalsInTesting; i++) {
        std::vector<std::map<std::string, double>> rawFitnesses;
        std::map<std::string, double> fitnessResult = getFitness(phenoTypeMap,
                                                                 get_gait_evaluation_client,
                                                                 rawFitnesses);
    }
}

void menu_demo() {

    std::string choice;

    std::cout << "  Please choose one demonstration: (enter to go back)\n";

    fprintf(logOutput, "    ss - Test small robot (small HLSC)\n"
                       "    ls - Test large robot (small HLSC)\n"
                       "    ll - Test large robot (large HLSC)\n"
                       "    cs - Test low level spline control (small LLSC)\n"
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

    if (choice.empty() == false) {
        if (choice == "ss") {
            run_individual("highLevelSplineGait", individuals::smallRobotSmallControl);
        } else if (choice == "ls") {
            run_individual("highLevelSplineGait", individuals::largeRobotSmallControl);
        } else if (choice == "ll") {
            run_individual("highLevelSplineGait", individuals::largeRobotLargeControl);
        } else if (choice == "cs") {
            std::vector<std::string> names = {"liftDuration", "frequencyFactor",
                                              "p0_x", "p0_y", "p1_x", "p1_y", "p2_x", "p2_y", "p2_z", "p3_x", "p3_y", "p3_z", "p4_x", "p4_y", "p4_z"};
            std::vector<float> values = {0.20, 0.28,
                                         0.0,  92.5,
                                         0.0, -92.5,
                                         0.0, -92.5, 50.0,
                                         0.0,   0.0, 75.00,
                                         0.0, 142.5, 18.75};
            setGaitParams("lowLevelSplineGait", names, values);
            sleep(1);
            sendContGaitMessage(0.0, actionMessages_pub);
            sleep(10);
            sendRestPoseMessage(actionMessages_pub);
            sleep(1);

        } else if (choice == "ms") {
            setLegLengths(0.0, 0.0);
            fprintf(logOutput, "Small morphology requested\n");
        } else if (choice == "mm") {
            setLegLengths(12.5, 47.5);
            fprintf(logOutput, "Medium morphology requested\n");
        } else if (choice == "ml") {
            setLegLengths(25.0, 95.0);
            fprintf(logOutput, "Large morphology requested\n");
        }
    }
};

std::string getDateString(struct tm *givenTime) {
    std::stringstream ss;

    ss << givenTime->tm_year + 1900
       << std::setw(2) << std::setfill('0') << givenTime->tm_mon + 1
       << std::setw(2) << std::setfill('0') << givenTime->tm_mday
       << std::setw(2) << std::setfill('0') << givenTime->tm_hour
       << std::setw(2) << std::setfill('0') << givenTime->tm_min
       << std::setw(2) << std::setfill('0') << givenTime->tm_sec;

    return ss.str();

}

std::string createExperimentDirectory(std::string prefix, struct tm *givenTime) {
    std::stringstream ss;
    ss << getenv("HOME") << "/catkin_ws/experimentResults/";
    mkdir(ss.str().c_str(), 0700);
    ss.str(std::string());

    ss << getenv("HOME") << "/catkin_ws/experimentResults/" << getDateString(givenTime) << "_" << prefix << "/";

    mkdir(ss.str().c_str(), 0700);
    return ss.str();
}

void experiments_evolve(const std::string givenAlgorithm, const std::string givenMorphology, const std::string givenController, bool evolveMorphology) {
    if (givenAlgorithm != "map-elites" && givenAlgorithm != "nsga2") {
        ROS_ERROR("Unknown evolution algorithm: %s", givenAlgorithm.c_str());
    }

    evolveMorph = evolveMorphology;
    morphology = givenMorphology;
    gaitType = givenController;
    currentIndividual = -1;

    fitnessFunctions.clear();
    fitnessFunctions.emplace_back("Stability");
    if (givenAlgorithm == "nsga2") fitnessFunctions.emplace_back("MocapSpeed");

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

    for (int i = 0; i < numberOfTests; i++) {
        currentIndividual = -1;

        time_t t = time(0);   // get time now
        struct tm *now = localtime(&t);

        std::string experimentDirectory = createExperimentDirectory(givenAlgorithm, now);

        std::stringstream ss;
        ss << experimentDirectory.c_str() << getDateString(now) << "_" << givenAlgorithm << ".json";

        evoLogPath = ss.str();

        FILE *evoLog = fopen(evoLogPath.c_str(), "a");
        if (evoLog == NULL) {
            ROS_ERROR("evoLog could not be opened (err%d)\n", errno);
        }

        char hostname[1024];
        gethostname(hostname, 1024);

        fprintf(evoLog, "{\n");
        fprintf(evoLog, "  \"experiment_info\": {\n");
        fprintf(evoLog, "    \"time\": \"%s\",\n", getDateString(now).c_str());
        if (fullCommand.size() != 0) fprintf(evoLog, "    \"command\": \"%s\",\n", trim(fullCommand).c_str());
        fprintf(evoLog, "    \"type\": \"evolution\",\n");
        fprintf(evoLog, "    \"algorithm\": \"%s\",\n", givenAlgorithm.c_str());
        fprintf(evoLog, "    \"controller\": \"%s\",\n", givenController.c_str());
        fprintf(evoLog, "    \"machine\": \"%s\",\n", hostname);
        fprintf(evoLog, "    \"user\": \"%s\",\n", getenv("USER"));

        if (ros::Time::isSimTime()) fprintf(evoLog, "    \"platform\": \"simulation\",\n");
        else
            fprintf(evoLog, "    \"platform\": \"hardware\",\n");
        fprintf(evoLog, "    \"generations\": %d,\n", generations);
        fprintf(evoLog, "    \"population\": %d,\n", popSize);

        if (evolveMorphology) fprintf(evoLog, "    \"morphology\": \"*evolved*\",\n");
        else
            fprintf(evoLog, "    \"morphology\": \"%s\",\n", givenMorphology.c_str());

        fprintf(evoLog, "    \"fitness\": [\n");
        if (instantFitness == false) {
            for (int i = 0; i < fitnessFunctions.size(); i++) {
                fprintf(evoLog, "      \"%s\"", fitnessFunctions[i].c_str());
                if (i != fitnessFunctions.size() - 1) fprintf(evoLog, ",\n");
            }
        } else fprintf(evoLog, "      \"*INSTANT*\"");
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
        argv_tmp[1] = const_cast<char *>(sferesCommand);

        fprintf(logOutput, "%s, %s\n", argv_tmp[0], argv_tmp[1]);

        if (givenAlgorithm == "nsga2") sferes::run_ea(argc_tmp, argv_tmp, sferes_nsga2::ea, evoLogPath);
        else if (givenAlgorithm == "map-elites") sferes::run_ea(argc_tmp, argv_tmp, sferes_mapElites::ea, evoLogPath);
        else {
            ROS_FATAL("Invalid algorithm (%s) and controller (%s) choices", givenAlgorithm.c_str(), givenController.c_str());
            exit(-1);
        }
        evoLogPath.clear();

        fprintf(logOutput, "Experiment finished. Log written to:\n  %s\n", ss.str().c_str());
    }

};

void experiments_verifyFitness() {
    currentIndividual = 0;

    std::cout << "How many individuals do you want to test? >";

    int numberOfTests;
    std::cin >> numberOfTests;
    std::cin.ignore(10000, '\n');

    time_t t = time(0);   // get time now
    struct tm *now = localtime(&t);

    std::string logDirectory = createExperimentDirectory("ver", now);

    std::stringstream ss;
    ss << logDirectory << getDateString(now) << "_ver.txt";

    std::string verifyLogPath = ss.str();

    FILE *verifyLog = fopen(verifyLogPath.c_str(), "a");
    if (verifyLog == NULL) {
        ROS_ERROR("verifyLog couldnt be opened (err%d)\n", errno);
    }
    fclose(verifyLog);

    fitnessFunctions.clear();
    fitnessFunctions.emplace_back("MocapSpeed");
    fitnessFunctions.emplace_back("Stability");


    ROS_ERROR("Not yet implemented!"); //TODO
    /*
    for (int i = 0; i < numberOfTests; i++) {

        std::vector<float> fitnessResult = getFitness(individuals::smallRobotSmallControl,
                                                      get_gait_evaluation_client);


        fprintf(logOutput, "Returned fitness (%lu): ", fitnessResult.size());
        FILE *verifyLog = fopen(verifyLogPath.c_str(), "a");

        if (i > 0) fprintf(verifyLog, "\n");

        for (int j = 0; j < fitnessResult.size(); j++) {
            fprintf(logOutput, "%.2f ", fitnessResult[j]);

            if (j > 0) fprintf(verifyLog, ", ");
            fprintf(verifyLog, "%f", fitnessResult[j]);
        }

        fclose(verifyLog);
        fprintf(logOutput, "\n");

    }*/
}

void experiments_fitnessNoise() {
    for (int i = 0; i < 100; i++) {
        resetGaitRecording(get_gait_evaluation_client);
        startGaitRecording(get_gait_evaluation_client);

        sleep(30);

        std::map<std::string, double> gaitResults = getGaitResults(get_gait_evaluation_client);

        fprintf(logOutput, "  Res: ");
        for(auto elem : gaitResults){
            fprintf(logOutput, "        \"%s\": %.3f", elem.first.c_str(), elem.second);
            if (i != gaitResults.size()-1) fprintf(logOutput, ",\n"); else fprintf(logOutput, "\n");
            i++;
        }

    }
}

std::vector<double> getRandomIndividual() {
    std::vector<double> individual(17);

    for (int j = 0; j < individual.size(); j++) individual[j] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

    return individual;
}

void experiments_randomSearch() {

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

    for (int i = 0; i < numberOfTests; i++) {

        time_t t = time(0);   // get time now
        struct tm *now = localtime(&t);

        std::string logDirectory = createExperimentDirectory("rand", now);
        std::stringstream ss;
        ss << logDirectory << getDateString(now) << "_rand.json";

        FILE *randomSearchLog = fopen(ss.str().c_str(), "a");
        if (randomSearchLog == NULL) {
            ROS_ERROR("randomSearchLog couldnt be opened (err%d)\n", errno);
        }

        char hostname[1024];
        gethostname(hostname, 1024);

        fprintf(randomSearchLog, "{\n");
        fprintf(randomSearchLog, "  \"experiment_info\": {\n");
        fprintf(randomSearchLog, "    \"time\": \"%s\",\n", getDateString(now).c_str());
        if (fullCommand.size() != 0) fprintf(randomSearchLog, "    \"command\": \"%s\",\n", trim(fullCommand).c_str());
        fprintf(randomSearchLog, "    \"type\": \"random\",\n");
        fprintf(randomSearchLog, "    \"algorithm\": \"random\",\n");
        fprintf(randomSearchLog, "    \"controller\": \"%s\",\n", gaitType.c_str());
        fprintf(randomSearchLog, "    \"machine\": \"%s\",\n", hostname);
        fprintf(randomSearchLog, "    \"user\": \"%s\",\n", getenv("USER"));
        if (ros::Time::isSimTime()) fprintf(randomSearchLog, "    \"platform\": \"simulation\",\n");
        else
            fprintf(randomSearchLog, "    \"platform\": \"hardware\",\n");
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

        for (int j = 0; j < popSize * (generations); j++) {

            // Generate random individual:
            std::vector<double> randomIndividual = getRandomIndividual();

            // Log individual
            randomSearchLog = fopen(ss.str().c_str(), "a");
            fprintf(randomSearchLog, "    {\n");
            fprintf(randomSearchLog, "      \"id\": %d,\n", currentIndividual);

            fprintf(randomSearchLog, "      \"genotype\": [\n");
            for (int k = 0; k < randomIndividual.size(); k++) {
                fprintf(randomSearchLog, "        %f", randomIndividual[k]);
                if (k == randomIndividual.size() - 1) fprintf(randomSearchLog, "\n");
                else
                    fprintf(randomSearchLog, ",\n");
            }
            fprintf(randomSearchLog, "      ],\n");

            std::map<std::string, double> individualParameters;

            if (gaitType == "highLevelSplineGait") {
                individualParameters = genToHighLevelSplineGaitPhen(randomIndividual);
            } else if (gaitType == "lowLevelSplineGait"){
                individualParameters = genToLowLevelSplineGaitPhen(randomIndividual);
            } else {
                ROS_FATAL("Unknown controller: %s", gaitType.c_str());
                exit(-1);
            }


            fprintf(randomSearchLog, "      \"phenotype\": {\n");
            int i = 0;
            for(auto elem : individualParameters){
                fprintf(randomSearchLog, "        \"%s\": %.3f", elem.first.c_str(), elem.second);
                if (i != individualParameters.size()-1) fprintf(randomSearchLog, ",\n"); else fprintf(randomSearchLog, "\n");
                i++;
            }
            fprintf(randomSearchLog, "      },\n");

            fclose(randomSearchLog);

            std::vector<std::map<std::string, double>> rawFitness;
            std::map<std::string, double> fitnessResult = getFitness(individualParameters, get_gait_evaluation_client, rawFitness);

            fprintf(logOutput, "Returned fitness (%lu): ", fitnessResult.size());

            randomSearchLog = fopen(ss.str().c_str(), "a");
            fprintf(randomSearchLog, "      \"fitness\": {\n");

            printMap(fitnessResult, "        ", randomSearchLog);

            fprintf(randomSearchLog, "      },\n");

            fprintf(randomSearchLog, "      \"raw_fitness\": [\n");
            fprintf(randomSearchLog, "        {\n");
            printMap(rawFitness[0], "          ", randomSearchLog);
            fprintf(randomSearchLog, "        },{\n");
            printMap(rawFitness[1], "          ", randomSearchLog);
            fprintf(randomSearchLog, "        }\n");

            fprintf(randomSearchLog, "      ]\n");

            if (currentIndividual == ((generations) * popSize)) { // If last individual
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

    fprintf(logOutput,
            "    cs - evolve control, small morphology, highLevel\n"
            "    cm - evolve control, medium morphology, highLevel\n"
            "    cl - evolve control, large morphology, highLevel\n"
            "    mn - evolve cont+morph, highLevel\n"
            "    me - evolve map-elites, highLevel\n"
            "    el - evolve cont-morph, lowLevel\n"
            "    rh - random search, highLevel\n"
            "    rl - random search, lowLevel\n"
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

    if (!choice.empty()) {
        if (choice == "cs") {
            experiments_evolve("nsga2", "small", "highLevelSplineGait", false);
        } else if (choice == "cm") {
            experiments_evolve("nsga2", "medium", "highLevelSplineGait", false);
        } else if (choice == "cl") {
            experiments_evolve("nsga2", "large", "highLevelSplineGait", false);
        } else if (choice == "mn") {
            experiments_evolve("nsga2", "", "highLevelSplineGait", true);
        } else if (choice == "el") {
            experiments_evolve("nsga2", "", "lowLevelSplineGait", true);
        } else if (choice == "me") {
            experiments_evolve("map-elites", "", "highLevelSplineGait", true);
        } else if (choice == "vf") {
            experiments_verifyFitness();
        } else if (choice == "vn") {
            experiments_fitnessNoise();
        } else if (choice == "rl") {
            gaitType = "lowLevelSplineGait";
            experiments_randomSearch();
        } else if (choice == "rh") {
            gaitType = "highLevelSplineGait";
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
    fprintf(logOutput, "    r - restPose\n");
    fprintf(logOutput, "    x - reset simulation\n");
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
        } else if (choice == "s") {
            if (robotOnStand == true) {
                fprintf(logOutput, "   RobotOnStand now disabled!\n");
            } else {
                fprintf(logOutput, "   RobotOnStand now enabled!\n");
            }

            robotOnStand = !robotOnStand;
        } else if (choice == "e") {
            enableServos(actionMessages_pub);
            fprintf(logOutput, "Servos enabled!\n");
        } else if (choice == "d") {
            disableServos(servoConfigClient, actionMessages_pub);
            fprintf(logOutput, "Servos disabled!\n");
        } else if (choice == "r") {
            sendRestPoseMessage(actionMessages_pub);
        } else if (choice == "x") {
            resetSimulation();
        }

    }
    fprintf(logOutput, "\n");
}

int main(int argc, char **argv) {

    argv_g = argv;

    // If we are running automated run:
    fullCommand = "";
    if (argc > 1) {
        setbuf(stdout, NULL);
        for (int i = 1; i < argc; i++) {
            commandQueue.emplace_back(argv[i]);
            fullCommand.append(argv[i]);
            fullCommand.append(" ");
        }
    }

    FILE *fp = fopen("generation", "w");
    fprintf(fp, "%d", 0);
    fclose(fp);

    fitnessFunctions.emplace_back("Speed");
    fitnessFunctions.emplace_back("Efficiency");

    ros::init(argc, argv, "exp2Gui");
    ros::NodeHandle rch;

    actionMessages_pub = rch.advertise<dyret_controller::ActionMessage>("/dyret/dyret_controller/actionMessages", 10);
    positionCommand_pub = rch.advertise<dyret_controller::PositionCommand>("/dyret/dyret_controller/positionCommand", 1);
    gaitConfiguration_pub = rch.advertise<dyret_controller::GaitConfiguration>("/dyret/dyret_controller/gaitConfiguration", 1);

    servoConfigClient = rch.serviceClient<dyret_common::Configure>("/dyret/configuration");
    get_gait_evaluation_client = rch.serviceClient<dyret_controller::GetGaitEvaluation>("get_gait_evaluation");
    gaitControllerStatus_client = rch.serviceClient<dyret_controller::GetGaitControllerStatus>(
            "get_gait_controller_status");

    poseCommand_pub = rch.advertise<dyret_common::Pose>("/dyret/command", 10);
    dyretState_sub = rch.subscribe("/dyret/state", 1, dyretStateCallback);
    gaitInferredPos_sub = rch.subscribe("/dyret/dyret_controller/gaitInferredPos", 100, gaitInferredPosCallback);


    waitForRosInit(get_gait_evaluation_client, "get_gait_evaluation");
    waitForRosInit(gaitControllerStatus_client, "gaitControllerStatus");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string evoInfo = "testInfo";

    if (resetGaitRecording(get_gait_evaluation_client) == false) {
        spinner.stop();
        ros::shutdown();
        exit(0);
    }

    currentIndividual = -1;

    if (ros::Time::isSimTime()) {
        fprintf(logOutput, "Currently running in simulation mode\n");
    } else {
        fprintf(logOutput, "Currently running in hardware mode\n");
    }

    sleep(1);

    std::map<std::string, boost::function<void()> > menu;
    menu["demo"] = &menu_demo;
    menu["experiments"] = &menu_experiments;
    menu["configure"] = &menu_configure;

    std::string choice;
    for (;;) {
        std::cout << "Please choose: (enter to quit)\n";
        std::map<std::string, boost::function<void()> >::iterator it = menu.begin();
        while (it != menu.end()) {
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

        if (choice.empty() == true || choice == "exit") {
            spinner.stop();
            ros::shutdown();
            exit(0);
        } else if (menu.find(choice) == menu.end()) {
            fprintf(logOutput, "Unknown choice!\n");
            continue;
        }

        menu[choice]();
    }

}
