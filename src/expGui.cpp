#include <iostream>
#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
#include <iomanip>
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

#include "dyret_controller/ActionMessage.h"
#include "dyret_controller/GetGaitEvaluation.h"
#include "dyret_controller/GetGaitControllerStatus.h"
#include "dyret_controller/ConfigureGait.h"
#include "dyret_controller/DistAngMeasurement.h"
#include "dyret_controller/GetInferredPosition.h"
#include "dyret_controller/GaitControllerCommandService.h"

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
ros::ServiceClient inferredPositionClient;
ros::ServiceClient get_gait_evaluation_client;
ros::Publisher poseCommand_pub;
ros::ServiceClient gaitControllerStatus_client;
ros::ServiceClient servoStatus_client;
ros::ServiceClient gaitConfiguration_client;
ros::ServiceClient gaitCommandService_client;
ros::Subscriber dyretState_sub;
ros::Subscriber gaitInferredPos_sub;
ros::Publisher actionMessages_pub;

gazebo::WorldConnection& gz = gazebo::WorldConnection::instance();

unsigned int randomSeed;

FILE *logOutput = stdout;

double currentFemurLength = 0.0;
double currentTibiaLength = 0.0;
int evaluationTimeout = 15; // 15 sec max each direction
float evaluationDistance = 1500.0;
int currentIndividual;

std::string evoLogPath;

const int numberOfEvalsInTesting = 1;

const bool useStopCondition = false;
const int evalsWithoutImprovement = 64; // Number of individuals without improvement before evolution is stopped

float gaitDifficultyFactor = 0.5;

bool evolveMorph = true;
bool instantFitness = false;

std::string morphology;
std::string gaitType;

bool robotOnStand = false;

char **argv_g;

std::vector<std::vector<float>> lastOptimalParents;

std::vector<std::string> fitnessFunctions; // Used to specify which fitness functions to use
std::vector<std::string> commandQueue; // Used to store commands from the arguments temporarily
std::string fullCommand; // Used to store commands from the arguments permanently

bool automatedRun() {
    return !fullCommand.empty();
}

void resetSimulation(){
/*
    usleep(1000);

    std_srvs::Empty empty;

    // First pause physics:
    ros::service::call("/gazebo/pause_physics", empty);

    usleep(1000);

    // Then reset world:
    ros::service::call("/gazebo/reset_world", empty);

    usleep(1000);

    // Then reset DyRET:
    std_srvs::SetBool setBool;
    setBool.request.data = true; // Reset prismatic joints
    ros::service::call("/dyret/simulation/reset", setBool);

    usleep(1000);

    // Unpause physics:
    ros::service::call("/gazebo/unpause_physics", empty);

    // Run a few ticks
    usleep(1000);

    ROS_INFO("Simulation reset");*/

    if (gz.reset() == false){
        ROS_ERROR("Could not reset simulation");
    }
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

    dyret_controller::ConfigureGait srv;

    srv.request.gaitConfiguration.gaitName       = gaitName;
    srv.request.gaitConfiguration.parameterName  = parameterNames;
    srv.request.gaitConfiguration.parameterValue = parameterValues;

    gaitConfiguration_client.call(srv);
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

float getInferredPosition(){
    dyret_controller::GetInferredPosition srv;

    if (!inferredPositionClient.call(srv)){
        printf("Error while calling GetInferredPosition service\n");
        ROS_ERROR("Error while calling GetInferredPosition service");

        return 0.0;
    }

    return srv.response.currentInferredPosition.distance;
}

void runGaitControllerWithActionMessage(bool forward){

    resetGaitRecording(get_gait_evaluation_client);

    if (forward) sendContGaitMessage(0.0, actionMessages_pub);
    else sendContGaitMessage(M_PI, actionMessages_pub);

    sleep(1);

    // Wait until the robot is done walking
    ros::Time startTime = ros::Time::now();
    while (getInferredPosition() < evaluationDistance) {
        usleep(1000);
        if ((ros::Time::now() - startTime).sec > (evaluationTimeout)) {
            printf("  Timed out forward at %ds\n", (ros::Time::now() - startTime).sec);
            break;
        }
    }

    sendIdleMessage(actionMessages_pub);
    sleep(1);
}

void spinGaitControllerOnce(){

    dyret_controller::GaitControllerCommandService srv;

    srv.request.gaitControllerCommand.gaitControllerCommand = srv.request.gaitControllerCommand.t_spinOnce;

    if (gaitCommandService_client.call(srv) == false) {
        //ROS_ERROR("Error while calling gaitControllerCommand service");
        // ERROR HERE
    }

}

void runGaitWithServiceCalls(){
    resetGaitRecording(get_gait_evaluation_client);

    ros::Time startTime_sim = ros::Time::now();
    ros::WallTime startTime_rw = ros::WallTime::now();
    while (getInferredPosition() < evaluationDistance) {
        gz.step(30);
        spinGaitControllerOnce();

        // Timeout in sim time
        if ((ros::Time::now() - startTime_sim).sec > (evaluationTimeout)) {
            printf("  Timed out (sim) forward at %ds\n", (ros::Time::now() - startTime_sim).sec);
            break;
        }

        // Timeout in realtime
        if ((ros::WallTime::now() - startTime_rw).sec > 120) {
            printf("  Timed out (forward) at %ds\n", (ros::WallTime::now() - startTime_rw).sec);
            return;
        }


    }

    pauseGaitRecording(get_gait_evaluation_client);

    auto current_time = std::chrono::high_resolution_clock::now();

    std::cout << "Program has been running for " << (ros::WallTime::now() - startTime_rw).sec << " seconds in realtime" << std::endl;
    std::cout << "Program has been running for " << (ros::Time::now() - startTime_sim).sec << " seconds in sim time" << std::endl;

}

// This function takes in a phenotype, and returns the fitness for the individual
std::map<std::string, double> getFitness(std::map<std::string, double> phenoType,
                                         const ros::ServiceClient& get_gait_evaluation_client,
                                         std::vector<std::map<std::string, double>> &rawFitnesses) {

    std::map<std::string, double> mapToReturn;

    currentIndividual++;

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

    // Reset if we are in simulation
    if (ros::Time::isSimTime()){
        resetSimulation();
        usleep(1000);
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
            ROS_ERROR("  Temperature is too high at %.1f\n", getMaxServoTemperature());
            return std::map<std::string, double>();
        }
    }

    // Set gait parameters
    setGaitParams(gaitType, phenoType);

    // Set leg lengths and wait until they reach the correct length
    ROS_INFO("Setting leg lengths");
    setLegLengths(phenoType.at("femurLength"), phenoType.at("tibiaLength"));
    int secPassed = 0;
    while (!legsAreLength(phenoType.at("femurLength"), phenoType.at("tibiaLength"))) {
        sleep(1);
        setLegLengths(phenoType.at("femurLength"), phenoType.at("tibiaLength"));
        if ( ((ros::Time::isSystemTime()) && (secPassed > 60)) || (ros::Time::isSimTime() && (secPassed > 5))){
            ROS_ERROR("Timed out waiting for legs to be at length");
            return std::map<std::string, double>();
        }
        secPassed += 1;
    }
    ROS_INFO("Leg lengths achieved");



    if (ros::Time::isSystemTime()) {
        runGaitControllerWithActionMessage(true);
    } else {
        pauseGazebo();
        runGaitWithServiceCalls();
        unpauseGazebo();
    }

    std::map<std::string, double> gaitResultsForward = getGaitResults(get_gait_evaluation_client);

    // Check for empty results
    if (gaitResultsForward.empty()) {
        ROS_ERROR("GaitResultsForward.size() == 0!");
        return std::map<std::string, double>();
    }

    //printMap(gaitResultsForward, "gaitResultsForward: ", stderr);

    // Check for nan values
    for(auto elem : gaitResultsForward){
        if (std::isnan(elem.second)){
            ROS_ERROR("Got NAN value");
            return std::map<std::string, double>();
        }
    }

    // Print forward results
    fprintf(logOutput, "  Res F:\n");
    printMap(gaitResultsForward, "    ", logOutput);

    if (ros::Time::isSimTime()){
        resetSimulation();
        usleep(1000);
    }

    std::map<std::string, double> gaitResultsReverse;

    // Only evaluate reverse if in the real world:
    if (ros::Time::isSystemTime()) {

        resetGaitRecording(get_gait_evaluation_client);

        if (ros::Time::isSimTime()) {
            ROS_INFO("Setting leg lengths");
            setLegLengths(phenoType.at("femurLength"), phenoType.at("tibiaLength"));
            int secPassed = 0;
            while (!legsAreLength(phenoType.at("femurLength"), phenoType.at("tibiaLength"))) {
                sleep(1);
                setLegLengths(phenoType.at("femurLength"), phenoType.at("tibiaLength"));
                if (((ros::Time::isSystemTime()) && (secPassed > 60)) || (ros::Time::isSimTime() && (secPassed > 5))) {
                    ROS_ERROR("Timed out waiting for legs to be at length");
                    return std::map<std::string, double>();
                }
                secPassed += 1;
            }
            ROS_INFO("Leg lengths achieved");
        }

        runGaitControllerWithActionMessage(false);

        gaitResultsReverse = getGaitResults(get_gait_evaluation_client);

        // Check for empty
        if (gaitResultsReverse.empty()) {
            ROS_ERROR("GaitResultsReverse.size() == 0!");
            return std::map<std::string, double>();
        }

        // Check for nan values
        for (auto elem : gaitResultsReverse) {
            if (std::isnan(elem.second)) {
                ROS_ERROR("Got NAN value");
                return std::map<std::string, double>();
            }
        }

        // Print reverse results
        fprintf(logOutput, "  Res R:\n");
        printMap(gaitResultsReverse, "    ", logOutput);
    }

    // Handle robots that fell
    if ((gaitResultsForward.at("linAcc_z") > 0) || ((!gaitResultsReverse.empty()) && (gaitResultsReverse.at("linAcc_z") > 0))){
        if (gaitResultsReverse.empty()) {
            mapToReturn["Stability"] = fmax(-1.0, gaitResultsForward["combImuStab"]);
        } else {
            mapToReturn["Stability"] = fmax(-1.0, (gaitResultsForward["combImuStab"] + gaitResultsReverse["combImuStab"]) / 2.0);
        }

        if (ros::Time::isSimTime()){
            mapToReturn["MocapSpeed"] = getMapValue(gaitResultsForward, "sensorSpeedForward");
        } else {
            if (gaitResultsReverse.empty()) {
                mapToReturn["MocapSpeed"] = gaitResultsForward["sensorSpeed"];
            } else {
                mapToReturn["MocapSpeed"] = (gaitResultsForward["sensorSpeed"] + gaitResultsReverse["sensorSpeed"]) / 2.0;
            }
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
    if (!gaitResultsReverse.empty()){
        rawFitnesses.push_back(gaitResultsReverse);
    }

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
    phenoType["frequency"]       = 0.25 + givenGenotype[5] * 1.25;   //  0.25 ->   1.5
    phenoType["wagPhase"]        = (givenGenotype[6] * 0.4) - 0.2;   // -0.2  ->   0.2
    phenoType["wagAmplitude_x"]  = givenGenotype[7] * 50.0;          //  0    ->  50
    phenoType["wagAmplitude_y"]  = givenGenotype[8] * 50.0;          //  0    ->  50
    phenoType["liftDuration"]    = (givenGenotype[9] * 0.15) + 0.05; //  0.05 ->   0.20

    return phenoType;
}

double mapNumber(double value, double start1, double stop1, double start2, double stop2) {
    return start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
}

double getPoint(double givenNumber, double givenMinValue, double givenMaxValue, double givenOffset, double givenMinRange, double difficultyLevel) {
    //printf("  getPoint(givenNumber=%.2f, minValue=%.2f, maxvalue=%.2f, offset=%.2f, difficultyLevel=%.2f", givenNumber, givenMinValue, givenMaxValue, givenOffset, difficultyLevel);

    double oldRange = givenMaxValue - givenMinValue;
    double newRange = (((givenMaxValue - givenMinValue) - givenMinRange) * (difficultyLevel)) + givenMinRange;

    double newMax = (newRange / 2) + givenOffset;
    double newMin = (-newRange / 2) + givenOffset;

    //printf("  oldRange: %.2f, newRange: %.2f, newMax: %.2f, newMin: %.2f",oldRange, newRange, newMax, newMin);

    if (newMax > givenMaxValue) {
        newMin = newMin - (newMax - givenMaxValue);
        newMax = givenMaxValue;
    }

    if (newMin < givenMinValue) {
        newMax = newMax - (newMin - givenMinValue);
        newMin = givenMinValue;
    }

    //printf("  oldRange: %.2f, newRange: %.2f, newMax: %.2f, newMin: %.2f", oldRange, newRange, newMax, newMin);

    double numberToReturn = mapNumber(givenNumber, 0, 1, newMin, newMax);

    //printf("  giveNumber: %.2f, new number: %.2f", givenNumber, numberToReturn);

    return numberToReturn;
}

std::map<std::string, double> genToLowLevelSplineGaitPhen(std::vector<double> givenGenotype) {

    if (givenGenotype.size() < 20){
        ROS_ERROR("givenGenotype.size() < 20: %lu", givenGenotype.size());
        exit(-1);
    }

    std::map<std::string, double> phenoType;

    phenoType["difficultyFactor"] = gaitDifficultyFactor;

    phenoType["femurLength"]     = givenGenotype[0] * 25.0;          // 0    -> 25
    phenoType["tibiaLength"]     = givenGenotype[1] * 95.0;          // 0    -> 95
    phenoType["liftDuration"]    = getPoint(givenGenotype[2], 0.05, 0.20, 0.175, 0.05, gaitDifficultyFactor); // 0.15, 0.2 -> 0.05, 0.2
    phenoType["frequency"]       = 0.25 + (givenGenotype[3] * 1.25); // 0.25 ->  1.5

    phenoType["wagPhase"]        = getPoint(givenGenotype[4], -M_PI/2.0, M_PI/2.0, 0.0, 0.2, gaitDifficultyFactor);
    phenoType["wagAmplitude_x"]  = getPoint(givenGenotype[5],         0,     50.0, 0.0, 5.0, gaitDifficultyFactor);
    phenoType["wagAmplitude_y"]  = getPoint(givenGenotype[6],         0,     50.0, 0.0, 5.0, gaitDifficultyFactor);

    phenoType["p0_x"] =   givenGenotype[7] * 0.0;
    phenoType["p1_x"] =   givenGenotype[9] * 0.0;

    // StepLength 25 -> 300, p0 center around 75, p1 center around -75
    phenoType["p0_y"] = getPoint(fmax(givenGenotype[8], givenGenotype[10]), -150.0, 150.0,   50.0, 50.0, gaitDifficultyFactor);
    phenoType["p1_y"] = getPoint(fmin(givenGenotype[8], givenGenotype[10]), -150.0, 150.0, -100.0, 50.0, gaitDifficultyFactor);

    // (potential) Front air point:
    phenoType["p2_x"] = getPoint(givenGenotype[11],  -25.0,  25.0,  0.0,  0.0, gaitDifficultyFactor); // -25, 25 -> 0, 0
    phenoType["p2_y"] = getPoint(givenGenotype[12], -150.0, 150.0, 75.0, 50.0, gaitDifficultyFactor); // -150, 150 -> 50, 100
    phenoType["p2_z"] = getPoint(givenGenotype[13],   10.0,  80.0, 30.0, 10.0, gaitDifficultyFactor); // 10, 80 -> 25, 35

    // (potential) Top air point:
    phenoType["p3_x"] = getPoint(givenGenotype[14],  -25.0,  25.0,  0.0,  0.0, gaitDifficultyFactor); // -25, 25 -> 0, 0
    phenoType["p3_y"] = getPoint(givenGenotype[15], -150.0, 150.0,  0.0,  0.0, gaitDifficultyFactor); // -150, 150 -> 0, 0
    phenoType["p3_z"] = getPoint(givenGenotype[16],   10.0,  80.0, 50.0, 10.0, gaitDifficultyFactor); // 10, 80 -> 45, 55

    // (potential) Back air point:
    phenoType["p4_x"] = getPoint(givenGenotype[17],  -25.0,  25.0,   0.0,  0.0, gaitDifficultyFactor); // -25, 25 -> 0, 0
    phenoType["p4_y"] = getPoint(givenGenotype[18], -150.0, 150.0, -75.0, 50.0, gaitDifficultyFactor); // -150, 150 -> -50, -100
    phenoType["p4_z"] = getPoint(givenGenotype[19],   10.0,  80.0,  30.0, 10.0, gaitDifficultyFactor); // 10, 80 -> 25, 35

    return phenoType;
}

std::map<std::string, double> evaluateIndividual(std::vector<double> givenIndividualGenotype, FILE *logFile) {

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

        if (fitnessResult.empty()){
            ROS_ERROR("Received empty fitness result");
            validSolution = false;
        }

        // A valid solution could not be found:
        if (!validSolution || fitnessResult.empty()) {
            if (automatedRun()) {
                if (retryCounter < 5) {
                    fprintf(logOutput, "Retrying\n");

                    if (ros::Time::isSimTime()){
                        sleep(5);
                        resetSimulation();
                        sleep(5);
                    }

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
                    if (ros::Time::isSimTime()){
                        sleep(5);
                        resetSimulation();
                        sleep(5);
                    }

                    fprintf(logOutput, "Retrying\n");
                    currentIndividual--;
                    validSolution = false;
                }
            }
        }


    } while (validSolution == false);

    FILE *genFile = fopen("generation", "r");

    int currentGeneration;
    fscanf(genFile, "%d", &currentGeneration);
    fclose(genFile);

    if (currentIndividual != 0) {
        fprintf(logFile, ",\n");
    }

    fprintf(logFile, "    {\n");

    fprintf(logFile, "      \"id\": %d,\n", currentIndividual);
    fprintf(logFile, "      \"generation\": %d,\n", currentGeneration);

    fprintf(logFile, "      \"genotype\": [\n");
    for (int i = 0; i < givenIndividualGenotype.size(); i++) {
        fprintf(logFile, "        %f", givenIndividualGenotype[i]);
        if (i != givenIndividualGenotype.size() - 1) fprintf(logFile, ",");
        fprintf(logFile, "  \n");
    }
    fprintf(logFile, "      ],\n");

    fprintf(logFile, "      \"phenotype\": {\n");
    int i = 0;
    for(auto elem : individualParameters){
        fprintf(logFile, "        \"%s\": %f", elem.first.c_str(), elem.second);
        if (i != individualParameters.size()-1) fprintf(logFile, ",\n"); else fprintf(logFile, "\n");
        i++;
    }
    fprintf(logFile, "      },\n");

    fprintf(logFile, "      \"fitness\": {\n");
    i = 0;
    for(auto elem : fitnessResult){
        fprintf(logFile, "        \"%s\": %f", elem.first.c_str(), elem.second);
        if (i != fitnessResult.size()-1) fprintf(logFile, ",\n"); else fprintf(logFile, "\n");
        i++;
    }
    fprintf(logFile, "      },\n");

    fprintf(logFile, "      \"raw_fitness\": [\n");
    fprintf(logFile, "        {\n");
    printMap(rawFitness[0], "          ", logFile);

    if (rawFitness.size() > 1) {
        fprintf(logFile, "        },{\n");
        printMap(rawFitness[1], "          ", logFile);
    }

    fprintf(logFile, "        }\n");

    fprintf(logFile, "      ]\n");

    fprintf(logFile, "    }");

/*    if (currentIndividual != -1) { // If last individual
        fprintf(logFile, "\n");
        fprintf(logFile, "  ]\n");
        fprintf(logFile, "}");
    } else {
        fprintf(logFile, ",\n");
    }*/

    return fitnessResult;

}

void stopEa();
bool stopCondition();

SFERES_FITNESS (FitExp2MO, sferes::fit::Fitness)
{
public:
    FitExp2MO() {}

    template<typename Indiv>
    void eval(Indiv &ind) {

        this->_objs.resize(fitnessFunctions.size());

        std::vector<double> individualData(ind.gen().size());

        for (int i = 0; i < individualData.size(); i++) individualData[i] = ind.gen().data(i);

        FILE *evoLog = fopen(evoLogPath.c_str(), "a");
        if (evoLog == NULL) {
            ROS_ERROR("evoLog couldnt be opened (err%d)\n", errno);
        }

        std::map<std::string, double> fitnessResult = evaluateIndividual(individualData, evoLog);

        // Run at end of generations:
        if ((currentIndividual != 0) && ((currentIndividual+1) != popSize) &&  ((currentIndividual+1) % (popSize)) == 0){
            if(currentIndividual == ((popSize * generations)-1)) { // Check for end without stop condition
                fprintf(stdout, "Printing at end of run (individual %d)!\n", currentIndividual);
                fprintf(evoLog, "\n");
                fprintf(evoLog, "  ]\n");
                fprintf(evoLog, "}");
            } else if (stopCondition()){
                stopEa();

                fprintf(stdout, "Printing at stop condition (individual %d)!\n", currentIndividual);
                fprintf(stdout, "currentIndividual: %d, popSize: %d\n", currentIndividual, popSize);

                fprintf(evoLog, "\n");
                fprintf(evoLog, "  ]\n");
                fprintf(evoLog, "}");
            }
        }


        //fprintf(stderr, "%u, %u\n", currentIndividual, ((popSize * generations)-1));


        fclose(evoLog);

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

        FILE *evoLog = fopen(evoLogPath.c_str(), "a");
        if (evoLog == NULL) {
            ROS_ERROR("evoLog couldnt be opened (err%d)\n", errno);
        }

        std::map<std::string, double> fitnessResult = evaluateIndividual(individualData, evoLog);

        fclose(evoLog);

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

void stopEa(){
    sferes_nsga2::ea.stop();
}

int generationsWithoutImprovement = 0;
bool stopCondition(){

    if (!useStopCondition) return false;

    if (currentIndividual > popSize*2){

        printf("\nOld optimal parents:\n");
        for (int i = 0; i < lastOptimalParents.size(); i++) {
            printf("  ");
            for (int j = 0; j < lastOptimalParents[i].size(); j++){
                printf("%.2f, ", lastOptimalParents[i][j]);
            }
            printf("\n");
        }

        bool foundNew = false;

        printf("\nNew optimal parents:\n");
        for (int i = 0; i < sferes_nsga2::ea.pop().size(); i++) {
            if (sferes_nsga2::ea.parent_pop()[i]->rank() == 0) {
                printf("  ");
                for (int j = 0; j < sferes_nsga2::ea.parent_pop()[i]->data().size(); j++){
                    printf("%.2f, ", sferes_nsga2::ea.parent_pop()[i]->data()[j]);
                }

                bool alreadyExists = false;
                for (int j = 0; j < lastOptimalParents.size(); j++) {
                    if (sferes_nsga2::ea.parent_pop()[i]->data() == lastOptimalParents[j]){
                        alreadyExists = true;
                        printf(" *found*");
                    }
                }

                if (!alreadyExists){
                    foundNew = true;
                    printf(" *new*");
                }

                printf("\n");

            }
        }

        if (foundNew){
            generationsWithoutImprovement = 0;
        } else {
            generationsWithoutImprovement += 1;

            int limit = ceil(evalsWithoutImprovement / popSize);

            printf("  No improvement, counter now at %d / %d\n", generationsWithoutImprovement, limit);

            if (generationsWithoutImprovement >= limit){
                return true;
            }
        }

    }

    // Save lastOptimalParents
    lastOptimalParents.clear();
    for (int i = 0; i < sferes_nsga2::ea.parent_pop().size(); i++){
        if (sferes_nsga2::ea.parent_pop()[i]->rank() == 0) {
            lastOptimalParents.push_back(sferes_nsga2::ea.parent_pop()[i]->data());
        }
    }

    return false;
}

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

    fprintf(logOutput, "    ts - test hard coded robot\n"
                       "    ss - Test small robot (small HLSC)\n"
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
        if (choice == "ts") {

        std::map<std::string, double> customRobot =
                {{"femurLength", 0.0}, //{{"femurLength", 1.367467},
                 {"frequencyFactor", 0.98675},
                 {"liftDuration", 0.159514},
                 {"p0_x", 0.0},
                 {"p0_y", 102.399856},
                 {"p1_x", 0.0},
                 {"p1_y", 55.270207},
                 {"p2_x", 0.0},
                 {"p2_y", -125.971046},
                 {"p2_z", 69.163904},
                 {"p3_x", 0.0},
                 {"p3_y", 43.958044},
                 {"p3_z", 49.316892},
                 {"p4_x", 0.0},
                 {"p4_y", -4.313314},
                 {"p4_z", 16.276393},
                 {"tibiaLength", 0.0}}; //{"tibiaLength", 48.817356}};

            run_individual("lowLevelSplineGait", customRobot);
        } else if (choice == "ss") {
            run_individual("highLevelSplineGait", individuals::smallRobotSmallControl);
        } else if (choice == "ls") {
            run_individual("highLevelSplineGait", individuals::largeRobotSmallControl);
        } else if (choice == "ll") {
            run_individual("highLevelSplineGait", individuals::largeRobotLargeControl);
        } else if (choice == "cs") {

            std::vector<std::string> names = {"difficultyFactor", "liftDuration", "frequency", "femurLength", "tibiaLength", "wagPhase", "wagAmplitude_x", "wagAmplitude_y",
                                              "p0_x", "p0_y", "p1_x", "p1_y", "p2_x", "p2_y", "p2_z", "p3_x", "p3_y", "p3_z", "p4_x", "p4_y", "p4_z"};
            std::vector<float> values = {0.42, 0.20, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                         0.0,  92.5,
                                         0.0, -92.5,
                                         -25.0, -92.5, 50.0,
                                         -100.0,   0.0, 75.00,
                                         -25.0, 142.5, 18.75};
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

    ss << getenv("HOME") << "/catkin_ws/experimentResults/" << getDateString(givenTime) << "_" << prefix << "_" << std::setfill('0') << std::setw(3) << int(gaitDifficultyFactor*100.0) << "/";

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
        ss << experimentDirectory.c_str() << getDateString(now) << "_" << givenAlgorithm << "_" << std::setfill('0') << std::setw(3) << int(gaitDifficultyFactor*100.0) << ".json";

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
        fprintf(evoLog, "    \"gaitDifficultyFactor\": \"%.1f\",\n", gaitDifficultyFactor);
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
            fprintf(logOutput, "        \"%s\": %f", elem.first.c_str(), elem.second);
            if (i != gaitResults.size()-1) fprintf(logOutput, ",\n"); else fprintf(logOutput, "\n");
            i++;
        }

    }
}

std::vector<double> getRandomIndividual() {
    std::vector<double> individual(20);

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
        fprintf(randomSearchLog, "    \"seed\": \"%u\",\n", randomSeed);
        fprintf(randomSearchLog, "    \"controller\": \"%s\",\n", gaitType.c_str());
        fprintf(randomSearchLog, "    \"gaitDifficultyFactor\": \"%.1f\",\n", gaitDifficultyFactor);
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

        currentIndividual = -1;

        for (int j = 0; j < popSize * (generations); j++) {

            // Generate random individual:
            std::vector<double> randomIndividual = getRandomIndividual();

            std::map<std::string, double> fitnessResult = evaluateIndividual(randomIndividual, randomSearchLog);
        }

        fclose(randomSearchLog);

        fprintf(logOutput, "Experiment finished. Log written to:\n  %s\n", ss.str().c_str());
    }
}

void getDifficultyFactor(){
  printf("Difficulty?> ");

  std::string difficulty;

  if (commandQueue.empty()) {
    getline(std::cin, difficulty);
  } else {
    difficulty = commandQueue[0];
    fprintf(logOutput, "*%s*\n", difficulty.c_str());
    commandQueue.erase(commandQueue.begin());
  }

  if(difficulty.find_first_not_of("1234567890.-") != std::string::npos){
    ROS_FATAL("Invalid difficulty factor input");
    exit(-1);
  }

  gaitDifficultyFactor = atof(difficulty.c_str());
}

void menu_experiments() {
    std::string choice;

    std::cout << "  Please choose one experiment: (enter to go back)\n";

    fprintf(logOutput,
            "    Evolution:\n"
            "      cs - evolve control, small morphology, highLevel\n"
            "      cm - evolve control, medium morphology, highLevel\n"
            "      cl - evolve control, large morphology, highLevel\n"
            "      mn - evolve cont+morph, highLevel\n"
            "      me - evolve map-elites, highLevel\n"
            "      el - evolve cont+morph, lowLevel\n"
            "    Random:\n"
            "      rh - random search, highLevel\n"
            "      rl - random search, lowLevel\n"
            "      vf - verify fitness on single individual\n"
            "    Misc:\n"
            "      vn - check noise in stability fitness\n");
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
            getDifficultyFactor();

            experiments_evolve("nsga2", "", "lowLevelSplineGait", true);
        } else if (choice == "me") {
            experiments_evolve("map-elites", "", "highLevelSplineGait", true);
        } else if (choice == "vf") {
            experiments_verifyFitness();
        } else if (choice == "vn") {
            experiments_fitnessNoise();
        } else if (choice == "rl") {
            gaitType = "lowLevelSplineGait";

            getDifficultyFactor();

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
    fprintf(logOutput, "    q - set random seed\n");
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
        } else if (choice == "q") {
            randomSeed = 0;
            srand(randomSeed);
        }

    }
    fprintf(logOutput, "\n");
}

int main(int argc, char **argv) {

    randomSeed = time(NULL);

    srand(randomSeed);

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
    gaitConfiguration_client = rch.serviceClient<dyret_controller::ConfigureGait>("/dyret/dyret_controller/gaitConfigurationService");
    gaitCommandService_client = rch.serviceClient<dyret_controller::GaitControllerCommandService>("/dyret/dyret_controller/gaitControllerCommandService");
    inferredPositionClient = rch.serviceClient<dyret_controller::GetInferredPosition>("/dyret/dyret_controller/getInferredPosition");

    servoConfigClient = rch.serviceClient<dyret_common::Configure>("/dyret/configuration");
    get_gait_evaluation_client = rch.serviceClient<dyret_controller::GetGaitEvaluation>("get_gait_evaluation");
    gaitControllerStatus_client = rch.serviceClient<dyret_controller::GetGaitControllerStatus>(
            "get_gait_controller_status");

    poseCommand_pub = rch.advertise<dyret_common::Pose>("/dyret/command", 10);
    dyretState_sub = rch.subscribe("/dyret/state", 1, dyretStateCallback);

    waitForRosInit(get_gait_evaluation_client, "get_gait_evaluation");
    waitForRosInit(gaitControllerStatus_client, "gaitControllerStatus");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string evoInfo = "testInfo";

    if (resetGaitRecording(get_gait_evaluation_client) == false) {
        spinner.stop();
        ros::shutdown();
        exit(-2);
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
        } else if (commandQueue.empty() && automatedRun()) {
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
