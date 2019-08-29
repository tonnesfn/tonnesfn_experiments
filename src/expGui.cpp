#include <iostream>
#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <unistd.h>
#include <stdio.h>
#include <cassert>

#include "ros/ros.h"
#include "ros/package.h"

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include "dyret_common/Configuration.h"
#include "dyret_common/Configure.h"
#include "dyret_common/Pose.h"
#include "dyret_common/State.h"

#include "dyret_common/wait_for_ros.h"

#include "dyret_controller/GetGaitEvaluation.h"
#include "dyret_controller/GetGaitControllerStatus.h"
#include "dyret_controller/ConfigureGait.h"
#include "dyret_controller/DistAngMeasurement.h"
#include "dyret_controller/GetInferredPosition.h"
#include "dyret_controller/GaitControllerCommandService.h"
#include "dyret_controller/LoggerCommand.h"

#include "camera_recorder/Configure.h"
#include "camera_recorder/Record.h"

#include "dyret_hardware/ActuatorBoardState.h"

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
#include "individuals_journal2019.h"

#include "evoSettings.h"

#include "expFunctions.h"

ros::ServiceClient servoConfigClient;
ros::ServiceClient inferredPositionClient;
ros::ServiceClient get_gait_evaluation_client;
ros::ServiceClient gaitControllerStatus_client;
ros::ServiceClient servoStatus_client;
ros::ServiceClient gaitConfiguration_client;
ros::ServiceClient gaitCommandService_client;
ros::ServiceClient loggerCommandService_client;
ros::Subscriber dyretState_sub;
ros::Subscriber actuator_boardState_sub;
ros::Subscriber gaitInferredPos_sub;
ros::Publisher poseCommand_pub;

gazebo::WorldConnection* gz;

// Configuration:
const bool skipReverseEvaluation = true; // Only evaluate forwards, not back again
int numberOfEvalsInTesting = 1;

const bool useStopCondition = false;    // Use stop condition in evolution
const int evalsWithoutImprovement = 64; // Number of individuals without improvement before evolution is stopped

const std::string resumeFile = ""; // File to resume. Does not resume if empty

const bool useActionMessageInSim = true; // Use action message (or manual stepping) in simulation
const bool skipSimulationReset = true;   // Skip reseting simulation between evaluations

const int verificationEvals = 20;

bool cooldownPromptEnabled = true; // Prompt for cooldown between each generation in hardware

//

std::array<int, 8> prismaticActuatorStates;

bool promptForConfirmation = false; // Prompt for confirmation after each evaluation
bool currentlyLoggingFitness = false; // Whether gaitEvaluator is logging fitness or not

std::vector<float> restPose = {0.1839425265789032, 0.7079652547836304, -1.1992725133895874, -0.1839425265789032, 0.7079652547836304, -1.1992725133895874, -0.1839425265789032, -0.7079652547836304, 1.1992725133895874, 0.1839425265789032, -0.7079652547836304, 1.1992725133895874};

unsigned int randomSeed;

double currentFemurLength = 0.0;
double currentTibiaLength = 0.0;
int evaluationTimeout = 10;
float evaluationDistance = 1000.0;
int currentIndividual;
std::array<float, 12> servoTemperatures;

std::string logDirectoryPath;

float gaitDifficultyFactor = 0.5;

bool instantFitness = false;

float frequencyFactor = 1.0;

std::string morphology;
std::string gaitType;

bool robotOnStand = false;

bool enableLogging = true;

char **argv_g;

std::vector<std::vector<float>> lastOptimalParents;

std::vector<std::string> fitnessFunctions; // Used to specify which fitness functions to use
std::vector<std::string> commandQueue; // Used to store commands from the arguments temporarily
std::string fullCommand; // Used to store commands from the arguments permanently

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
    if (ros::Time::isSimTime() && !skipSimulationReset){
        resetSimulation(gz);
        usleep(1000);
    }

    // Code to stop for cooldown at the start of each new generation:
    if (ros::Time::isSystemTime() && cooldownPromptEnabled && (currentIndividual % popSize == 0) && (currentIndividual != 0)) {
        playSound("info_generationdone");
        cooldownServos(servoConfigClient, servoTemperatures, poseCommand_pub, restPose);
    }

    printf("\n  %03u: Evaluating individual:\n", currentIndividual);
    printMap(phenoType, "    ");

    // Check servo temperature and consider fitness invalid if too high (due to DC motor characterics)
    if (ros::Time::isSystemTime()) { // Only check temperature in real world
        if (getMaxServoTemperature(servoTemperatures) > 70.0) {
            ROS_ERROR("  Temperature is too high at %.1f\n", getMaxServoTemperature(servoTemperatures));
            return std::map<std::string, double>();
        }
    }

    std::string logPath = "";
    if (logDirectoryPath != ""){
        logPath = logDirectoryPath.substr(0, logDirectoryPath.find_last_of("\\/")) + "/splines/";
        mkdir(logPath.c_str(), 0700);
    }

    // Set gait parameters
    if (!(ros::Time::isSystemTime() || useActionMessageInSim)) unpauseGazebo();

    std::vector<float> femurLengths;
    if (phenoType.count("femurLength")){
        femurLengths = {(float) phenoType.at("femurLength")};
    } else {
        femurLengths = {(float) phenoType.at("femurLength_front"), (float) phenoType.at("femurLength_front"), (float) phenoType.at("femurLength_rear"), (float) phenoType.at("femurLength_rear")};
    }

    std::vector<float> tibiaLengths;
    if (phenoType.count("tibiaLength")){
        tibiaLengths = {(float) phenoType.at("tibiaLength")};
    } else {
        tibiaLengths = {(float) phenoType.at("tibiaLength_front"), (float) phenoType.at("tibiaLength_front"), (float) phenoType.at("tibiaLength_rear"), (float) phenoType.at("tibiaLength_rear")};
    }

    setGaitParams(gaitType, logPath + std::to_string(currentIndividual), true, true, false, femurLengths, tibiaLengths, phenoType, gaitConfiguration_client);

    if (!(ros::Time::isSystemTime() || useActionMessageInSim)) {
        ros::spinOnce();
        pauseGazebo();
        ros::spinOnce();
    }

    //start
    playSound("beep_low");
    getInputFromTerminal("  Press enter when ready");

    // Run gait
    if (ros::Time::isSystemTime() || useActionMessageInSim) {
        runGaitControllerWithActionMessage(true,
                                           currentIndividual,
                                           get_gait_evaluation_client,
                                           loggerCommandService_client,
                                           gaitCommandService_client,
                                           inferredPositionClient,
                                           enableLogging,
                                           evaluationTimeout,
                                           evaluationDistance,
                                           logDirectoryPath);

    } else {
        gz->step(100);
        runGaitWithServiceCalls(evaluationDistance, evaluationTimeout, gz, get_gait_evaluation_client, inferredPositionClient, gaitCommandService_client);
    }

    // Get results from gaitEvaluator
    std::map<std::string, double> gaitResultsForward = getGaitResults(get_gait_evaluation_client);

    // Check for empty results
    if (gaitResultsForward.empty()) {
        ROS_ERROR("GaitResultsForward.size() == 0!");
        return std::map<std::string, double>();
    }

    // Check for nan values
    for(const auto elem : gaitResultsForward){
        if (std::isnan(elem.second)){
            ROS_ERROR("Got NAN value for %s", elem.first.c_str());

            for(const auto elem2 : gaitResultsForward){
                fprintf(stderr, "  %s: %.2f\n", elem2.first.c_str(), elem2.second);
            }

            return std::map<std::string, double>();
        }
    }

    // Print forward results
    printf("  Res F:\n");
    printMap(gaitResultsForward, "    ");

    std::map<std::string, double> gaitResultsReverse;

    if (!skipReverseEvaluation) {

        ROS_INFO("Evaluating reverse");

        if (ros::Time::isSimTime() && !skipSimulationReset){
            resetSimulation(gz);
            usleep(1000);
        }

        // Set gait parameters
        if (!(ros::Time::isSystemTime() || useActionMessageInSim)) unpauseGazebo();
        setGaitParams(gaitType, "", false, true, false, femurLengths, tibiaLengths, phenoType, gaitConfiguration_client);

        if (!(ros::Time::isSystemTime() || useActionMessageInSim)) {
            ros::spinOnce();
            pauseGazebo();
            ros::spinOnce();
        }

        resetGaitRecording(get_gait_evaluation_client);

        // Run gait
        if (ros::Time::isSystemTime() || useActionMessageInSim) {
            runGaitControllerWithActionMessage(false,
                                               currentIndividual,
                                               get_gait_evaluation_client,
                                               loggerCommandService_client,
                                               gaitCommandService_client,
                                               inferredPositionClient,
                                               enableLogging,
                                               evaluationTimeout,
                                               evaluationDistance,
                                               logDirectoryPath);
        } else {
            gz->step(100);
            runGaitWithServiceCalls(evaluationDistance, evaluationTimeout, gz, get_gait_evaluation_client, inferredPositionClient, gaitCommandService_client);
        }

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
        printf("  Res R:\n");
        printMap(gaitResultsReverse, "    ");
    }

    // Handle robots that fell
    if ((gaitResultsForward.at("linAcc_z") > 0) || ((!gaitResultsReverse.empty()) && (gaitResultsReverse.at("linAcc_z") > 0))){
        if (gaitResultsReverse.empty()) {
            mapToReturn["Stability"] = fmax(-1.0, gaitResultsForward["combImuStab"]);
        } else {
            mapToReturn["Stability"] = fmax(-1.0, (gaitResultsForward["combImuStab"] + gaitResultsReverse["combImuStab"]) / 2.0);
        }

        if (ros::Time::isSimTime() && !skipSimulationReset){
            mapToReturn["MocapSpeed"] = getMapValue(gaitResultsForward, "sensorSpeedForward");
        } else {
            if (gaitResultsReverse.empty()) {
                mapToReturn["MocapSpeed"] = gaitResultsForward["filteredSpeed"];
            } else {
                mapToReturn["MocapSpeed"] = (gaitResultsForward["filteredSpeed"] + gaitResultsReverse["filteredSpeed"]) / 2.0;
            }
        }
    } else {
        ROS_WARN("Robot fell, discarding fitness");

        mapToReturn["Stability"] = -1;
        mapToReturn["MocapSpeed"] = 0;

        if (ros::Time::isSimTime()){
            resetSimulation(gz);
            ROS_INFO("Reseting simulation due to fall");
        }

    }

    // Print total results
    printf("  Res total: \n");
    printMap(mapToReturn, "    ");

    if (mapToReturn["MocapSpeed"] == 0.0){
        ROS_WARN("MocapSpeed 0");
        playSound("warning_mocap");
    }

    printf("\n");

    // Add raw fitnesses to container
    rawFitnesses.push_back(gaitResultsForward);
    if (!gaitResultsReverse.empty()){
        rawFitnesses.push_back(gaitResultsReverse);
    }

    return mapToReturn;

}

void dyretStateCallback(const dyret_common::State::ConstPtr &msg) {
    currentFemurLength = (msg->prismatic[0].position + msg->prismatic[2].position + msg->prismatic[4].position +
                          msg->prismatic[6].position) / 4.0;
    currentTibiaLength = (msg->prismatic[1].position + msg->prismatic[3].position + msg->prismatic[5].position +
                          msg->prismatic[7].position) / 4.0;

    for (int i = 0; i < msg->revolute.size(); i++){
        servoTemperatures[i] = msg->revolute[i].temperature;
    }
}

void actuator_boardStateCallback(const dyret_hardware::ActuatorBoardState::ConstPtr &msg) {
    for (int i = 0; i < 8; i++) prismaticActuatorStates[i] = msg->status[i];
}

void stopEa();
bool stopCondition();

std::map<std::string, double> evaluateIndividual(std::vector<double> givenIndividualGenotype, FILE *logFile) {

    std::map<std::string, double> individualParameters;
    if (gaitType == "highLevelSplineGait") {
        individualParameters = genToHighLevelSplineGaitPhen(givenIndividualGenotype, frequencyFactor);
    } else if (gaitType == "lowLevelSplineGait"){
        individualParameters = genToLowLevelSplineGaitPhen(givenIndividualGenotype, frequencyFactor, gaitDifficultyFactor);
    } else if (gaitType == "lowLevelAdvancedSplineGait"){
        individualParameters = genToLowLevelAdvancedSplineGaitPhen(givenIndividualGenotype, frequencyFactor, gaitDifficultyFactor);
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

        // Check after each eval if enabled. If not - do regular checks
        if (promptForConfirmation) {
            //printf("  Select action: continue (enter), retry (r), discard (b), restart servos (d):\n");

            std::string input = getInputFromTerminal("  Action: continue (enter), retry (r), discard (b), restart servos (d), zero linear actuators (z):");

            if (input == "r"){
                validSolution = false;
                currentIndividual--;
                ROS_WARN("Retrying individual");
            } else if (input == "b"){
                fitnessResult["Stability"] = -1;
                fitnessResult["MocapSpeed"] = 0;
                ROS_WARN("Discarding individual");
            } else if (input == "d"){
                restartServos(servoConfigClient);
                ROS_WARN("Servos restarted");
                sleep(10);
                validSolution = false;
                currentIndividual--;
                printf("  Press enter when ready to retry");
                std::getline(std::cin, input);
            } else if (input == "z"){
                printf("  Zeroing actuators\n");
                zeroPrismaticActuators(false, poseCommand_pub, prismaticActuatorStates);
            } else if (input.length() > 0){
                ROS_WARN("Unknown input! Retrying");
            }
        } else {

            if (fitnessResult.empty()) {
                ROS_ERROR("Received empty fitness result");
                validSolution = false;
            }

            // A valid solution could not be found:
            if (!validSolution || fitnessResult.empty()) {
                if (!fullCommand.empty()) {
                    if (retryCounter < 5) {
                        printf("Retrying\n");
                        ROS_WARN("Retrying");

                        retryCounter += 1;
                        currentIndividual--;
                        validSolution = false;
                    } else {
                        printf("ABORT - after three retries without results\n");
                        exit(-1);
                    }
                } else if (!ros::ok()) {
                    ros::shutdown();
                    exit(-1);
                    break;
                } else {
                    printf("Got invalid fitness: choose action ((r)etry/(d)iscard: ");

                    std::string choice;
                    getline(std::cin, choice);

                    if (choice == "d") { // Discard
                        fitnessResult.clear();

                        printf("Discarding\n");
                        validSolution = true;
                    } else { // Retry
                        if (ros::Time::isSimTime()) {
                            sleep(5);
                            resetSimulation(gz);
                            sleep(5);
                        }

                        printf("Retrying\n");
                        currentIndividual--;
                        validSolution = false;
                    }
                }
            }
        }
    } while (!validSolution);

    FILE *genFile = fopen("generation", "r");

    int currentGeneration;
    fscanf(genFile, "%d", &currentGeneration);
    fclose(genFile);

    if (currentIndividual != 0) {
        printf(",\n");
    }

    printf("    {\n");

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

        FILE *evoLog = fopen(logDirectoryPath.c_str(), "a");
        if (evoLog == NULL) {
            ROS_ERROR("evoLog couldnt be opened (err%d)\n", errno);
        }

        std::map<std::string, double> fitnessResult = evaluateIndividual(individualData, evoLog);

        // Run at end of generations:
        if ((currentIndividual != 0) && ((currentIndividual+1) != popSize) &&  ((currentIndividual+1) % (popSize)) == 0){
            if(currentIndividual == ((popSize * generations)-1)) { // Check for end without stop condition
                printf("Printing at end of run (individual %d)!\n", currentIndividual);
                fprintf(evoLog, "\n");
                fprintf(evoLog, "  ]\n");
                fprintf(evoLog, "}");
            } else if (stopCondition()){
                stopEa();

                printf("Printing at stop condition (individual %d)!\n", currentIndividual);
                printf("currentIndividual: %d, popSize: %d\n", currentIndividual, popSize);

                fprintf(evoLog, "\n");
                fprintf(evoLog, "  ]\n");
                fprintf(evoLog, "}");
            }
        }

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

        FILE *evoLog = fopen(logDirectoryPath.c_str(), "a");
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

std::vector<std::map<std::string, double>> run_individual(std::string givenGaitType, std::map<std::string, double> givenPhenoTypeMap) {

    std::vector<std::map<std::string, double>> vectorToReturn;

    gaitType = givenGaitType;

    fitnessFunctions.clear();
    fitnessFunctions.emplace_back("MocapSpeed");
    fitnessFunctions.emplace_back("Stability");

    for (int i = 0; i < numberOfEvalsInTesting; i++) {
        std::vector<std::map<std::string, double>> rawFitnesses;
        vectorToReturn.push_back(getFitness(givenPhenoTypeMap, get_gait_evaluation_client, rawFitnesses));
    }

    return vectorToReturn;
}

void menu_demo() {
    enableLogging = false;

    std::string choice;

    std::cout << "  Please choose one demonstration: (enter to go back)\n";

    printf("    ss - Test small robot (small HLSC)\n"
           "    ls - Test large robot (small HLSC)\n"
           "    ll - Test large robot (large HLSC)\n"
           "    rz - Test zero robot (LLSC)\n"
           "    rm - Test medium robot (LLSC)\n"
           "    rd - Test doubleUneven robot (LLASC)\n"
           "    ru - Test uneven robot (LLASC)\n"
           "    rf - Test uneven robot sim (leaning front) (LLASC)\n"
           "    rb - Test uneven robot sim (leaning back) (LLASC)\n"
           "    ms - Request small morphology\n"
           "    mx - Request 10mm morphology\n"
           "    mm - Request medium morphology\n"
           "    ml - Request large morphology\n"
           "    mt - Test morphology changes\n"
           "    muf - Request uneven length morphology (forward)\n"
           "    mub - Request uneven length morphology (backward)\n"
           "    cs - Start camera\n"
           "    ch - Stop camera\n"
           "    b - Test beeps\n");
    printf("\n> ");

    if (commandQueue.empty()) {
        getline(std::cin, choice);
    } else {
        choice = commandQueue[0];
        printf("*%s*\n", choice.c_str());
        commandQueue.erase(commandQueue.begin());
    }

    if (choice.empty() == false) {
        if (choice == "ss") {
            run_individual("highLevelSplineGait", individuals_highLevelSplineGait::smallRobotSmallControl);
        } else if (choice == "ls") {
            run_individual("highLevelSplineGait", individuals_highLevelSplineGait::largeRobotSmallControl);
        } else if (choice == "ll") {
            run_individual("highLevelSplineGait", individuals_highLevelSplineGait::largeRobotLargeControl);
        } else if (choice == "rz") {
            run_individual("lowLevelSplineGait", individuals_lowLevelSplineGait::zeroHeight);
        } else if (choice == "rm") {
            run_individual("lowLevelSplineGait", individuals_lowLevelSplineGait::mediumHeight);
        } else if (choice == "rd") {
            run_individual("lowLevelAdvancedSplineGait", individuals_lowLevelAdvancedSplineGait::doubleUneven);
        } else if (choice == "ru") {
            run_individual("lowLevelAdvancedSplineGait", individuals_lowLevelAdvancedSplineGait::unevenSmallFrontLeaning);
        } else if (choice == "rf") {
            run_individual("lowLevelAdvancedSplineGait", individuals_lowLevelAdvancedSplineGait::unevenLargeFrontLeaning);
        } else if (choice == "rb") {
           run_individual("lowLevelAdvancedSplineGait", individuals_lowLevelAdvancedSplineGait::unevenLargeBackLeaning);
        } else if (choice == "ms") {
            setLegLengths(0.0, 0.0, poseCommand_pub);
            printf("Small morphology requested\n");
        } else if (choice == "mub") {
            setLegLengths(std::vector<float>{40, 40, 40, 40, 5, 5, 5, 5}, poseCommand_pub);
            printf("Uneven length morphology requested\n");
        } else if (choice == "muf") {
            setLegLengths(std::vector<float>{5, 5, 5, 5, 40, 40, 40, 40}, poseCommand_pub);
            printf("Uneven length morphology requested\n");
        } else if (choice == "mx") {
            setLegLengths(10.0, 10.0, poseCommand_pub);
            printf("10mm morphology requested\n");
        } else if (choice == "mm") {
            setLegLengths(25.0, 47.5, poseCommand_pub);
            printf("Medium morphology requested\n");
        } else if (choice == "ml") {
            setLegLengths(50.0, 95.0, poseCommand_pub);
            printf("Large morphology requested\n");
        }else if (choice == "mt") {
            while(ros::ok()) {
              setLegLengths(10.0, 10.0, poseCommand_pub);
              printf("  Leg lengths set to 10/10\n");
              sleep(1);
              while (!legsAtRest(prismaticActuatorStates) && ros::ok()) {}
              sleep(1);
              setLegLengths(5.0, 5.0, poseCommand_pub);
              printf("  Leg lengths set to 5/5\n");
              sleep(1);
              while (!legsAtRest(prismaticActuatorStates) && ros::ok()) {}
              sleep(1);
            }
        } else if (choice == "cs") { // Start camera
          printf("Start camera:\n");
          startVideo("/home/tonnesfn/test.mp4");
        } else if (choice == "ch") { // Stop camera
          printf("Stop camera:\n");
          stopVideo();
        } else if (choice == "b"){ // Test beep
          playSound("beep_low", 3);
        }
    }
    enableLogging = true;
};

void experiments_evolve(const std::string givenAlgorithm, const std::string givenMorphology, const std::string givenController) {
    if (givenAlgorithm != "map-elites" && givenAlgorithm != "nsga2") {
        ROS_ERROR("Unknown evolution algorithm: %s", givenAlgorithm.c_str());
    }

    morphology = givenMorphology;
    gaitType = givenController;
    currentIndividual = -1;

    fitnessFunctions.clear();
    fitnessFunctions.emplace_back("Stability");
    if (givenAlgorithm == "nsga2") fitnessFunctions.emplace_back("MocapSpeed");

    if (!(ros::Time::isSystemTime() || useActionMessageInSim)) {
        pauseGazebo();
    }

    currentIndividual = -1;

    time_t t = time(0);   // get time now
    struct tm *now = localtime(&t);

    std::string experimentDirectory = createExperimentDirectory(givenAlgorithm, now);

    std::stringstream ss;
    ss << experimentDirectory.c_str() << getDateString(now) << "_" << givenAlgorithm << ".json";

    logDirectoryPath = ss.str();

    FILE *evoLog = fopen(logDirectoryPath.c_str(), "a");
    if (evoLog == NULL) {
        ROS_ERROR("evoLog could not be opened (err%d)\n", errno);
    }

    writeVersionLog(experimentDirectory);

    char hostname[1024];
    gethostname(hostname, 1024);

    fprintf(evoLog, "{\n");
    fprintf(evoLog, "  \"experiment_info\": {\n");
    fprintf(evoLog, "    \"time\": \"%s\",\n", getDateString(now).c_str());
    if (fullCommand.size() != 0) fprintf(evoLog, "    \"command\": \"%s\",\n", trim(fullCommand).c_str());
    fprintf(evoLog, "    \"type\": \"evolution\",\n");
    fprintf(evoLog, "    \"algorithm\": \"%s\",\n", givenAlgorithm.c_str());
    fprintf(evoLog, "    \"controller\": \"%s\",\n", givenController.c_str());
    fprintf(evoLog, "    \"gaitDifficultyFactor\": \"%3f\",\n", gaitDifficultyFactor);
    fprintf(evoLog, "    \"evaluationTimeout\": \"%d\",\n", evaluationTimeout);
    fprintf(evoLog, "    \"evaluationDistance\": \"%.0f\",\n", evaluationDistance);
    fprintf(evoLog, "    \"machine\": \"%s\",\n", hostname);
    fprintf(evoLog, "    \"user\": \"%s\",\n", getenv("USER"));

    if (ros::Time::isSimTime()) fprintf(evoLog, "    \"platform\": \"simulation\",\n");
    else
        fprintf(evoLog, "    \"platform\": \"hardware\",\n");
    fprintf(evoLog, "    \"generations\": %d,\n", generations);
    fprintf(evoLog, "    \"population\": %d,\n", popSize);

    fprintf(evoLog, "    \"morphology\": \"*evolved*\",\n");

    fprintf(evoLog, "    \"fitness\": [\n");
    if (!instantFitness) {
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
    int argc_tmp;
    char *argv_tmp[5];
    argv_tmp[0] = argv_g[0]; // Copy the first reference

    if (resumeFile.empty()) {
        argc_tmp = 2;
        argv_tmp[2] = 0;
    } else {
        argc_tmp = 4;
        argv_tmp[4] = 0;
    }

    mkdir(std::string(experimentDirectory + "video").c_str(), 0700);

    mkdir(std::string(experimentDirectory + "sferes").c_str(), 0700);

    std::string commString = "-d" + experimentDirectory + "sferes";
    const char *sferesCommand = commString.c_str();
    argv_tmp[1] = const_cast<char *>(sferesCommand);

    std::string resString = "--resume";
    const char *res = resString.c_str();
    argv_tmp[2] = const_cast<char *>(res);

    const char *resName = resumeFile.c_str();
    argv_tmp[3] = const_cast<char *>(resName);

    for (int i = 0; i < argc_tmp; i++){
        fprintf(stderr, " %s ", argv_tmp[i]);
    }
    fprintf(stderr, "\n");

    if (givenAlgorithm == "nsga2") sferes::run_ea(argc_tmp, argv_tmp, sferes_nsga2::ea, logDirectoryPath);
    else if (givenAlgorithm == "map-elites") sferes::run_ea(argc_tmp, argv_tmp, sferes_mapElites::ea, logDirectoryPath);
    else {
        ROS_FATAL("Invalid algorithm (%s) and controller (%s) choices", givenAlgorithm.c_str(), givenController.c_str());
        exit(-1);
    }
    logDirectoryPath.clear();

    printf("Experiment finished. Log written to:\n  %s\n", ss.str().c_str());

    if (!(ros::Time::isSystemTime() || useActionMessageInSim)) {
        unpauseGazebo();
    }

};

void experiments_verifyFitness() {
    currentIndividual = 0;
    cooldownPromptEnabled = false;

    std::cout << "  Which ID do you want to test? > ";

    int individualId;

    std::cin >> individualId;
    std::cin.ignore(10000, '\n');

    char originalSurface = individuals_journal2019_names[individualId].c_str()[20];

    std::stringstream ss;
    ss << getenv("HOME") << "/catkin_ws/experimentResults/verification/";

    mkdir(ss.str().c_str(), 0700);
    ss << individuals_journal2019_names[individualId] << "/";

    if (directoryExists(ss.str())){
      fprintf(stderr, "  Directory for individual %s already exists. Will not overwrite verify fitness experiment.\n", individuals_journal2019_names[individualId].c_str());
      return;
    }

    mkdir(ss.str().c_str(), 0700);
    ss << individuals_journal2019_names[individualId] << "_ver.txt";

    logDirectoryPath = ss.str();

    FILE *verifyLog = fopen(logDirectoryPath.c_str(), "a");
    if (verifyLog == NULL) {
        ROS_ERROR("verifyLog couldnt be opened (err%d)\n", errno);
    }

    fprintf(verifyLog,
        "{\n"
        "  \"experiment_info\": {\n"
        "    \"type\": \"verification\",\n"
    );
    fprintf(verifyLog, "    \"originalSpeed\": \"%f\",\n", individuals_journal2019[individualId]["originalSpeed"]);
    fprintf(verifyLog, "    \"originalStability\": \"%f\",\n", individuals_journal2019[individualId]["originalStability"]);
    fprintf(verifyLog, "    \"originalSurface\": \"%c\"\n", originalSurface);

    fprintf(verifyLog,
        "  },\n"
        "  \"individuals\": ["
    );

    fclose(verifyLog);

    gaitType = "lowLevelSplineGait";

    fitnessFunctions.clear();
    fitnessFunctions.emplace_back("MocapSpeed");
    fitnessFunctions.emplace_back("Stability");

    char currentSurface = originalSurface;

    for (int i = 0; i < verificationEvals*4; i++) {

        if (i == 0){
          if (originalSurface == 'b'){
            playSound("basic", 1);
            ROS_ERROR("Run robot on BASIC surface");
          } else if (originalSurface == 'f'){
            playSound("fuzzy", 1);
            ROS_ERROR("Run robot on FUZZY surface");
          } else {
            ROS_ERROR("Unknown original surface: %c", originalSurface);
          }
        } else if (i == verificationEvals){
          if (originalSurface == 'b'){
            currentSurface = 'f';
            playSound("fuzzy", 1);
            ROS_ERROR("Run robot on FUZZY surface");
          } else if (originalSurface == 'f'){
            currentSurface = 'b';
            playSound("basic", 1);
            ROS_ERROR("Run robot on BASIC surface");
          } else {
            ROS_ERROR("Unknown original surface: %c", originalSurface);
          }
        } else if (i == verificationEvals * 2){
          currentSurface = 'g';
          playSound("grass", 1);
          ROS_ERROR("Run robot on GRASS surface");
        } else if (i == verificationEvals * 3){
          currentSurface = 'r';
          playSound("rough", 1);
          ROS_ERROR("Run robot on ROUGH surface");
        }

        std::vector<std::map<std::string, double>> rawFitnesses;

        std::map<std::string, double> fitnessResult = getFitness(individuals_journal2019[individualId],
                                                                 get_gait_evaluation_client,
                                                                 rawFitnesses);

        printf("  Returned fitness (%lu): \n", fitnessResult.size());
        printMap(fitnessResult, "    ");

        // Save to file:
        verifyLog = fopen(logDirectoryPath.c_str(), "a");

        if (i != 0){
          fprintf(verifyLog, ",");
        }

        fprintf(verifyLog, "\n    {\n      \"id\": %d,\n", i);
        fprintf(verifyLog, "      \"surface\": \"%c\",\n", currentSurface);

        fprintf(verifyLog, "      \"fitness\": {\n");
        printMap(fitnessResult, "        ", verifyLog);
        fprintf(verifyLog, "      },\n");

        for (int j = 0; j < rawFitnesses.size(); j++) {
            fprintf(verifyLog, "      \"raw_fitness\": {\n");
            printMap(rawFitnesses[j], "        ", verifyLog);
            fprintf(verifyLog, "      }");
        }

        fprintf(verifyLog, "\n    }");

        fclose(verifyLog);
        printf("\n");
    }

    verifyLog = fopen(logDirectoryPath.c_str(), "a");
    fprintf(verifyLog, "\n  ]\n}");
    fclose(verifyLog);

    cooldownPromptEnabled = true;
}

void experiments_fitnessNoise() {
    for (int i = 0; i < 100; i++) {
        resetGaitRecording(get_gait_evaluation_client);
        startGaitRecording(get_gait_evaluation_client);

        sleep(30);

        std::map<std::string, double> gaitResults = getGaitResults(get_gait_evaluation_client);

        printf("  Res: ");
        for(auto elem : gaitResults){
            printf("        \"%s\": %f", elem.first.c_str(), elem.second);
            if (i != gaitResults.size()-1) printf(",\n"); else printf("\n");
            i++;
        }

    }
}

void experiments_randomSearch() {

    currentIndividual = 0;

    std::cout << "How many runs do you want to do? >";

    int numberOfTests;

    if (commandQueue.empty()) {
        std::cin >> numberOfTests;
        std::cin.ignore(10000, '\n');
        std::cin.clear();
    } else {
        numberOfTests = std::stoi(commandQueue[0]);
        printf("*%d*\n", numberOfTests);
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
        fprintf(randomSearchLog, "    \"gaitDifficultyFactor\": \"%3f\",\n", gaitDifficultyFactor);
        fprintf(randomSearchLog, "    \"evaluationTimeout\": \"%d\",\n", evaluationTimeout);
        fprintf(randomSearchLog, "    \"evaluationDistance\": \"%.0f\",\n", evaluationDistance);
        fprintf(randomSearchLog, "    \"machine\": \"%s\",\n", hostname);
        fprintf(randomSearchLog, "    \"user\": \"%s\",\n", getenv("USER"));
        if (ros::Time::isSimTime()) fprintf(randomSearchLog, "    \"platform\": \"simulation\",\n");
        else
            fprintf(randomSearchLog, "    \"platform\": \"hardware\",\n");
        fprintf(randomSearchLog, "    \"generations\": %d,\n", generations);
        fprintf(randomSearchLog, "    \"population\": %d,\n", popSize);

        fprintf(randomSearchLog, "    \"fitness\": [\n");
        if (!instantFitness) {
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

        fprintf(randomSearchLog, "\n]}\n");
        fclose(randomSearchLog);

        printf("Experiment finished. Log written to:\n  %s\n", ss.str().c_str());
    }
}

// TODO (optional): Fix reposition legs code?
// TODO (optional): Add raw fitness here?
// TODO: Add support for number of evals
void experiments_sensorWalking(){

    printf("  Label for recording: ");
    std::string label;
    getline(std::cin, label);
    printf("  Femur (0-9): ");
    int femurLengthInput;
    std::cin >> femurLengthInput;
    printf("  Tibia (0-9): ");
    int tibiaLengthInput;
    std::cin >> tibiaLengthInput;
    printf("  Frequency (0.0-1.5): ");
    float frequency;
    std::cin >> frequency;
    printf("  Number of evals: ");
    int numberOfEvals_old = numberOfEvalsInTesting;
    std::cin >> numberOfEvalsInTesting;
    std::cin.clear();
    std::cin.ignore(10000, '\n');

    // Get controller parameters
    static std::map<std::string, double> individual = individuals_lowLevelSplineGait::conservativeIndividual;

    // Calculate morphology parameters
    assert(femurLengthInput >= 0 && femurLengthInput <= 9);
    assert(tibiaLengthInput >= 0 && tibiaLengthInput <= 9);

    float femurLength = (femurLengthInput/10.0) * 50.0;
    float tibiaLength = (tibiaLengthInput/10.0) * 50.0;

    individual["femurLength"] = femurLength;
    individual["tibiaLength"] = tibiaLength;

    // Set speed
    assert(frequency >= 0.0 && frequency <= 1.5);
    individual["frequency"] = frequency;

    logDirectoryPath = makeSensorDataDirectories(label, femurLength, tibiaLength).c_str();

    std::vector<std::map<std::string, double>> fitnesses = run_individual("lowLevelSplineGait", individual);

    FILE *sensorLog = fopen(logDirectoryPath.c_str(), "a");
    if (sensorLog == NULL) {
        ROS_ERROR("sensorLog could not be opened (err%d)\n", errno);
    }

    std::string experimentDirectory = logDirectoryPath.substr(0, logDirectoryPath.find_last_of("\\/")) + "/";
    std::string timeString = logDirectoryPath.substr(logDirectoryPath.find_last_of("\\/") + 1);

    char hostname[1024];
    gethostname(hostname, 1024);

    fprintf(sensorLog, "{\n");
    fprintf(sensorLog, "  \"experiment_info\": {\n");
    fprintf(sensorLog, "    \"femurLength\": \"%f\",\n", femurLength);
    fprintf(sensorLog, "    \"tibiaLength\": \"%f\",\n", tibiaLength);
    fprintf(sensorLog, "    \"time\": \"%s\",\n", timeString.c_str());
    if (fullCommand.size() != 0) fprintf(sensorLog, "    \"command\": \"%s\",\n", trim(fullCommand).c_str());
    fprintf(sensorLog, "    \"type\": \"sensorWalking\",\n");
    fprintf(sensorLog, "    \"evaluationTimeout\": \"%d\",\n", evaluationTimeout);
    fprintf(sensorLog, "    \"evaluationDistance\": \"%.0f\",\n", evaluationDistance);
    fprintf(sensorLog, "    \"machine\": \"%s\",\n", hostname);
    fprintf(sensorLog, "    \"user\": \"%s\",\n", getenv("USER"));

    if (ros::Time::isSimTime()) fprintf(sensorLog, "    \"platform\": \"simulation\",\n");
    else
        fprintf(sensorLog, "    \"platform\": \"hardware\",\n");

    fprintf(sensorLog, "    \"fitness\": [\n");
    if (!instantFitness) {
        for (int i = 0; i < fitnessFunctions.size(); i++) {
            fprintf(sensorLog, "      \"%s\"", fitnessFunctions[i].c_str());
            if (i != fitnessFunctions.size() - 1) fprintf(sensorLog, ",\n");
        }
    } else fprintf(sensorLog, "      \"*INSTANT*\"");
    fprintf(sensorLog, "\n");
    fprintf(sensorLog, "    ]\n");
    fprintf(sensorLog, "  },\n");

    fprintf(sensorLog, "  \"individual\": {\n");
    printMap(individual, "    ", sensorLog);
    fprintf(sensorLog, "  },\n");

    fprintf(sensorLog, "  \"fitness\": [\n");
    fprintf(sensorLog, "    {\n");

    for (int j = 0; j < fitnesses.size(); j++) {
        int i = 0;
        for (auto elem : fitnesses[j]) {
            fprintf(sensorLog, "      \"%s\": %f", elem.first.c_str(), elem.second);
            if (i != fitnesses[j].size() - 1) fprintf(sensorLog, ",\n"); else fprintf(sensorLog, "\n");
            i++;
        }
        fprintf(sensorLog, "    }");

        if (j != fitnesses.size()-1) fprintf(sensorLog, ",\n    {");
        fprintf(sensorLog, "\n");
    }

    /*fprintf(sensorLog, "      \"raw_fitness\": [\n");
    fprintf(sensorLog, "        {\n");
    printMap(rawFitness[0], "          ", sensorLog);

    if (rawFitness.size() > 1) {
        fprintf(sensorLog, "        },{\n");
        printMap(rawFitness[1], "          ", sensorLog);
    }

    fprintf(sensorLog, "        }\n");*/
    fprintf(sensorLog, "  ]\n");
    fprintf(sensorLog, "}");

    fclose(sensorLog);

     numberOfEvalsInTesting = numberOfEvals_old;
}

void menu_experiments() {
    std::string choice;

    std::cout << "  Please choose one experiment: (enter to go back)\n";

    printf("      curr - run current experiment\n"
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
           "    Verify:\n"
           "      vf - verify fitness on single individual\n"
           "    Sensors:\n"
           "      rw - record sensors while walking\n"
           "      rs - record sensors standing still\n"
           "    Misc:\n"
           "      vn - check noise in stability fitness\n");
    printf("\n> ");

    if (commandQueue.empty()) {
        getline(std::cin, choice);
    } else {
        choice = commandQueue[0];
        printf("*%s*\n", choice.c_str());
        commandQueue.erase(commandQueue.begin());
    }

    if (!choice.empty()) {
        if (choice == "curr") {
            promptForConfirmation = true;
            enableFitnessLog(get_gait_evaluation_client);
            gaitDifficultyFactor = 0.20;

            printf("  Press enter when robot is ready and on the ground>");
            std::string input;
            getline(std::cin, input);

            zeroPrismaticActuators(true, poseCommand_pub, prismaticActuatorStates);

            experiments_evolve("nsga2", "", "lowLevelSplineGait");
        } else if (choice == "cs") {
            experiments_evolve("nsga2", "small", "highLevelSplineGait");
        } else if (choice == "cm") {
            experiments_evolve("nsga2", "medium", "highLevelSplineGait");
        } else if (choice == "cl") {
            experiments_evolve("nsga2", "large", "highLevelSplineGait");
        } else if (choice == "mn") {
            experiments_evolve("nsga2", "small", "highLevelSplineGait");
        } else if (choice == "el") {
            gaitDifficultyFactor = getDifficultyFactor(commandQueue);

            experiments_evolve("nsga2", "small", "lowLevelSplineGait");
        } else if (choice == "me") {
            experiments_evolve("map-elites", "small", "highLevelSplineGait");
        } else if (choice == "vf") {
            experiments_verifyFitness();
        } else if (choice == "vn") {
            experiments_fitnessNoise();
        } else if (choice == "rl") {
            gaitType = "lowLevelSplineGait";

            gaitDifficultyFactor = getDifficultyFactor(commandQueue);

            experiments_randomSearch();
        } else if (choice == "rh") {
            gaitType = "highLevelSplineGait";
            experiments_randomSearch();
        } else if (choice == "rs") {
            printf("  Please choose a label for your recording: ");
            std::string label;
            getline(std::cin, label);
            printf("  How many seconds should be recorded: ");
            int secondsToRecord;
            std::cin >> secondsToRecord;
            std::cin.clear();
            std::cin.ignore(10000, '\n');

            recordSensorData(label, secondsToRecord, loggerCommandService_client);
        } else if (choice == "rw") {

            experiments_sensorWalking();
        }
    }
}

void menu_configure() {
    std::string choice;

    std::cout << "  Please choose a setting to change: (enter to go back)\n";

    printf("    z - zero prismatic joints\n");
    printf("    p - enable/disable evaluation prompt\n");
    printf("    l - enable/disable fitness logging\n");
    printf("    i - enable/disable instant fitness\n");
    printf("    m - enable/disable morphology evolution\n");
    printf("    s - enable/disable stand testing\n");
    printf("    n - set number of evals\n");
    printf("    f - set frequency factor\n");
    printf("    t - set evaluation timeout\n");
    printf("    e - enable servo torques\n");
    printf("    d - disable servo torques\n");
    printf("    r - restPose, adjusted\n");
    printf("    t - restPose, directly\n");
    printf("    q - set random seed\n");
    printf("    x - reset simulation\n");
    printf("\n> ");

    if (commandQueue.empty()) {
        getline(std::cin, choice);
    } else {
        choice = commandQueue[0];
        printf("*%s*\n", choice.c_str());
        commandQueue.erase(commandQueue.begin());
    }

    if (choice.empty() != true) {
        if (choice == "i") {
            instantFitness = !instantFitness;
            if (instantFitness) printf("Instant fitness evaluation now enabled!\n");
            else printf("Instant fitness evaluation now disabled!\n");
        } else if (choice == "z") { // Zero prismatic joints
          zeroPrismaticActuators(true, poseCommand_pub, prismaticActuatorStates);
        } else if (choice == "y") {
          while(true) {
            playSound("beep_high", 3);
            sleep(1);
            playSound("beep_low", 3);
            sleep(3);
          }
        } else if (choice == "p") {

            promptForConfirmation = !promptForConfirmation;

            if (promptForConfirmation) printf("   evaluation prompt now enabled!\n");
            else printf("   evaluation prompt now disabled!\n");
        } else if (choice == "l") {

            currentlyLoggingFitness = !currentlyLoggingFitness;

            if (currentlyLoggingFitness) {
                enableFitnessLog(get_gait_evaluation_client);
                printf("   fitness log now enabled!\n");
            } else {
                disableFitnessLog(get_gait_evaluation_client);
                printf("   fitness log now disabled!\n");
            }

        } else if (choice == "s") {
            if (robotOnStand) {
                printf("   RobotOnStand now disabled!\n");
            } else {
                printf("   RobotOnStand now enabled!\n");
            }

            robotOnStand = !robotOnStand;
        } else if (choice == "e") {
            enableServos(servoConfigClient);
            printf("Torque enabled!\n");
        } else if (choice == "d") {
            disableServos(servoConfigClient);
            printf("Torque disabled!\n");
        } else if (choice == "f") {
            printf("  Frequency factor> ");
            std::cin >> frequencyFactor;
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            printf("  FrequencyFactor set to %f\n", frequencyFactor);
        } else if (choice == "n") {
            printf("  numberOfEvalsInTesting> ");
            std::cin >> numberOfEvalsInTesting;
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            printf("  numberOfEvalsInTesting set to %d\n", numberOfEvalsInTesting);
        } else if (choice == "t") {
            printf("  Evaluation timeout (in s)> ");
            std::cin >> evaluationTimeout;
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            printf("  Evaluation timeout set to %d\n", evaluationTimeout);
            printf("  Evaluation distance (in mm)> ");
            std::cin >> evaluationDistance;
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            printf("  Evaluation distance set to %.0f\n", evaluationDistance);
        } else if (choice == "r") {
            adjustRestPose(gaitCommandService_client);
        } else if (choice == "t"){
            if (ros::Time::isSystemTime()){
                setServoSpeeds(0.01, servoConfigClient);
            }
            sendAngleCommand(restPose, poseCommand_pub);
        } else if (choice == "x") {
            resetSimulation(gz);
        } else if (choice == "q") {
            randomSeed = 0;
            srand(randomSeed);
        }

    }
    printf("\n");
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

    gaitConfiguration_client = rch.serviceClient<dyret_controller::ConfigureGait>("/dyret/dyret_controller/gaitConfigurationService");
    gaitCommandService_client = rch.serviceClient<dyret_controller::GaitControllerCommandService>("/dyret/dyret_controller/gaitControllerCommandService");
    inferredPositionClient = rch.serviceClient<dyret_controller::GetInferredPosition>("/dyret/dyret_controller/getInferredPosition");
    loggerCommandService_client = rch.serviceClient<dyret_controller::LoggerCommand>("/dyret/dyret_logger/loggerCommand");

    servoConfigClient = rch.serviceClient<dyret_common::Configure>("/dyret/configuration");
    get_gait_evaluation_client = rch.serviceClient<dyret_controller::GetGaitEvaluation>("get_gait_evaluation");
    gaitControllerStatus_client = rch.serviceClient<dyret_controller::GetGaitControllerStatus>(
            "get_gait_controller_status");

    poseCommand_pub = rch.advertise<dyret_common::Pose>("/dyret/command", 10);
    dyretState_sub = rch.subscribe("/dyret/state", 1, dyretStateCallback);
    actuator_boardState_sub = rch.subscribe("/dyret/actuator_board/state", 1, actuator_boardStateCallback);

    waitForRosInit(get_gait_evaluation_client, "get_gait_evaluation");
    waitForRosInit(gaitControllerStatus_client, "gaitControllerStatus");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string evoInfo = "testInfo";

    if (!resetGaitRecording(get_gait_evaluation_client)) {
        spinner.stop();
        ros::shutdown();
        exit(-2);
    }

    currentIndividual = -1;

    if (ros::Time::isSimTime()) {
        printf("Currently running in simulation mode\n");

        gz = &gazebo::WorldConnection::instance();
    } else {
        printf("Currently running in hardware mode\n");
    }

    printf("Evo settings: %d generations of %d individuals\n", generations, popSize);

    printf("\n");

    sleep(1);

    if (ros::Time::isSystemTime()){
        setServoSpeeds(0.01, servoConfigClient);
    }

    sendAngleCommand(restPose, poseCommand_pub);

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
        printf("\n> ");

        if (commandQueue.empty() && fullCommand.empty()) {
            getline(std::cin, choice);
        } else if (commandQueue.empty() && !fullCommand.empty()) {
            choice = "exit";
        } else {
            choice = commandQueue[0];
            printf("*%s*\n", choice.c_str());
            commandQueue.erase(commandQueue.begin());
        }

        if (choice.empty() == true || choice == "exit") {
            fprintf(stderr, "Exiting\n");
            spinner.stop();
            ros::shutdown();
            exit(0);
        } else if (menu.find(choice) == menu.end()) {
            printf("Unknown choice!\n");
            continue;
        }

        menu[choice]();
    }

}
