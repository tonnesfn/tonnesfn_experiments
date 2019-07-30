#include "expFunctions.h"

// Needed to access `WorldControl` message
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
// Needed to access `spinOnce`
#include <ros/ros.h>
// Needed for reset functionality
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

namespace gazebo {
    WorldConnection::WorldConnection() {
        // Initialize Gazebo on creation, this is safe since this class
        // can only be used through singleton!
        if(!gazebo::client::setup(0, nullptr)){
            ROS_FATAL("Gazebo client setup failed!");
        }
        // Initialize connection to gazebo through node
        node.reset(new gazebo::transport::Node());
        node->Init();
        // Create publisher
        pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    }

    WorldConnection::~WorldConnection() {
        // Shutdown Gazebo connection
        if(!gazebo::client::shutdown()){
            ROS_FATAL("Gazebo client shutdown failed!");
        }
    }

    void WorldConnection::step(const size_t steps) {
        gazebo::msgs::WorldControl msg;

        msg.set_multi_step(steps);

        pub->Publish(msg);
        ros::spinOnce();
        //usleep(100000);
    }

    bool WorldConnection::reset() {
        static std_srvs::Empty empty;
        if(!ros::service::call("/gazebo/reset_world", empty)){
            // Could not reset world!
            return false;
        }
        // Create service request for DyRET
        std_srvs::SetBool b;
        b.request.data = true;
        if(!ros::service::call("/dyret/simulation/reset", b)){
            // Could not reset DyRET!
            return false;
        }
        // All is well that ends well
        return true;
    }
}

void pauseGazebo(){
    usleep(1000);
    std_srvs::Empty empty;
    if (ros::service::call("/gazebo/pause_physics", empty) == false){
      fprintf(stderr, "Error when calling pause_physics\n");
    }
    ROS_INFO("Paused physics");
    usleep(1000);
}

void unpauseGazebo(){
    std_srvs::Empty empty;
    if(ros::service::call("/gazebo/unpause_physics", empty) == false){
      fprintf(stderr, "Error when calling unpause_physics\n");
    }
    ROS_INFO("Unpaused physics");
}

std::string trim(std::string& str){
    size_t first = str.find_first_not_of(' ');
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last-first+1));
}

double getMapValue(const std::map<std::string, double> &givenMap, const std::string &givenValue){
    try {
        return givenMap.at(givenValue);
    }
    catch (const std::out_of_range& e) {
        ROS_FATAL("Value %s not found in map", givenValue.c_str());
        ros::shutdown();
        exit(-1);
    }
}

void printMap(std::map<std::string, double> givenMap, std::string givenLeadingString, FILE* givenFileDescriptor){

    int i = 0;
    for(auto elem : givenMap){
        fprintf(givenFileDescriptor, (givenLeadingString + "\"%s\": %f").c_str(), elem.first.c_str(), elem.second);
        if (i != givenMap.size()-1) fprintf(givenFileDescriptor, ",\n"); else fprintf(givenFileDescriptor,"\n");
        i++;
    }
}

bool callServoConfigService(dyret_common::Configure givenCall, ros::ServiceClient givenServoConfigService){
    if (givenServoConfigService.call(givenCall))  {
        switch(givenCall.response.status){
            case dyret_common::Configure::Response::STATUS_NOERROR:
                ROS_DEBUG("Configure servo service returned no error");
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

bool setServoSpeeds(float givenSpeed, ros::ServiceClient givenServoConfigClient){
    dyret_common::Configure srv;

    srv.request.configuration.revolute.type = dyret_common::RevoluteConfig::TYPE_SET_SPEED;
    srv.request.configuration.revolute.ids.resize(12);
    srv.request.configuration.revolute.parameters.resize(12);

    for (int i = 0; i < 12; i++){
        srv.request.configuration.revolute.ids[i] = i;
        srv.request.configuration.revolute.parameters[i] = givenSpeed;
    }

    return callServoConfigService(srv, givenServoConfigClient);

}

bool sendServoTorqueMessage(bool enable, ros::ServiceClient givenServoConfigClient){
    dyret_common::Configure srv;

    if (enable == true){
        srv.request.configuration.revolute.type = dyret_common::RevoluteConfig::TYPE_ENABLE_TORQUE;
    } else {
        srv.request.configuration.revolute.type = dyret_common::RevoluteConfig::TYPE_DISABLE_TORQUE;
    }

    srv.request.configuration.revolute.ids = {0,1,2,3,4,5,6,7,8,9,10,11,12};

    return callServoConfigService(srv, givenServoConfigClient);
}

bool restartServos(ros::ServiceClient givenServoConfigClient){
  dyret_common::Configure srv;

  srv.request.configuration.revolute.type = dyret_common::RevoluteConfig::TYPE_RESTART;
  srv.request.configuration.revolute.ids = {0,1,2,3,4,5,6,7,8,9,10,11,12};

  return callServoConfigService(srv, givenServoConfigClient);
}

bool enableServos(ros::ServiceClient givenServoConfigClient){
    return sendServoTorqueMessage(true, givenServoConfigClient);
}

bool disableServos(ros::ServiceClient givenServoConfigClient){
    return sendServoTorqueMessage(false, givenServoConfigClient);
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
        printf("Error while calling GaitRecording service with t_resetStatistics\n");
        ROS_ERROR("Error while calling GaitRecording service with t_resetStatistics");

        return false;
    }

    return true;

}

bool pauseGaitRecording(ros::ServiceClient get_gait_evaluation_client){
    dyret_controller::GetGaitEvaluation srv;
    srv.request.givenCommand = dyret_controller::GetGaitEvaluation::Request::t_pause;
    if (!get_gait_evaluation_client.call(srv)){
        printf("Error while calling GaitRecording service with t_pause\n");
        ROS_ERROR("Error while calling GaitRecording service with t_pause");

        return false;
    }

    return true;

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

void startVideo(std::string fileName) {

  ROS_INFO("Starting video and saving to %s", fileName.c_str());

  camera_recorder::Record srv;
  srv.request.output = fileName;
  srv.request.overwrite = true;

  // First pause physics:
  ros::service::call("/camera_recorder/start", srv);

  if (!srv.response.success){
    ROS_ERROR("Start recording not successful: %s", srv.response.message.c_str());
  }
}

void stopVideo() {
  std_srvs::Trigger srv;

  ros::service::call("/camera_recorder/stop", srv);

  if (!srv.response.success){
    ROS_ERROR("Stop recording not successful: %s", srv.response.message.c_str());
  }

}

void playSound(std::string soundName, int number){

  std::string path = ros::package::getPath("tonnesfn_experiments");
  std::string command = "canberra-gtk-play -f " + path + "/sound/" + soundName + ".wav";

  for (int i = 0; i < number; i++){
    system(command.c_str());
  }
}

void startWalking(ros::ServiceClient gaitCommandService_client){
    dyret_controller::GaitControllerCommandService srv;

    srv.request.gaitControllerCommand.gaitControllerCommand = srv.request.gaitControllerCommand.t_startWalking;

    if (gaitCommandService_client.call(srv) == false) {
        ROS_ERROR("Error while calling gaitControllerCommand service for startWalking");
    }
}

void stopWalking(ros::ServiceClient gaitCommandService_client){
    dyret_controller::GaitControllerCommandService srv;

    srv.request.gaitControllerCommand.gaitControllerCommand = srv.request.gaitControllerCommand.t_stopWalking;

    if (gaitCommandService_client.call(srv) == false) {
        ROS_ERROR("Error while calling gaitControllerCommand service for stopWalking");
    }
}

bool initLog(std::string individual, std::string logDirectoryPath, ros::ServiceClient loggerCommandService_client){
    dyret_controller::LoggerCommand srv;

    std::string logPath = logDirectoryPath.substr(0, logDirectoryPath.find_last_of("\\/")) + "/bags/";

    mkdir(logPath.c_str(), 0700);

    srv.request.command = srv.request.INIT_LOG;
    srv.request.logPath = logPath;
    srv.request.individual = individual;

    if (!loggerCommandService_client.call(srv)) {
        printf("Error while calling LoggerCommand service\n");
        ROS_ERROR("Error while calling LoggerCommand service");
        return false;
    }

    ROS_INFO("Log initialized");

    return true;
}

bool startLogging(ros::ServiceClient loggerCommandService_client){
    dyret_controller::LoggerCommand srv;

    srv.request.command = srv.request.ENABLE_LOGGING;

    if (!loggerCommandService_client.call(srv)) {
        printf("Error while calling LoggerCommand service\n");
        ROS_ERROR("Error while calling LoggerCommand service");
        return false;
    }

    return true;
}

bool saveLog(ros::ServiceClient loggerCommandService_client){
    dyret_controller::LoggerCommand srv;

    srv.request.command = srv.request.SAVE_LOG;

    if (!loggerCommandService_client.call(srv)) {
        printf("Error while calling LoggerCommand service\n");
        ROS_ERROR("Error while calling LoggerCommand service");
        return false;
    }

    return true;
}

float getInferredPosition(ros::ServiceClient inferredPositionClient){
    dyret_controller::GetInferredPosition srv;

    if (!inferredPositionClient.call(srv)){
        printf("Error while calling GetInferredPosition service\n");
        ROS_ERROR("Error while calling GetInferredPosition service");

        return 0.0;
    }

    return srv.response.currentInferredPosition.distance;
}

std::string getInputFromTerminal(std::string output){

    printf("%s> ", output.c_str());

    std::cin.sync();
    std::cin.clear();

    std::string input;
    getline(std::cin, input);

    return input;

}

bool legsAtRest(const std::array<int, 8> prismaticActuatorStates){
    for (int i = 0; i < 8; i++){
        if (prismaticActuatorStates[i] != 0){
            return false;
        }
    }

    return true;
}

void setLegLengths(std::vector<float> lengths, ros::Publisher poseCommand_pub) {
    dyret_common::Pose msg;

    msg.header.stamp = ros::Time().now();

    msg.prismatic.resize(lengths.size());

    for (int i = 0; i < lengths.size(); i++){
        msg.prismatic[i] = lengths[i];
    }

    poseCommand_pub.publish(msg);
}

void setLegLengths(float femurLengths, float tibiaLengths, ros::Publisher poseCommand_pub) {
    dyret_common::Pose msg;

    msg.header.stamp = ros::Time().now();

    msg.prismatic.resize(2);

    msg.prismatic[0] = femurLengths;
    msg.prismatic[1] = tibiaLengths;

    poseCommand_pub.publish(msg);
}

void setLegLengths(float lengths, ros::Publisher poseCommand_pub) {
    dyret_common::Pose msg;

    msg.header.stamp = ros::Time().now();

    msg.prismatic.resize(1);

    msg.prismatic[0] = lengths;

    poseCommand_pub.publish(msg);
}

void resetSimulation(gazebo::WorldConnection* gz){
    if (!gz->reset()) ROS_ERROR("Could not reset simulation");
}

void zeroPrismaticActuators(bool runOutFirst, ros::Publisher poseCommand_pub, std::array<int, 8> prismaticActuatorStates){

    if (runOutFirst) {
        // Set all lengths to 5mm
        setLegLengths(5.0, 5.0, poseCommand_pub);

        // Wait for actuators to go to rest
        sleep(1);
        while (!legsAtRest(prismaticActuatorStates) && ros::ok()) {}
        sleep(1);
    }

    // Set all lengths to -100mm
    setLegLengths(-100.0, -100.0, poseCommand_pub);

    // Wait for actuators to go to rest
    sleep(1);
    while(!legsAtRest(prismaticActuatorStates)){}
    sleep(1);
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

bool enableFitnessLog(ros::ServiceClient get_gait_evaluation_client){

    dyret_controller::GetGaitEvaluation srv;
    srv.request.givenCommand = dyret_controller::GetGaitEvaluation::Request::t_enableLogging;

    return get_gait_evaluation_client.call(srv);
}

bool disableFitnessLog(ros::ServiceClient get_gait_evaluation_client){

    dyret_controller::GetGaitEvaluation srv;
    srv.request.givenCommand = dyret_controller::GetGaitEvaluation::Request::t_disableLogging;

    return get_gait_evaluation_client.call(srv);
}

void setGaitParams(std::string gaitType,
                   std::string logFilePath,
                   bool directionForward,
                   bool prepareForGait,
                   bool liveUpdate,
                   std::vector<float> femurLengths,
                   std::vector<float> tibiaLengths,
                   std::vector<std::string> parameterNames,
                   std::vector<float> parameterValues,
                   ros::ServiceClient gaitConfiguration_client){

    dyret_controller::ConfigureGait srv;

    srv.request.gaitConfiguration.gaitType           = gaitType;
    srv.request.gaitConfiguration.logFilePath        = logFilePath;
    srv.request.gaitConfiguration.directionForward   = directionForward;
    srv.request.gaitConfiguration.prepareForGait     = prepareForGait;
    srv.request.gaitConfiguration.gaitParameterName  = parameterNames;
    srv.request.gaitConfiguration.gaitParameterValue = parameterValues;

    srv.request.gaitConfiguration.liveUpdate = liveUpdate;

    srv.request.gaitConfiguration.femurLengths = femurLengths;
    srv.request.gaitConfiguration.tibiaLengths = tibiaLengths;

    gaitConfiguration_client.call(srv);
}

void setGaitParams(std::string gaitType,
                   std::string logFilePath,
                   bool directionForward,
                   bool prepareForGait,
                   bool liveUpdate,
                   std::vector<float> femurLengths,
                   std::vector<float> tibiaLengths,
                   std::map<std::string, double> phenoTypeMap,
                   ros::ServiceClient gaitConfiguration_client){
    std::vector<std::string> parameterNames;
    std::vector<float> parametervalues;

    for(auto elem : phenoTypeMap){
        parameterNames.push_back(elem.first);
        parametervalues.push_back((float) elem.second);
    }

    setGaitParams(gaitType, logFilePath, directionForward, prepareForGait, liveUpdate, femurLengths, tibiaLengths, parameterNames, parametervalues, gaitConfiguration_client);
}

void sendAngleCommand(std::vector<float> angles, ros::Publisher poseCommand_pub){
    dyret_common::Pose msg;

    msg.header.stamp = ros::Time().now();
    msg.revolute = angles;

    poseCommand_pub.publish(msg);
}

float getMaxServoTemperature(std::array<float, 12> servoTemperatures, bool printAllTemperatures) {
    float maxTemp = -1.0f;

    if (printAllTemperatures) printf("Servo temperatures: ");
    for (int i = 0; i < servoTemperatures.size(); i++){
        if (servoTemperatures[i] > maxTemp) maxTemp = servoTemperatures[i];
        if (printAllTemperatures) printf("%.2f ", servoTemperatures[i]);
    }
    if (printAllTemperatures) printf("\n");

    return maxTemp;
}

void adjustGaitPose(ros::ServiceClient gaitCommandService_client){
    dyret_controller::GaitControllerCommandService srv;

    srv.request.gaitControllerCommand.gaitControllerCommand = srv.request.gaitControllerCommand.t_adjustGaitPose;

    if (gaitCommandService_client.call(srv) == false) {
        ROS_ERROR("Error while calling gaitControllerCommand service for adjustGaitPose");
    }
}

void adjustRestPose(ros::ServiceClient gaitCommandService_client){
    dyret_controller::GaitControllerCommandService srv;

    srv.request.gaitControllerCommand.gaitControllerCommand = srv.request.gaitControllerCommand.t_adjustRestPose;

    if (gaitCommandService_client.call(srv) == false) {
        ROS_ERROR("Error while calling gaitControllerCommand service for adjustRestPose");
    }
}

void runGaitControllerWithActionMessage(bool forward,
                                        int currentIndividual,
                                        ros::ServiceClient get_gait_evaluation_client,
                                        ros::ServiceClient loggerCommandService_client,
                                        ros::ServiceClient gaitCommandService_client,
                                        ros::ServiceClient inferredPositionClient,
                                        bool enableLogging,
                                        int evaluationTimeout,
                                        float evaluationDistance,
                                        std::string logDirectoryPath){

    sleep(1);

    resetGaitRecording(get_gait_evaluation_client);

    std::string direction;
    if (forward) direction = "F"; else direction = "R";

    // Start walking
    startWalking(gaitCommandService_client);

    // Start bag logging
    if (!ros::Time::isSimTime() && enableLogging) {
        initLog(std::to_string(currentIndividual) + direction, logDirectoryPath, loggerCommandService_client);
        startLogging(loggerCommandService_client);

        std::string logPath = logDirectoryPath.substr(0, logDirectoryPath.find_last_of("\\/")) + "/video/";
        mkdir(logPath.c_str(), 0700);

        startVideo(logPath + std::to_string(currentIndividual) + direction + ".mp4");

    }

    // Start fitness recording
    startGaitRecording(get_gait_evaluation_client);

    // Wait until the robot is done walking
    ros::Time startTime = ros::Time::now();
    while (ros::ok()) {
        usleep(100);

        float currentPos = getInferredPosition(inferredPositionClient);

        if ((ros::Time::now() - startTime).sec > (evaluationTimeout)) {
            printf("  Timed out at %ds/%ds (dist %.0fmm/%.0fmm)\n", (ros::Time::now() - startTime).sec, evaluationTimeout, currentPos, evaluationDistance);
            break;
        }

        if (currentPos > evaluationDistance){
            printf("  Reached position with %.2fmm / %.2fmm (time %ds/%ds)\n", currentPos, evaluationDistance, (ros::Time::now() - startTime).sec, evaluationTimeout);
            break;
        }
    }

    // Pause fitness recording
    pauseGaitRecording(get_gait_evaluation_client);

    // Save and stop bag logging
    if (!ros::Time::isSimTime() && enableLogging) {
        saveLog(loggerCommandService_client);
        stopVideo();
    }

    // Stop walking
    stopWalking(gaitCommandService_client);

    playSound("beep_high");

}

void spinGaitControllerOnce(ros::ServiceClient gaitCommandService_client){

    dyret_controller::GaitControllerCommandService srv;

    srv.request.gaitControllerCommand.gaitControllerCommand = srv.request.gaitControllerCommand.t_spinOnce;

    if (gaitCommandService_client.call(srv) == false) {
        ROS_ERROR("Error while calling gaitControllerCommand service");
    }

}

void runGaitWithServiceCalls(float evaluationDistance,
        int evaluationTimeout,
        gazebo::WorldConnection* gz,
        ros::ServiceClient get_gait_evaluation_client,
        ros::ServiceClient inferredPositionClient,
        ros::ServiceClient gaitCommandService_client){

    resetGaitRecording(get_gait_evaluation_client);
    startGaitRecording(get_gait_evaluation_client);

    if (!ros::Time::isSimTime()) {
        ROS_ERROR("In runGaitWithServiceCalls in hardware experiments!");
    }


    ros::Time startTime_sim = ros::Time::now();
    ros::WallTime startTime_rw = ros::WallTime::now();
    while (ros::ok() && (getInferredPosition(inferredPositionClient) < evaluationDistance)) {
        gz->step(30);
        spinGaitControllerOnce(gaitCommandService_client);

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

std::map<std::string, double> genToHighLevelSplineGaitPhen(std::vector<double> givenGenotype, float frequencyFactor) {

    assert(givenGenotype.size() >= 10);

    std::map<std::string, double> phenoType;

    phenoType["femurLength"]     = givenGenotype[0] * 50.0;          //  0    ->  50
    phenoType["tibiaLength"]     = givenGenotype[1] * 95.0;          //  0    ->  95
    phenoType["stepLength"]      = givenGenotype[2] * 300.0;         //  0    -> 300
    phenoType["stepHeight"]      = 25.0 + (givenGenotype[3] * 50.0); // 25    ->  75
    phenoType["smoothing"]       = givenGenotype[4] * 50.0;          //  0    ->  50
    phenoType["frequency"]       = (0.25 + givenGenotype[5] * 0.75) * frequencyFactor;   //  0.25 ->   1.0
    phenoType["wagPhase"]        = (givenGenotype[6] * 0.4) - 0.2;   // -0.2  ->   0.2
    phenoType["wagAmplitude_x"]  = givenGenotype[7] * 50.0;          //  0    ->  50
    phenoType["wagAmplitude_y"]  = givenGenotype[8] * 50.0;          //  0    ->  50
    phenoType["liftDuration"]    = (givenGenotype[9] * 0.15) + 0.05; //  0.05 ->   0.20

    if (frequencyFactor != 1.0){
        ROS_WARN("Using frequencyFactor %.2f", frequencyFactor);
    }

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

std::map<std::string, double> genToLowLevelSplineGaitPhen(std::vector<double> givenGenotype, float frequencyFactor, float gaitDifficultyFactor) {

    if (givenGenotype.size() < 20){
        ROS_ERROR("givenGenotype.size() < 20: %lu", givenGenotype.size());
        exit(-1);
    }

    std::map<std::string, double> phenoType;

    phenoType["difficultyFactor"] = gaitDifficultyFactor;

    phenoType["femurLength"]     = givenGenotype[0] * 50.0;          // 0    -> 50
    phenoType["tibiaLength"]     = givenGenotype[1] * 95.0;          // 0    -> 95
    phenoType["liftDuration"]    = getPoint(givenGenotype[2], 0.05, 0.20, 0.175, 0.05, gaitDifficultyFactor); // 0.15, 0.2 -> 0.05, 0.2
    phenoType["frequency"]       = (0.25 + (givenGenotype[3] * 0.75)) * frequencyFactor; // 0.25 ->  1.0

    if (frequencyFactor != 1.0){
        ROS_WARN("Using frequencyFactor %.2f", frequencyFactor);
    }

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

std::map<std::string, double> genToLowLevelAdvancedSplineGaitPhen(std::vector<double> givenGenotype, float frequencyFactor, float gaitDifficultyFactor) {

    if (givenGenotype.size() < 20){
        ROS_ERROR("givenGenotype.size() < 20: %lu", givenGenotype.size());
        exit(-1);
    }

    std::map<std::string, double> phenoType;

    phenoType["difficultyFactor"] = gaitDifficultyFactor;

    phenoType["femurLength_front"]    = givenGenotype[0] * 50.0;          // 0    -> 50
    phenoType["femurLength_rear"]     = givenGenotype[1] * 50.0;          // 0    -> 50
    phenoType["tibiaLength_front"]    = givenGenotype[2] * 95.0;          // 0    -> 95
    phenoType["tibiaLength_rear"]     = givenGenotype[3] * 95.0;          // 0    -> 95
    phenoType["liftDuration"]    = getPoint(givenGenotype[4], 0.05, 0.20, 0.175, 0.05, gaitDifficultyFactor); // 0.15, 0.2 -> 0.05, 0.2
    phenoType["frequency"]       = (0.25 + (givenGenotype[5] * 0.75)) * frequencyFactor; // 0.25 ->  1.0

    if (frequencyFactor != 1.0){
        ROS_WARN("Using frequencyFactor %.2f", frequencyFactor);
    }

    phenoType["wagPhase"]        = getPoint(givenGenotype[6], -M_PI/2.0, M_PI/2.0, 0.0, 0.2, gaitDifficultyFactor);
    phenoType["wagAmplitude_x"]  = getPoint(givenGenotype[7],         0,     50.0, 0.0, 5.0, gaitDifficultyFactor);
    phenoType["wagAmplitude_y"]  = getPoint(givenGenotype[8],         0,     50.0, 0.0, 5.0, gaitDifficultyFactor);

    // StepLength 25 -> 300, p0 center around 75, p1 center around -75
    phenoType["p0_y"] = getPoint(givenGenotype[10], -150.0, 150.0,   50.0, 50.0, gaitDifficultyFactor);
    phenoType["p1_y"] = getPoint(givenGenotype[12], -150.0, 150.0, -100.0, 50.0, gaitDifficultyFactor);

    // Make sure the front point is actually in front:
    float gnd_min = fmin(phenoType["p0_y"], phenoType["p1_y"]);
    float gnd_max = fmax(phenoType["p0_y"], phenoType["p1_y"]);

    phenoType["p0_y"] = gnd_max;
    phenoType["p1_y"] = gnd_min;

    // (potential) Front air point:
    phenoType["p2_x_front"] = getPoint(givenGenotype[13],  -25.0,  25.0,  0.0,  0.0, gaitDifficultyFactor); // -25, 25 -> 0, 0
    phenoType["p2_y_front"] = getPoint(givenGenotype[14], -150.0, 150.0, 75.0, 50.0, gaitDifficultyFactor); // -150, 150 -> 50, 100
    phenoType["p2_z_front"] = getPoint(givenGenotype[15],   10.0,  80.0, 30.0, 10.0, gaitDifficultyFactor); // 10, 80 -> 25, 35
    phenoType["p2_x_rear"]  = getPoint(givenGenotype[16],  -25.0,  25.0,  0.0,  0.0, gaitDifficultyFactor); // -25, 25 -> 0, 0
    phenoType["p2_y_rear"]  = getPoint(givenGenotype[17], -150.0, 150.0, 75.0, 50.0, gaitDifficultyFactor); // -150, 150 -> 50, 100
    phenoType["p2_z_rear"]  = getPoint(givenGenotype[18],   10.0,  80.0, 30.0, 10.0, gaitDifficultyFactor); // 10, 80 -> 25, 35

    // (potential) Top air point:
    phenoType["p3_x_front"] = getPoint(givenGenotype[19],  -25.0,  25.0,  0.0,  0.0, gaitDifficultyFactor); // -25, 25 -> 0, 0
    phenoType["p3_y_front"] = getPoint(givenGenotype[20], -150.0, 150.0,  0.0,  0.0, gaitDifficultyFactor); // -150, 150 -> 0, 0
    phenoType["p3_z_front"] = getPoint(givenGenotype[21],   10.0,  80.0, 50.0, 10.0, gaitDifficultyFactor); // 10, 80 -> 45, 55
    phenoType["p3_x_rear"]  = getPoint(givenGenotype[22],  -25.0,  25.0,  0.0,  0.0, gaitDifficultyFactor); // -25, 25 -> 0, 0
    phenoType["p3_y_rear"]  = getPoint(givenGenotype[23], -150.0, 150.0,  0.0,  0.0, gaitDifficultyFactor); // -150, 150 -> 0, 0
    phenoType["p3_z_rear"]  = getPoint(givenGenotype[24],   10.0,  80.0, 50.0, 10.0, gaitDifficultyFactor); // 10, 80 -> 45, 55

    // (potential) Back air point:
    phenoType["p4_x_front"] = getPoint(givenGenotype[25],  -25.0,  25.0,   0.0,  0.0, gaitDifficultyFactor); // -25, 25 -> 0, 0
    phenoType["p4_y_front"] = getPoint(givenGenotype[26], -150.0, 150.0, -75.0, 50.0, gaitDifficultyFactor); // -150, 150 -> -50, -100
    phenoType["p4_z_front"] = getPoint(givenGenotype[27],   10.0,  80.0,  30.0, 10.0, gaitDifficultyFactor); // 10, 80 -> 25, 35
    phenoType["p4_x_rear"]  = getPoint(givenGenotype[28],  -25.0,  25.0,   0.0,  0.0, gaitDifficultyFactor); // -25, 25 -> 0, 0
    phenoType["p4_y_rear"]  = getPoint(givenGenotype[29], -150.0, 150.0, -75.0, 50.0, gaitDifficultyFactor); // -150, 150 -> -50, -100
    phenoType["p4_z_rear"]  = getPoint(givenGenotype[30],   10.0,  80.0,  30.0, 10.0, gaitDifficultyFactor); // 10, 80 -> 25, 35

    return phenoType;
}

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

std::string getCommitHash(std::string packageName) {

    std::string path = ros::package::getPath(packageName.c_str());
    std::string command = ("git -C " + path + " rev-parse HEAD").c_str();

    std::array<char, 128> buffer;
    std::string result = packageName + ": ";
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

void writeVersionLog(std::string givenLogDirectory){


    fprintf(stderr, "Writing version info to %s\n", givenLogDirectory.c_str());

    FILE *versionLog = fopen((givenLogDirectory + "versions.txt").c_str(), "w");

    std::string packages[] = {"tonnesfn_experiments", "dyret_common", "dyret_controller", "dyret_hardware", "camera_recorder", "xsens_driver", "mocap_optitrack", "rosserial_arduino"};

    for (const std::string &package : packages){
        fprintf(versionLog, "%s", getCommitHash(package).c_str());
    }

    fclose(versionLog);

}

bool directoryExists(std::string givenPath){
    struct stat info;

    if(stat(givenPath.c_str(), &info ) != 0 ) {
        return false;
    } else if( info.st_mode & S_IFDIR ) {
        return true;
    }
    return false;
}

std::vector<double> getRandomIndividual() {
    std::vector<double> individual(20);

    for (int j = 0; j < individual.size(); j++) individual[j] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

    return individual;
}

float getDifficultyFactor(std::vector<std::string> commandQueue){
    printf("Difficulty?> ");
    float gaitDifficultyFactor;

    std::string difficulty;

    if (commandQueue.empty()) {
        getline(std::cin, difficulty);
    } else {
        difficulty = commandQueue[0];
        printf("*%s*\n", difficulty.c_str());
        commandQueue.erase(commandQueue.begin());
    }

    if(difficulty.find_first_not_of("1234567890.-") != std::string::npos){

        // Get random number and limit to 0.xxx
        gaitDifficultyFactor = ((int)(1000.0 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX))) / 1000.0;

        printf("Did not receive valid complexity number. Setting to random: %3f\n", gaitDifficultyFactor);

    } else {
        gaitDifficultyFactor = atof(difficulty.c_str());
    }

    return gaitDifficultyFactor;
}

void cooldownServos(ros::ServiceClient servoConfigClient, std::array<float, 12> servoTemperatures, ros::Publisher poseCommand_pub, std::vector<float> restPose){
    getMaxServoTemperature(servoTemperatures, true);
    printf("Cooldown? TURNS OFF SERVOS! (y/n) > ");

    std::string input;
    getline(std::cin, input);

    if (input == "y") {
        disableServos(servoConfigClient);
        long long int currentTime = getMs();
        printf("00.0 ");
        while (getMaxServoTemperature(servoTemperatures, true) > 50) {
            sleep(10);
            printf("%3.1f: ", ((getMs() - currentTime) / 1000.0) / 60.0);
        }

        std::string input;
        std::cout << "Press enter to enable servos";
        getline(std::cin, input);

        usleep(1000);
        setServoSpeeds(0.01, servoConfigClient);
        enableServos(servoConfigClient);
        usleep(1000);
        sendAngleCommand(restPose, poseCommand_pub);

        std::cout << "Press enter to continue evolution";
        getline(std::cin, input);
    }
}

std::string makeSensorDataDirectories(std::string givenSurface, int givenMorphology){
    std::stringstream ss;
    ss << getenv("HOME") << "/catkin_ws/experimentResults/";
    mkdir(ss.str().c_str(), 0700);

    ss.str(std::string());
    ss << getenv("HOME") << "/catkin_ws/experimentResults/sensorWalking/";
    mkdir(ss.str().c_str(), 0700);

    ss.str(std::string());
    ss << getenv("HOME") << "/catkin_ws/experimentResults/sensorWalking/" << givenSurface << "/";
    mkdir(ss.str().c_str(), 0700);

    time_t t = time(0);   // get time now
    struct tm *now = localtime(&t);

    ss.str(std::string());
    ss << getenv("HOME") << "/catkin_ws/experimentResults/sensorWalking/" << givenSurface << "/" << std::to_string(givenMorphology) << "_" << getDateString(now) << "/";
    mkdir(ss.str().c_str(), 0700);

    ss << getDateString(now) << ".json";

    return ss.str();

}

void recordSensorData(std::string label, int secondsToRecord, ros::ServiceClient loggerCommandService_client){
    time_t t = time(0);   // get time now
    struct tm *now = localtime(&t);

    std::stringstream ss;
    ss << getenv("HOME") << "/catkin_ws/experimentResults/";
    mkdir(ss.str().c_str(), 0700);

    ss.str(std::string());
    ss << getenv("HOME") << "/catkin_ws/experimentResults/sensors/";
    mkdir(ss.str().c_str(), 0700);

    ss.str(std::string());
    ss << getenv("HOME") << "/catkin_ws/experimentResults/sensors/" << label << "/";
    mkdir(ss.str().c_str(), 0700);

    initLog(getDateString(now), ss.str(), loggerCommandService_client);
    startLogging(loggerCommandService_client);
    fprintf(stderr, "starting sleep\n");
    sleep(secondsToRecord);
    fprintf(stderr, "ending sleep\n");
    saveLog(loggerCommandService_client);
    fprintf(stderr, "Done\n");
}