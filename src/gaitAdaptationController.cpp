
#include "ros/ros.h"

#include "expFunctions.h"

#include "individuals.h"

ros::ServiceClient gaitConfiguration_client;

void updateGaitConfiguration(std::string gaitType,
                             std::map<std::string, double> givenPhenoType,
                             ros::ServiceClient givenGaitConfiguration_client){
    setGaitParams(gaitType,
                  "",
                  true,
                  false,
                  true,
                  std::vector<float>(),
                  std::vector<float>(),
                  givenPhenoType,
                  givenGaitConfiguration_client);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "gaitAdaptationController");
    ros::NodeHandle rch;

    gaitConfiguration_client = rch.serviceClient<dyret_controller::ConfigureGait>("/dyret/dyret_controller/gaitConfigurationService");


    while(1 != 2) {

        printf("\nMenu:\n");
        printf("  rz - Send lowLevelSplineGait::zeroHeight command\n");
        printf("  rzf - Send faster lowLevelSplineGait::zeroHeight command\n");
        printf("> ");

        std::string choice;

        getline(std::cin, choice);


        if (choice == "rz") {
            updateGaitConfiguration("lowLevelSplineGait", individuals_lowLevelSplineGait::zeroHeight, gaitConfiguration_client);

            printf("  Sent gait configuration message with lowLevelSplineGait::zeroHeight individual\n");

        } else if (choice == "rzf") {
            std::map<std::string, double> tmpIndividual = individuals_lowLevelSplineGait::zeroHeight;
            tmpIndividual["frequency"] = 0.5;
            updateGaitConfiguration("lowLevelSplineGait", tmpIndividual, gaitConfiguration_client);

            printf("  Sent gait configuration message with faster lowLevelSplineGait::zeroHeight individual\n");
        } else if (choice == "") {
            break;
        } else {
            printf("Unknown command!\n\n");
        }
    }
}