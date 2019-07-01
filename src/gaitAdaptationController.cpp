
#include "ros/ros.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "gaitAdaptationController");
    ros::NodeHandle rch;

    ros::spin();

}