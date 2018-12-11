#include "ros/ros.h"
#include "ros/console.h"
#include <rosbag/bag.h>

#include "dyret_common/State.h"
#include "tonnesfn_experiments/LoggerCommand.h"

rosbag::Bag bag;
bool loggingEnabled = false;

bool loggerCommandCallback(tonnesfn_experiments::LoggerCommand::Request  &req,
                           tonnesfn_experiments::LoggerCommand::Response &res) {

    std::string bagPath = req.logPath + "/" + req.individual + ".bag";

    if (req.command == req.INIT_LOG){
        ROS_INFO("Received INIT_LOG command with path \"%s\" and individual \"%s\"", req.logPath.c_str(), req.individual.c_str());
        bag.open(bagPath.c_str(), rosbag::bagmode::Write);
    } else if (req.command == req.ENABLE_LOGGING){
        ROS_INFO("Received START_LOGGING command");
        loggingEnabled = true;
    } else if (req.command == req.DISABLE_LOGGING){
        ROS_INFO("Received DISABLE_LOGGING command");
        loggingEnabled = true;
    } else if (req.command == req.SAVE_LOG) {
        ROS_INFO("Received SAVE_LOG command");
        loggingEnabled = false;
        bag.close();
    } else {
        ROS_ERROR("Received unknown command %d", req.command);
    }

    return true;

}

void dyretStateCallback(const dyret_common::State::ConstPtr &msg) {
    if (loggingEnabled){
        bag.write("/dyret/state", msg->header.stamp, msg);
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dyret_logger");
  ros::NodeHandle n;

  ros::ServiceServer loggerCommandServer = n.advertiseService("/dyret/dyret_logger/loggerCommand", loggerCommandCallback);

  ros::Subscriber dyretState_sub = n.subscribe("/dyret/state", 1, dyretStateCallback);

  ROS_INFO("dyret_logger running");

  ros::spin();

}
