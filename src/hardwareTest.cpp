#include "ros/ros.h"

#include "dyret_common/GetGaitControllerStatus.h"
#include "dyret_common/ActionMessage.h"
#include "dyret_common/Trajectory.h"

#include "dyret_utils/wait_for_ros.h"

ros::Publisher trajectoryMessage_pub;

ros::ServiceClient gaitControllerStatus_client;

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

int main(int argc, char **argv){

  ros::init(argc, argv, "hardwareTest");
  ros::NodeHandle n;
  sleep(3); // Allow the other nodes to start first

  ROS_INFO("HardwareTest initialized");

  gaitControllerStatus_client = n.serviceClient<dyret_common::GetGaitControllerStatus>("get_gait_controller_status");
  trajectoryMessage_pub = n.advertise<dyret_common::Trajectory>("trajectoryMessages", 1000);

  waitForRosInit(gaitControllerStatus_client, "gaitControllerStatus");
  waitForRosInit(trajectoryMessage_pub, "trajectoryMessage");

  resetTrajectoryPos(trajectoryMessage_pub);

  if (ros::ok()) sleep(5);

  while(ros::ok()){

      if (!ros::ok()) break;
      ROS_INFO("Starting trajectory");

      // Initialize
      std::vector<float> trajectoryAngles(1);
      std::vector<float> trajectoryDistances(1);
      std::vector<int>   trajectoryTimeouts(1);
      trajectoryDistances[0] = 1000.0;
      trajectoryTimeouts[0]  = 10.0; // 10 sec timeout

      // Walk forward
      resetTrajectoryPos(trajectoryMessage_pub); // Reset position before starting
      sendTrajectories(trajectoryDistances, trajectoryAngles, trajectoryTimeouts, trajectoryMessage_pub);
      if (ros::ok()) sleep(5); else break;

      // Wait until completion
      while(gaitControllerDone(gaitControllerStatus_client) == false && ros::ok()) sleep(1);

      // Walk back
      trajectoryTimeouts[0] = 12.0; // 12 sec timeout
      trajectoryDistances[0] = 0.0;

      sendTrajectories(trajectoryDistances, trajectoryAngles, trajectoryTimeouts, trajectoryMessage_pub);
      if (ros::ok()) sleep(15); else break;
  }

}
