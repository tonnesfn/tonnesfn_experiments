#include <chrono>

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

long unsigned int getMs(){
  return std::chrono::duration_cast< std::chrono::milliseconds > (std::chrono::system_clock::now().time_since_epoch()).count();
}

void waitForRosInit(ros::Publisher givenPublisher, std::string givenOutput){
  if (givenPublisher.getNumSubscribers() >= 1) return;
    printf("Waiting for pub %s: ", givenOutput.c_str());
    fflush(stdout);
    while (givenPublisher.getNumSubscribers() < 1){ printf("Waiting for pub %s: %d\n", givenOutput.c_str(), givenPublisher.getNumSubscribers()); sleep(3);}
    printf("Done\n");
}

void waitForRosInit(ros::Subscriber givenSubscriber, std::string givenOutput){
  if (givenSubscriber.getNumPublishers() >= 1) return;
  printf("Waiting for sub %s: ", givenOutput.c_str());
  fflush(stdout);
  while (givenSubscriber.getNumPublishers() < 1){ printf("Waiting for sub %s: %d\n", givenOutput.c_str(), givenSubscriber.getNumPublishers()); sleep(3); }
  printf("Done\n");
}

void waitForRosInit(ros::ServiceClient givenServiceClient, std::string givenOutput){
  if (givenServiceClient.waitForExistence(ros::Duration(0.1)) == false){
    printf("Waiting for service %s: ", givenOutput.c_str());
    fflush(stdout);
  } else return;

  givenServiceClient.waitForExistence();
  printf("Done\n");
}
