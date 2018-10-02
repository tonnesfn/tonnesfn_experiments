#ifndef TONNESFN_EXPERIMENTS_INDIVIDUALS_H
#define TONNESFN_EXPERIMENTS_INDIVIDUALS_H

#include <vector>
#include <cmath>

namespace individuals{


  std::vector<double> smallRobotSmallControl = {185.0,   // stepLength
                                                 75.0,   // stepHeight
                                                 50.0,   // smoothing
                                                0.275,   // frequency
                                                  NAN,   // speed
                                                  0.0,   // wagPhase
                                                 15.0,   // wagAmp_x
                                                 10.0,   // wagAmp_y
                                                  0.0,   // femurLength
                                                  0.0,   // tibiaLength
                                                 0.20};  // liftDuration

  std::vector<double> largeRobotSmallControl = {185.0,   // stepLength
                                                 75.0,   // stepHeight
                                                 50.0,   // smoothing
                                                0.275,   // frequency
                                                  NAN,   // speed
                                                  0.0,   // wagPhase
                                                 15.0,   // wagAmp_x
                                                 10.0,   // wagAmp_y
                                                 20.0,   // femurLength
                                                 76.0,   // tibiaLength
                                                 0.20};  // liftDuration

  std::vector<double> largeRobotLargeControl = {215.0,   // stepLength
                                                 75.0,   // stepHeight
                                                 50.0,   // smoothing
                                                 0.35,   // frequency
                                                  NAN,   // speed
                                                  0.0,   // wagPhase
                                                 15.0,   // wagAmp_x
                                                 10.0,   // wagAmp_y
                                                 25.0,   // femurLength
                                                 95.0,   // tibiaLength
                                                 0.20};  // liftDuration


}

#endif
