#ifndef TONNESFN_EXPERIMENTS_INDIVIDUALS_H
#define TONNESFN_EXPERIMENTS_INDIVIDUALS_H

#include <vector>
#include <cmath>
#include <string>
#include <map>

namespace individuals{

    std::map<std::string, double> smallRobotSmallControl =
        {{"stepLength"      , 185.000},
         {"stepHeight"      ,  75.000},
         {"smoothing"       ,  50.000},
         {"frequency"       ,   0.306},
         {"wagPhase"        ,   0.000},
         {"wagAmplitude_x"  ,  15.000},
         {"wagAmplitude_y"  ,  10.000},
         {"femurLength"     ,   0.000},
         {"tibiaLength"     ,   0.000},
         {"liftDuration"    ,   0.200}};

    std::map<std::string, double> largeRobotSmallControl =
        {{"stepLength"      , 185.000},
         {"stepHeight"      ,  75.000},
         {"smoothing"       ,  50.000},
         {"frequency"       ,   0.306},
         {"wagPhase"        ,   0.000},
         {"wagAmplitude_x"  ,  15.000},
         {"wagAmplitude_y"  ,  10.000},
         {"femurLength"     ,  25.000},
         {"tibiaLength"     ,  76.000},
         {"liftDuration"    ,   0.200}};

    std::map<std::string, double> largeRobotLargeControl =
        {{"stepLength"      , 215.00},
         {"stepHeight"      ,  75.00},
         {"smoothing"       ,  50.00},
         {"frequency"       ,   0.45},
         {"wagPhase"        ,   0.00},
         {"wagAmplitude_x"  ,  15.00},
         {"wagAmplitude_y"  ,  10.00},
         {"femurLength"     ,  50.00},
         {"tibiaLength"     ,  95.00},
         {"liftDuration"    ,   0.20}};

}

#endif
