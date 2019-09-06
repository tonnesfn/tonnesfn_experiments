#ifndef TONNESFN_EXPERIMENTS_INDIVIDUALS_H
#define TONNESFN_EXPERIMENTS_INDIVIDUALS_H

#include <vector>
#include <cmath>
#include <string>
#include <map>

namespace individuals_highLevelSplineGait{

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

namespace individuals_lowLevelSplineGait{

    static std::map<std::string, double> conservativeIndividual = {
            {"originalSpeed",       0.0},
            {"originalStability",   0.0},
            {"frequency",           0.25},
            {"liftDuration",        0.15},
            {"p0_x",                0.0},
            {"p1_x",                0.0},
            {"p2_x",                0.0},
            {"p3_x",                0.0},
            {"p4_x",                0.0},
            {"p0_y",               75.0},
            {"p1_y",              -90.0},
            {"p2_y",               85.0},
            {"p3_y",                0.0},
            {"p4_y",              -90.0},
            {"p2_z",               75.0},
            {"p3_z",              120.0},
            {"p4_z",               75.0},
            {"difficultyFactor",    0.0},
            {"wagPhase",            0.05},
            {"wagAmplitude_x",     25.0},
            {"wagAmplitude_y",     75.0},
            {"femurLength",         0.0},
            {"tibiaLength",         0.0},
            {"splineScalingFactor", 1.0}
    };

    static std::map<std::string, double> zeroHeight = {
            {"originalSpeed", 14.559952},
            {"originalStability", -0.185659},
            {"frequency", 0.1},
            {"liftDuration", 0.182437},
            {"p0_x", 0.0},
            {"p0_y", 27.205533},
            {"p1_x", 0.0},
            {"p1_y", -130.956925},
            {"p2_x", 0.907779},
            {"p2_y", 105.398715},
            {"p2_z", 33.367528},
            {"p3_x", -1.821296},
            {"p3_y", 11.47728},
            {"p3_z", 51.076677},
            {"p4_x", -3.272923},
            {"p4_y", -76.049644},
            {"p4_z", 27.204622},
            {"difficultyFactor", 0.2},
            {"wagPhase", 0.051088},
            {"wagAmplitude_x", 6.14355},
            {"wagAmplitude_y", 2.793298},
            {"femurLength", 0.0},
            {"tibiaLength", 0.0},
            {"splineScalingFactor", 1.0}
    };

    static std::map<std::string, double> mediumHeight = {
            {"originalSpeed", 14.559952},
            {"originalStability", -0.185659},
            {"frequency", 0.1},
            {"liftDuration", 0.182437},
            {"p0_x", 0.0},
            {"p0_y", 27.205533},
            {"p1_x", 0.0},
            {"p1_y", -130.956925},
            {"p2_x", 0.907779},
            {"p2_y", 105.398715},
            {"p2_z", 33.367528},
            {"p3_x", -1.821296},
            {"p3_y", 11.47728},
            {"p3_z", 51.076677},
            {"p4_x", -3.272923},
            {"p4_y", -76.049644},
            {"p4_z", 27.204622},
            {"difficultyFactor", 0.2},
            {"wagPhase", 0.051088},
            {"wagAmplitude_x", 6.14355},
            {"wagAmplitude_y", 2.793298},
            {"femurLength", 35.0},
            {"tibiaLength", 35.0},
    };

}

namespace individuals_lowLevelAdvancedSplineGait{
    static std::map<std::string, double> doubleUneven = {
            {"originalSpeed",       14.559952},
            {"originalStability",   -0.185659},
            {"frequency",            0.1},
            {"liftDuration",         0.20},
            {"p0_y",                20.0},
            {"p1_y",              -120.0},
            {"p2_x_front",           0.0},
            {"p2_y_front",          50.0},
            {"p2_z_front",          10.0},
            {"p3_x_front",           0.0},
            {"p3_y_front",          30.0},
            {"p3_z_front",          10.0},
            {"p4_x_front",           0.0},
            {"p4_y_front",         -10.0},
            {"p4_z_front",          10.0},
            {"p2_x_rear",            0.0},
            {"p2_y_rear",          150.0},
            {"p2_z_rear",           80.0},
            {"p3_x_rear",           0.0},
            {"p3_y_rear",           10.0},
            {"p3_z_rear",           80.0},
            {"p4_x_rear",           0.0},
            {"p4_y_rear",          -150.0},
            {"p4_z_rear",           80.0},
            {"difficultyFactor",     0.2},
            {"wagPhase",             0.051088},
            {"wagAmplitude_x",       6.14355},
            {"wagAmplitude_y",       2.793298},
            {"femurLength_front",    0.0},
            {"femurLength_rear",     5.0},
            {"tibiaLength_front",    0.0},
            {"tibiaLength_rear",     5.0}
    };

    static std::map<std::string, double> unevenSmallFrontLeaning = {
            {"originalSpeed",       14.559952},
            {"originalStability",   -0.185659},
            {"frequency",            0.1},
            {"liftDuration",         0.182437},
            {"p0_y",                27.205533},
            {"p1_y",              -130.956925},
            {"p2_x_front",           0.907779},
            {"p2_y_front",         105.398715},
            {"p2_z_front",          33.367528},
            {"p3_x_front",          -1.821296},
            {"p3_y_front",          11.47728},
            {"p3_z_front",          51.076677},
            {"p4_x_front",          -3.272923},
            {"p4_y_front",         -76.049644},
            {"p4_z_front",          27.204622},
            {"p2_x_rear",            0.907779},
            {"p2_y_rear",          105.398715},
            {"p2_z_rear",           33.367528},
            {"p3_x_rear",           -1.821296},
            {"p3_y_rear",           11.47728},
            {"p3_z_rear",           51.076677},
            {"p4_x_rear",           -3.272923},
            {"p4_y_rear",          -76.049644},
            {"p4_z_rear",           27.204622},
            {"difficultyFactor",     0.2},
            {"wagPhase",             0.051088},
            {"wagAmplitude_x",       6.14355},
            {"wagAmplitude_y",       2.793298},
            {"femurLength_front",    0.0},
            {"femurLength_rear",     5.0},
            {"tibiaLength_front",    0.0},
            {"tibiaLength_rear",     5.0}
    };

    static std::map<std::string, double> unevenSmallBackLeaning = {
            {"originalSpeed",       14.559952},
            {"originalStability",   -0.185659},
            {"frequency",            0.1},
            {"liftDuration",         0.182437},
            {"p0_x",                 0.0},
            {"p0_y",                27.205533},
            {"p1_x",                 0.0},
            {"p1_y",              -130.956925},
            {"p2_x_front",           0.907779},
            {"p2_y_front",         105.398715},
            {"p2_z_front",          33.367528},
            {"p3_x_front",          -1.821296},
            {"p3_y_front",          11.47728},
            {"p3_z_front",          51.076677},
            {"p4_x_front",          -3.272923},
            {"p4_y_front",         -76.049644},
            {"p4_z_front",          27.204622},
            {"p2_x_rear",            0.907779},
            {"p2_y_rear",          105.398715},
            {"p2_z_rear",           33.367528},
            {"p3_x_rear",           -1.821296},
            {"p3_y_rear",           11.47728},
            {"p3_z_rear",           51.076677},
            {"p4_x_rear",           -3.272923},
            {"p4_y_rear",          -76.049644},
            {"p4_z_rear",           27.204622},
            {"difficultyFactor",     0.2},
            {"wagPhase",             0.051088},
            {"wagAmplitude_x",       6.14355},
            {"wagAmplitude_y",       2.793298},
            {"femurLength_front",    0.0},
            {"femurLength_rear",     5.0},
            {"tibiaLength_front",    0.0},
            {"tibiaLength_rear",     5.0}
    };

    static std::map<std::string, double> unevenLargeFrontLeaning = {
            {"originalSpeed",       14.559952},
            {"originalStability",   -0.185659},
            {"frequency",            0.1},
            {"liftDuration",         0.182437},
            {"p0_x",                 0.0},
            {"p0_y",                27.205533},
            {"p1_x",                 0.0},
            {"p1_y",              -130.956925},
            {"p2_x_front",           0.907779},
            {"p2_y_front",         105.398715},
            {"p2_z_front",          33.367528},
            {"p3_x_front",          -1.821296},
            {"p3_y_front",          11.47728},
            {"p3_z_front",          51.076677},
            {"p4_x_front",          -3.272923},
            {"p4_y_front",         -76.049644},
            {"p4_z_front",          27.204622},
            {"p2_x_rear",            0.907779},
            {"p2_y_rear",          105.398715},
            {"p2_z_rear",           33.367528},
            {"p3_x_rear",           -1.821296},
            {"p3_y_rear",           11.47728},
            {"p3_z_rear",           51.076677},
            {"p4_x_rear",           -3.272923},
            {"p4_y_rear",          -76.049644},
            {"p4_z_rear",           27.204622},
            {"difficultyFactor",     0.2},
            {"wagPhase",             0.051088},
            {"wagAmplitude_x",       6.14355},
            {"wagAmplitude_y",       2.793298},
            {"femurLength_front",   30.0},
            {"femurLength_rear",     0.0},
            {"tibiaLength_front",   40.0},
            {"tibiaLength_rear",     0.0}
    };

    static std::map<std::string, double> unevenLargeBackLeaning = {
            {"originalSpeed",      14.559952},
            {"originalStability",  -0.185659},
            {"frequency",           0.1},
            {"liftDuration",        0.182437},
            {"p0_y",                27.205533},
            {"p1_y",              -130.956925},
            {"p2_x_front",           0.907779},
            {"p2_y_front",         105.398715},
            {"p2_z_front",          33.367528},
            {"p3_x_front",          -1.821296},
            {"p3_y_front",          11.47728},
            {"p3_z_front",          51.076677},
            {"p4_x_front",          -3.272923},
            {"p4_y_front",         -76.049644},
            {"p4_z_front",          27.204622},
            {"p2_x_rear",            0.907779},
            {"p2_y_rear",          105.398715},
            {"p2_z_rear",           33.367528},
            {"p3_x_rear",           -1.821296},
            {"p3_y_rear",           11.47728},
            {"p3_z_rear",           51.076677},
            {"p4_x_rear",           -3.272923},
            {"p4_y_rear",          -76.049644},
            {"p4_z_rear",           27.204622},
            {"difficultyFactor",     0.2},
            {"wagPhase",             0.051088},
            {"wagAmplitude_x",       6.14355},
            {"wagAmplitude_y",       2.793298},
            {"femurLength_front",   30.0},
            {"femurLength_rear",     0.0},
            {"tibiaLength_front",   40.0},
            {"tibiaLength_rear",     0.0}
    };

}

#endif
