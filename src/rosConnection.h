#pragma once

#include <ros/ros.h>

class rosConnectionHandler_t
{
    public:
        rosConnectionHandler_t (int argc_, char** argv_)
        {
            ros::init (argc_, argv_, "exp2Gui");

            nh = new ros::NodeHandle();
        }

        ~rosConnectionHandler_t ()
        {
            delete nh;
        }

        ros::NodeHandle *nodeHandle() { return nh; };

    public:
        ros::NodeHandle *nh;
};
