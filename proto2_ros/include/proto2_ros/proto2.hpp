#ifndef PROTO2_HPP
#define PROTO2_HPP

#include "proto2_ros/motion.hpp"
#include "proto2_ros/vr_head_control.hpp"
#include <ros/ros.h>

class Proto2
{
private:

public:
    /* constructor */
    Proto2(ros::NodeHandle n);

    /* do everything in this function */
    void whole_control(void);

    /* sub modules */
    Motion motion;
    VrHeadControl vr_head_control;
};

#endif /* PROTO2_HPP */