#ifndef MOTION_HPP
#define MOTION_HPP

#include "proto2_ros/pca9685.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class Motion : public virtual Servo
{
private:
    void joy_callback(const sensor_msgs::Joy joy_tmp);

    ros::Subscriber sub;

public:
    Motion(ros::NodeHandle h);
    void crouch(void);
    void stop(void);

    sensor_msgs::Joy joy;
};

#endif /* MOTION_HPP */