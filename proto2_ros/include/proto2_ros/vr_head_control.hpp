#ifndef VR_HEAD_CONTROL_HPP
#define VR_HEAD_CONTROL_HPP

#include "pca9685.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class VrHeadControl : public virtual Servo
{
private:
    void vr_head_callback(const geometry_msgs::PoseStamped posestamped);

    bool is_vr_head_control_enabled;

    static const double angle_yaw_min = -60;
    static const double angle_yaw_max =  60;
    static const double angle_pitch_min = -60;
    static const double angle_pitch_max =  60;

    ros::Subscriber sub;

public:
    /* create subscriber inside constructor */
    VrHeadControl(ros::NodeHandle n);

    /* turn on VR head tracking */
    void enable_vr_head_control(void);

    /* turn off VR head tracking */
    void disable_vr_head_control(void);
};

#endif