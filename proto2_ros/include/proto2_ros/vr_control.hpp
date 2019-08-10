#ifndef VR_CONTROL_HPP
#define VR_CONTROL_HPP

#include "pca9685.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class VrControl : public virtual Servo
{
private:
    void vr_head_callback(const geometry_msgs::PoseStamped posestamped);
    void vr_right_hand_callback(const geometry_msgs::PoseStamped posestamped);
    void vr_left_hand_callback(const geometry_msgs::PoseStamped posestamped);

    void ik_for_left_hand(double x, double y, double z, double *degs);

    static const double angle_yaw_min = -60;
    static const double angle_yaw_max =  60;
    static const double angle_pitch_min = -45;
    static const double angle_pitch_max =  30;

    ros::Subscriber sub_head;
    ros::Subscriber sub_right_hand;
    ros::Subscriber sub_left_hand;

public:
    /* create subscriber inside constructor */
    VrControl(ros::NodeHandle n);

    /* turn on VR head tracking */
    void enable_vr_control(void);

    /* turn off VR head tracking */
    void disable_vr_control(void);
};

#endif