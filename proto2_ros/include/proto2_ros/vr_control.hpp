#ifndef VR_CONTROL_HPP
#define VR_CONTROL_HPP

#include "pca9685.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>

class VrControl : public virtual Servo
{
private:
    void vr_head_callback(const geometry_msgs::PoseStamped posestamped);
    void vr_right_hand_callback(const geometry_msgs::PoseStamped posestamped);
    void vr_left_hand_callback(const geometry_msgs::PoseStamped posestamped);
    void vr_joy_callback(const sensor_msgs::Joy joy_tmp);

    void ik_for_left_hand(double x, double y, double z, double *degs);

    static const double angle_yaw_min = -60;
    static const double angle_yaw_max =  60;
    static const double angle_pitch_min = -45;
    static const double angle_pitch_max =  30;

    ros::Subscriber sub_head;
    ros::Subscriber sub_right_hand;
    ros::Subscriber sub_left_hand;
    ros::Subscriber sub_joy;

public:
    /* create subscriber inside constructor */
    VrControl(ros::NodeHandle n);

    /* turn on VR head tracking */
    void enable_vr_control(void);

    /* turn off VR head tracking */
    void disable_vr_control(void);

    /* variables */
    sensor_msgs::Joy vr_joy;

    /* index number of gamepad */
    static const uint8_t VR_BUTTON_B = 0;      // Oculus_CrossPlatform_Button1 1
    static const uint8_t VR_BUTTON_A = 1;      // Oculus_CrossPlatform_Button2 0
    static const uint8_t VR_BUTTON_Y = 2;      // Oculus_CrossPlatform_Button3 3
    static const uint8_t VR_BUTTON_X = 3;      // Oculus_CrossPlatform_Button4 2
    static const uint8_t VR_BUTTON_NUM = 4;

    static const uint8_t VR_TRIGGER_IL = 0;    // Oculus_CrossPlatform_PrimaryIndexTrigger
    static const uint8_t VR_TRIGGER_IR = 1;    // Oculus_CrossPlatform_SecondaryIndexTrigger
    static const uint8_t VR_TRIGGER_HL = 2;    // Oculus_CrossPlatform_PrimaryHandTrigger
    static const uint8_t VR_TRIGGER_HR = 3;    // Oculus_CrossPlatform_SecondaryHandTrigger
    static const uint8_t VR_STICK_LX = 4;      // Oculus_CrossPlatform_PrimaryThumbstickHorizontal
    static const uint8_t VR_STICK_LY = 5;      // Oculus_CrossPlatform_PrimaryThumbstickVertical
    static const uint8_t VR_STICK_RX = 6;      // Oculus_CrossPlatform_SecondaryThumbstickHorizontal
    static const uint8_t VR_STICK_RY = 7;      // Oculus_CrossPlatform_SecondaryThumbstickVertical
    static const uint8_t VR_AXIS_NUM = 8;
};

#endif