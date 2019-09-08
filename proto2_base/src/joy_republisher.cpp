#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "proto2_base/motion.hpp"
#include "proto2_base/vr_control.hpp"

ros::Publisher joy_pub;
ros::Subscriber joy_sub;

void joy_callback(const sensor_msgs::Joy vr_joy)
{
    /*
    BUTTON_A <- VR_BUTTON_A      // Oculus_CrossPlatform_Button2 0
    BUTTON_B <- VR_BUTTON_B      // Oculus_CrossPlatform_Button1 1
    BUTTON_X <- VR_BUTTON_X      // Oculus_CrossPlatform_Button4 2
    BUTTON_Y <- VR_BUTTON_Y      // Oculus_CrossPlatform_Button3 3
    BUTTON_BACK  <- VR_BUTTON_LS // Oculus_CrossPlatform_Button4 8
    BUTTON_START <- VR_BUTTON_RS // Oculus_CrossPlatform_Button4 9
    BUTTON_LS <- None
    BUTTON_RS <- None

    BUTTON_LB <- VR_TRIGGER_IL    // Oculus_CrossPlatform_PrimaryIndexTrigger
    BUTTON_RB <- VR_TRIGGER_IR    // Oculus_CrossPlatform_SecondaryIndexTrigger
    BUTTON_LT <- VR_TRIGGER_HL    // Oculus_CrossPlatform_PrimaryHandTrigger
    BUTTON_RT <- VR_TRIGGER_HR    // Oculus_CrossPlatform_SecondaryHandTrigger
    STICK_LX <- VR_STICK_LX       // Oculus_CrossPlatform_PrimaryThumbstickHorizontal
    STICK_LY <- VR_STICK_LY       // Oculus_CrossPlatform_PrimaryThumbstickVertical
    STICK_RX <- VR_STICK_RX       // Oculus_CrossPlatform_SecondaryThumbstickHorizontal
    STICK_RY <- VR_STICK_RY       // Oculus_CrossPlatform_SecondaryThumbstickVertical
    CROSS_X <- None
    CROSS_Y <- None
    */

    sensor_msgs::Joy joy;

    joy.buttons.push_back(vr_joy.buttons[VrControl::VR_BUTTON_X]);
    joy.buttons.push_back(vr_joy.buttons[VrControl::VR_BUTTON_A]);
    joy.buttons.push_back(vr_joy.buttons[VrControl::VR_BUTTON_B]);
    joy.buttons.push_back(vr_joy.buttons[VrControl::VR_BUTTON_Y]);
    joy.buttons.push_back( (int)(vr_joy.axes[VrControl::VR_TRIGGER_IL] > 0.5) );
    joy.buttons.push_back( (int)(vr_joy.axes[VrControl::VR_TRIGGER_IR] > 0.5) );
    joy.buttons.push_back( (int)(vr_joy.axes[VrControl::VR_TRIGGER_HL] > 0.5) );
    joy.buttons.push_back( (int)(vr_joy.axes[VrControl::VR_TRIGGER_HR] > 0.5) );
    joy.buttons.push_back(vr_joy.buttons[VrControl::VR_BUTTON_LS]);
    joy.buttons.push_back(vr_joy.buttons[VrControl::VR_BUTTON_RS]);

    joy.axes.push_back(-vr_joy.axes[VrControl::VR_STICK_LX]);
    joy.axes.push_back( vr_joy.axes[VrControl::VR_STICK_LY]);
    joy.axes.push_back(-vr_joy.axes[VrControl::VR_STICK_RX]);
    joy.axes.push_back( vr_joy.axes[VrControl::VR_STICK_RY]);

    joy_pub.publish(joy);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "joy_republisher");

    ros::NodeHandle n;

    joy_pub = n.advertise<sensor_msgs::Joy>("/joy", 1);
    joy_sub = n.subscribe("/oculus/joy", 1, joy_callback);

    ros::spin();

    return 0;
}