#include "proto2_base/pca9685.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

/* variables */
int32_t id = 0;
double degs[] = {0.,0.,0.,0., 0.,0.,0.,0., 0.,0.,0., 0.,0.,0.};

/* index number of gamepad */
const uint8_t BUTTON_X = 0;
const uint8_t BUTTON_A = 1;
const uint8_t BUTTON_B = 2;
const uint8_t BUTTON_Y = 3;
const uint8_t BUTTON_LB = 4;
const uint8_t BUTTON_RB = 5;
const uint8_t BUTTON_LT = 6;
const uint8_t BUTTON_RT = 7;
const uint8_t BUTTON_BACK = 8;
const uint8_t BUTTON_START = 9;
const uint8_t BUTTON_LS = 10;
const uint8_t BUTTON_RS = 11;

const uint8_t STICK_LX = 0;
const uint8_t STICK_LY = 1;
const uint8_t STICK_RX = 2;
const uint8_t STICK_RY = 3;
const uint8_t CROSS_X = 4;
const uint8_t CROSS_Y = 5;

/* update gamepad input */
void joy_callback(const sensor_msgs::Joy joy_tmp)
{
    double deg = 0.0;

    if(joy_tmp.axes[CROSS_X] > 0)
        id++;
    if(joy_tmp.axes[CROSS_X] < 0)
        id--;

    if(joy_tmp.axes[CROSS_Y] > 0)
        deg = 1.0;
    if(joy_tmp.axes[CROSS_Y] < 0)
        deg = -1.0;

    if(joy_tmp.buttons[BUTTON_Y] == 1)
        deg = 10.0;
    if(joy_tmp.buttons[BUTTON_A] == 1)
        deg = -10.0;

    if(id < 0)
        id = 0;
    if(id > 13)
        id = 13;

    degs[id] += deg;

    ROS_INFO("%d %d", id, (int)degs[id]);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joy", 1, &joy_callback);

    Servo servo;

    while(1)
    {
        servo.set_and_move_servos(50, degs);
    }

    return 0;
}