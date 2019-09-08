#ifndef MOTION_HPP
#define MOTION_HPP

#include "proto2_base/pca9685.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class Motion : public virtual Servo
{
private:
    void joy_callback(const sensor_msgs::Joy joy_tmp);

    ros::Subscriber sub;

public:
    Motion(ros::NodeHandle h);

    /* forward */
    void walk_forward_pre(void);
    void walk_forward_1(void);
    void walk_forward_2(void);

    /* backward */
    void walk_backward_pre(void);
    void walk_backward_1(void);
    void walk_backward_2(void);

    /* side */
    void walk_right_pre(void);
    void walk_right(void);
    void walk_left_pre(void);
    void walk_left(void);

    /* turn */
    void turn_right(void);
    void turn_left(void);

    /* punch */
    void punch_right(void);
    void punch_left(void);
    void punch_front(void);

    /* throw dice (10cm cube) */
    void throw_dice(void);

    /* get down */
    void get_down(void);

    /* getup */
    void getup_front(void);
    void getup_back(void);

    /* poses for preliminary */
    void fighting_pose(void);
    void holdup_pose(void);
    void t_pose(void);
    
    /* for stability */
    void crouch_little(void);

    /* stop */
    void stop(void);
    void stop_and_look_right(void);
    void stop_and_look_left(void);

    /* variables */
    sensor_msgs::Joy joy;

    /* index number of gamepad */
    static const uint8_t BUTTON_X = 0;
    static const uint8_t BUTTON_A = 1;
    static const uint8_t BUTTON_B = 2;
    static const uint8_t BUTTON_Y = 3;
    static const uint8_t BUTTON_LB = 4;
    static const uint8_t BUTTON_RB = 5;
    static const uint8_t BUTTON_LT = 6;
    static const uint8_t BUTTON_RT = 7;
    static const uint8_t BUTTON_BACK = 8;
    static const uint8_t BUTTON_START = 9;
    static const uint8_t BUTTON_LS = 10;
    static const uint8_t BUTTON_RS = 11;
    static const uint8_t BUTTON_NUM = 12;

    static const uint8_t STICK_LX = 0;
    static const uint8_t STICK_LY = 1;
    static const uint8_t STICK_RX = 2;
    static const uint8_t STICK_RY = 3;
    static const uint8_t CROSS_X = 4;
    static const uint8_t CROSS_Y = 5;
    static const uint8_t AXIS_NUM = 6;
};

#endif /* MOTION_HPP */