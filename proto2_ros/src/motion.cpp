#include "proto2_ros/motion.hpp"

/* constructor */
Motion::Motion(ros::NodeHandle n)
{
    sub = n.subscribe("joy", 1, &Motion::joy_callback, this);

    // create empty joy message
    for(uint8_t i=0; i<12; i++)
        joy.buttons.push_back(0);
    for(uint8_t i=0; i<6; i++)
        joy.axes.push_back(0.0);
}

/* update gamepad input */
void Motion::joy_callback(const sensor_msgs::Joy joy_tmp)
{
    // ROS_INFO("Joy message received");
    joy = joy_tmp;
}

/* forward */
void Motion::walk_forward_pre(void)
{
    double poss[8] = {0,20,-230,0, 0,-20,-235,0};
    double degs[6] = {0,-45,45, 0,-45,45};
    set_and_move_servos(80, poss, degs);
}

void Motion::walk_forward_1(void)
{
    double poss0[8] = {-15,20,-230,0, 15,-20,-210,0};
    double degs0[6] = {10,-45,45, -10,-45,45};
    set_and_move_servos(80, poss0, degs0);

    double poss1[8] = {-15,20,-230,0, 15,-20,-235,0};
    double degs1[6] = {10,-45,45, -10,-45,45};
    set_and_move_servos(80, poss1, degs1);
}

void Motion::walk_forward_2(void)
{
    double poss0[8] = {15,20,-210,0, -15,-20,-230,0};
    double degs0[6] = {-10,-45,45,  10,-45,45};
    set_and_move_servos(80, poss0, degs0);

    double poss1[8] = {15,20,-235,0, -15,-20,-230,0};
    double degs1[6] = {-10,-45,45,  10,-45,45};
    set_and_move_servos(80, poss1, degs1);
}

/* for stability */
void Motion::crouch_little(void)
{
    double poss[8] = {0.,20.,-200.,0., 0.,-20.,-200.,0.};
    double degs[6] = {0.,-45.,45., 0.,-45.,45.};
    set_and_move_servos(80, poss, degs);
}

/* stop */
void Motion::stop(void)
{
    double poss[8] = {0.,20.,-240.,0., 0.,-20.,-240.,0.};
    double degs[6] = {0.,-45.,45., 0.,-45.,45.};
    set_and_move_servos(100, poss, degs);
}


// int main(int argc, char **argv)
// {
//     Servo servo;
//     double poss[8] = {0.,20.,-240.,0., 0.,-20.,-240.,0.};
//     double degs[6] = {0.,-45.,45., 0.,-45.,45.};
//     servo.set_and_move_servos(1000, poss, degs);

//     ROS_INFO("finish");

//     return(0);
// }