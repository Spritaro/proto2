#include "proto2_ros/motion.hpp"

/* constructor */
Motion::Motion(ros::NodeHandle n)
{
    sub = n.subscribe("joy", 1, &Motion::joy_callback, this);
}

/* update gamepad input */
void Motion::joy_callback(const sensor_msgs::Joy joy_tmp)
{
    joy = joy_tmp;
}

void Motion::stop(void)
{
    double poss[8] = {0.,20.,-240.,0., 0.,-20.,-240.,0.};
    double degs[6] = {0.,-45.,45., 0.,-45.,45.};
    set_and_move_servos(1000, poss, degs);
}

void Motion::crouch(void)
{
    double poss[8] = {0.,20.,-200.,0., 0.,-20.,-200.,0.};
    double degs[6] = {0.,-45.,45., 0.,-45.,45.};
    set_and_move_servos(1000, poss, degs);
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