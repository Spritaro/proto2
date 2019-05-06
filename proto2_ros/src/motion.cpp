#include <proto2_ros/pca9685.hpp>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    Servo servo;
    double poss[8] = {0.,20.,-240.,0., 0.,-20.,-240.,0.};
    double degs[6] = {0.,-45.,45., 0.,-45.,45.};
    servo.set_and_move_servos(1000, poss, degs);

    ROS_INFO("finish");

    return(0);
}