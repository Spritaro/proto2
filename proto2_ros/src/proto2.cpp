#include "proto2_ros/vr_head_control.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");

    ros::NodeHandle n;

    // VR頭部制御のテスト
    // TODO: 全体制御が出来たら統合
    VrHeadControl vr_head_tmp(n);

    double poss[8] = {0.,20.,-240.,0., 0.,-20.,-240.,0.};
    double degs[6] = {0.,-45.,45., 0.,-45.,45.};
    while(ros::ok())
    {
        // ros::spinOnce()はサーボ制御モジュールの中で実施される
        vr_head_tmp.set_and_move_servos(100, poss, degs);
    }

    return 0;
}