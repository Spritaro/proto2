#include "proto2_ros/proto2.hpp"

/* constructor */
Proto2::Proto2(ros::NodeHandle n) :
    motion(n),
    vr_head_control(n)
{
}

void Proto2::whole_control(void)
{
    /* initial setting*/
    vr_head_control.enable_vr_head_control();

    /* main loop */
    while(ros::ok())
    {
        ROS_INFO("Stop");
        motion.stop();
        motion.crouch();
    }

    ROS_INFO("Done");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");

    ros::NodeHandle n;

    // 全体制御モジュール
    Proto2 proto2(n);

    proto2.whole_control();


    return 0;
}