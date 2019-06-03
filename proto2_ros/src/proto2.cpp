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
    ROS_INFO("Start main loop");
    while(ros::ok())
    {
        ros::spinOnce();

        if(motion.joy.axes[motion.STICK_LY] > 0)
        {
            /* forward */
            motion.walk_forward_pre();
            while(ros::ok())
            {
                motion.walk_forward_1();
                if(motion.joy.axes[motion.STICK_LY] <= 0) break;
                motion.walk_forward_2();
                if(motion.joy.axes[motion.STICK_LY] <= 0) break;
            }
            motion.crouch_little();
        }
        else if(motion.joy.axes[motion.STICK_LY] < 0)
        {
            /* backward */
            motion.walk_backward_pre();
            while(ros::ok())
            {
                motion.walk_backward_1();
                if(motion.joy.axes[motion.STICK_LY] >= 0) break;
                motion.walk_backward_2();
                if(motion.joy.axes[motion.STICK_LY] >= 0) break;
            }
            motion.crouch_little();
        }
        else if(motion.joy.buttons[motion.BUTTON_RB] == 1)
        {
            /* right turn */
            while(ros::ok())
            {
                motion.turn_right();
                if(motion.joy.buttons[motion.BUTTON_RB] != 1) break;
            }
            motion.crouch_little();
        }
        else if(motion.joy.buttons[motion.BUTTON_LB] == 1)
        {
            /* left turn */
            while(ros::ok())
            {
                motion.turn_left();
                if(motion.joy.buttons[motion.BUTTON_LB] != 1) break;
            }
            motion.crouch_little();
        }
        else
        {
            motion.stop();
        }
    }
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