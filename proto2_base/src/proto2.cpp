#include "proto2_ros/proto2.hpp"

/* constructor */
Proto2::Proto2(ros::NodeHandle n) :
    motion(n),
    vr_control(n)
{
}

void Proto2::whole_control(void)
{
    /* initial setting*/
    vr_control.disable_vr_control();

    /* main loop */
    ROS_INFO("Start main loop");
    while(ros::ok())
    {
        ros::spinOnce();

        /* right trigger special actions */
        if(motion.joy.buttons[motion.BUTTON_RT] == 1)
        {
            if(motion.joy.buttons[motion.BUTTON_A] == 1)
                vr_control.enable_vr_control();

            else if(motion.joy.buttons[motion.BUTTON_B] == 1)
                vr_control.disable_vr_control();

            continue;
        }
        if(vr_control.vr_joy.axes[vr_control.VR_TRIGGER_HR] > 0.5)
        {
            if(vr_control.vr_joy.buttons[vr_control.VR_BUTTON_A] == 1)
                vr_control.enable_vr_control();

            else if(vr_control.vr_joy.buttons[vr_control.VR_BUTTON_B] == 1)
                vr_control.disable_vr_control();

            continue;
        }

        /* left trigger special actions */
        if(motion.joy.buttons[motion.BUTTON_LT] == 1)
        {
            if(motion.joy.buttons[motion.BUTTON_Y] == 1)
                motion.getup_front();

            else if(motion.joy.buttons[motion.BUTTON_A] == 1)
                motion.getup_back();

            while(ros::ok() && motion.joy.buttons[motion.BUTTON_B] == 1)
                motion.t_pose();
            
            while(ros::ok() && motion.joy.buttons[motion.BUTTON_X] == 1)
                motion.holdup_pose();
            
            while(ros::ok() && motion.joy.buttons[motion.BUTTON_RB] == 1)
                motion.fighting_pose();

            while(ros::ok() && motion.joy.axes[motion.STICK_LX] < 0)
                motion.stop_and_look_right();

            while(ros::ok() && motion.joy.axes[motion.STICK_LX] > 0)
                motion.stop_and_look_left();

            continue;
        }

        /* normal motions */
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
        else if(motion.joy.axes[motion.STICK_LX] < 0)
        {
            /* walk right */
            motion.walk_right_pre();
            while(ros::ok())
            {
                motion.walk_right();
                if(motion.joy.axes[motion.STICK_LX] >= 0) break;
            }
            motion.crouch_little();
        }
        else if(motion.joy.axes[motion.STICK_LX] > 0)
        {
            /* walk left */
            motion.walk_left_pre();
            while(ros::ok())
            {
                motion.walk_left();
                if(motion.joy.axes[motion.STICK_LX] <= 0) break;
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
        else if(motion.joy.buttons[motion.BUTTON_B] == 1)
        {
            /* punch right */
            motion.punch_right();
        }
        else if(motion.joy.buttons[motion.BUTTON_X] == 1)
        {
            /* punch left */
            motion.punch_left();
        }
        else if(motion.joy.buttons[motion.BUTTON_Y] == 1)
        {
            /* throw dice */
            motion.throw_dice();
        }
        else if(motion.joy.buttons[motion.BUTTON_A] == 1)
        {
            /* get down */
            while(ros::ok())
            {
                motion.get_down();
                if(motion.joy.buttons[motion.BUTTON_A] != 1) break;
            }
        }
        else if(motion.joy.buttons[motion.BUTTON_BACK] == 1)
        {
            /* poweroff servos */
            while(ros::ok())
            {
                motion.poweroff_servos();
                if(motion.joy.buttons[motion.BUTTON_START] == 1) break;
            }
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