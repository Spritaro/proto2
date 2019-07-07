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
    set_and_move_servos(50, poss, degs);
}

void Motion::walk_forward_1(void)
{
    double poss0[8] = {-15,20,-230,0, 15,-20,-210,0};
    double degs0[6] = {10,-45,45, -10,-45,45};
    set_and_move_servos(50, poss0, degs0);

    double poss1[8] = {-15,20,-230,0, 15,-20,-235,0};
    double degs1[6] = {10,-45,45, -10,-45,45};
    set_and_move_servos(50, poss1, degs1);
}

void Motion::walk_forward_2(void)
{
    double poss0[8] = {15,20,-210,0, -15,-20,-230,0};
    double degs0[6] = {-10,-45,45, 10,-45,45};
    set_and_move_servos(50, poss0, degs0);

    double poss1[8] = {15,20,-235,0, -15,-20,-230,0};
    double degs1[6] = {-10,-45,45, 10,-45,45};
    set_and_move_servos(50, poss1, degs1);
}

/* backward */
void Motion::walk_backward_pre(void)
{
    double poss[8] = {0,20,-230,0, 0,-20,-235,0};
    double degs[6] = {0,-45,45, 0,-45,45};
    set_and_move_servos(50, poss, degs);
}

void Motion::walk_backward_1(void)
{
    double poss0[8] = {15,20,-230,0, -15,-20,-210,0};
    double degs0[6] = {-10,-45,45, 10,-45,45};
    set_and_move_servos(50, poss0, degs0);

    double poss1[8] = {15,20,-230,0, -15,-20,-235,0};
    double degs1[6] = {-10,-45,45, 10,-45,45};
    set_and_move_servos(50, poss1, degs1);
}

void Motion::walk_backward_2(void)
{
    double poss0[8] = {-15,20,-210,0, 15,-20,-230,0};
    double degs0[6] = {10,-45,45, -10,-45,45};
    set_and_move_servos(50, poss0, degs0);

    double poss1[8] = {-15,20,-235,0, 15,-20,-230,0};
    double degs1[6] = {10,-45,45, -10,-45,45};
    set_and_move_servos(50, poss1, degs1);
}

/* side */
void Motion::walk_right_pre(void)
{
    double poss0[8] = {0,0,-215,0, 0,0,-225,0};
    double degs0[6] = {0,-45,45, 0,-45,45};
    set_and_move_servos(50, poss0, degs0);
}

void Motion::walk_right(void)
{
    double poss1[8] = {0,25,-220,0, 0,-25,-200,0};
    double degs1[6] = {0,-45,45, 0,-45,45};
    set_and_move_servos(50, poss1, degs1);

    double poss2[8] = {0,50,-220,0, 0,-50,-220,0};
    double degs2[6] = {0,-45,45, 0,-45,45};
    set_and_move_servos(50, poss2, degs2);

    double poss3[8] = {0,25,-200,0, 0,-25,-220,0};
    double degs3[6] = {0,-45,45, 0,-45,45};
    set_and_move_servos(50, poss3, degs3);

    double poss4[8] = {0,0,-220,0, 0,0,-220,0};
    double degs4[6] = {0,-45,45, 0,-45,45};
    set_and_move_servos(50, poss4, degs4);
}

void Motion::walk_left_pre(void)
{
    double poss0[8] = {0,0,-225,0, 0,0,-215,0};
    double degs0[6] = {0,-45,45, 0,-45,45};
    set_and_move_servos(50, poss0, degs0);
}

void Motion::walk_left(void)
{
    double poss1[8] = {0,25,-200,0, 0,-25,-220,0};
    double degs1[6] = {0,-45,45, 0,-45,45};
    set_and_move_servos(50, poss1, degs1);

    double poss2[8] = {0,50,-220,0, 0,-50,-220,0};
    double degs2[6] = {0,-45,45, 0,-45,45};
    set_and_move_servos(50, poss2, degs2);

    double poss3[8] = {0,25,-220,0, 0,-25,-200,0};
    double degs3[6] = {0,-45,45, 0,-45,45};
    set_and_move_servos(50, poss3, degs3);

    double poss4[8] = {0,0,-220,0, 0,0,-220,0};
    double degs4[6] = {0,-45,45, 0,-45,45};
    set_and_move_servos(50, poss4, degs4);
}

/* turn */
void Motion::turn_right(void)
{
    double poss0[8] = {40,40,-220,-10, -40,-40,-220,10};
    double degs0[6] = {-20,-45,45, 20,-45,45};
    set_and_move_servos(50, poss0, degs0);
    double poss1[8] = {0,0,-220,0, 0,0,-220,0};
    double degs1[6] = {0,-45,45, 0,-45,45};
    set_and_move_servos(200, poss1, degs1);
}
void Motion::turn_left(void)
{
    double poss0[8] = {-40,40,-220,-10, 40,-40,-220,10};
    double degs0[6] = {20,-45,45, -20,-45,45};
    set_and_move_servos(50, poss0, degs0);
    double poss1[8] = {0,0,-220,0, 0,0,-220,0};
    double degs1[6] = {0,-45,45, 0,-45,45};
    set_and_move_servos(200, poss1, degs1);
}

/* for stability */
void Motion::crouch_little(void)
{
    double poss[8] = {0.,20.,-200.,0., 0.,-20.,-200.,0.};
    double degs[6] = {0.,-45.,45., 0.,-45.,45.};
    set_and_move_servos(100, poss, degs);
}

/* punch */
void Motion::punch_right(void)
{
    double poss[8] = {0.,20.,-230.,0., 0.,-20.,-230.,0.};
    double degs0[6] = {0.,-45.,45., 90.,-100.,90.};
    set_and_move_servos(100, poss, degs0);
    double degs1[6] = {0.,-45.,45., 90.,-100.,90.};
    set_and_move_servos(100, poss, degs1);
    double degs2[6] = {0.,-45.,45., 90.,0.,-45.};
    set_and_move_servos(200, poss, degs2);
}

void Motion::punch_left(void)
{
    double poss[8] = {0.,20.,-230.,0., 0.,-20.,-230.,0.};
    double degs0[6] = {90.,-100.,90., 0.,-45.,45.};
    set_and_move_servos(100, poss, degs0);
    double degs1[6] = {90.,-100.,90., 0.,-45.,45.};
    set_and_move_servos(100, poss, degs1);
    double degs2[6] = {90.,0.,-45., 0.,-45.,45.};
    set_and_move_servos(200, poss, degs2);
}

/* throw dice */
void Motion::throw_dice(void)
{
    // sit down
    double poss0[8] = {0,0,-161,0, 0,0,-161,0};
    double degs0[6] = {0,-80,90, 0,-80,90};
    set_and_move_servos(500, poss0, degs0);
    set_and_move_servos(100, poss0, degs0);
    // throw
    double poss1[8] = {0,0,-161,0, 0,0,-161,0};
    double degs1[6] = {90,-110,90, 90,-110,90};
    set_and_move_servos(500, poss1, degs1);
    set_and_move_servos(100, poss1, degs1);
}

/* get down */
void Motion::get_down(void)
{
    double poss[8] = {0.,40.,-160.,0., 0.,-40.,-160.,0.};
    double degs[6] = {0.,-45.,45., 0.,-45.,45.};
    set_and_move_servos(100, poss, degs);
}

/* get up from fall */
void Motion::getup_front(void)
{
    // ready arms
    double poss0[8] = {0,100,-130,0, 0,-100,-130,0};
    double degs0[6] = {0,45,-60, 0,45,-60};
    set_and_move_servos(200, poss0, degs0);
    set_and_move_servos(100, poss0, degs0);
    // get ready
    double poss1[8] = {10,139,-80,0, 10,-139,-80,0};
    double degs1[6] = {90,45,-60, 90,45,-60};
    set_and_move_servos(200, poss1, degs1);
    set_and_move_servos(100, poss1, degs1);
    // get up
    double poss2[8] = {10,139,-80,0, 10,-139,-80,0};
    double degs2[6] = {70,-90,90, 70,-90,90};
    set_and_move_servos(150, poss2, degs2);
    set_and_move_servos(100, poss2, degs2);
    // move back
    double poss3[8] = {0,139,-80,0, 0,-139,-80,0};
    double degs3[6] = {70,-90,90, 70,-90,90};
    set_and_move_servos(180, poss3, degs3);
    set_and_move_servos(900, poss3, degs3);
    // close legs
    double poss4[8] = {0,20,-160,0, 0,-20,-160,0};
    double degs4[6] = {70,-45,60, 70,-45,60};
    set_and_move_servos(200, poss4, degs4);
    set_and_move_servos(100, poss4, degs4);
    // stand up
    double poss5[8] = {0,20,-230,0, 0,-20,-230,0};
    double degs5[6] = {0,-45,60, 0,-45,60};
    set_and_move_servos(200, poss5, degs5);
    set_and_move_servos(100, poss5, degs5);
}

void Motion::getup_back(void)
{
    // ready arms
    double poss0[8] = {0,100,-130,0, 0,-100,-130,0};
    double degs0[6] = {0,45,-60, 0,45,-60};
    set_and_move_servos(200, poss0, degs0);
    set_and_move_servos(100, poss0, degs0);
    // get ready
    double poss1[8] = {-20,139,-80,0, -20,-139,-80,0};
    double degs1[6] = {-90,45,-60, -90,45,-60};
    set_and_move_servos(200, poss1, degs1);
    set_and_move_servos(100, poss1, degs1);
    // get up
    double poss2[8] = {-20,139,-80,0, -20,-139,-80,0};
    double degs2[6] = {-70,-90,90, -70,-90,90};
    set_and_move_servos(100, poss2, degs2);
    set_and_move_servos(100, poss2, degs2);
    // move back
    double poss3[8] = {0,139,-80,0, 0,-139,-80,0};
    double degs3[6] = {-70,-90,90, -70,-90,90};
    set_and_move_servos(180, poss3, degs3);
    set_and_move_servos(900, poss3, degs3);
    // close legs
    double poss4[8] = {0,20,-160,0, 0,-20,-160,0};
    double degs4[6] = {-70,-45,60, -70,-45,60};
    set_and_move_servos(200, poss4, degs4);
    set_and_move_servos(100, poss4, degs4);
    // stand up
    double poss5[8] = {0,20,-230,0, 0,-20,-230,0};
    double degs5[6] = {0,-45,60, 0,-45,60};
    set_and_move_servos(200, poss5, degs5);
    set_and_move_servos(100, poss5, degs5);
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