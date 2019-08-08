#include "proto2_ros/pca9685.hpp"
#include <wiringPiI2C.h>
#include <unistd.h>

/*
 * this module controls servo motors from PCA9685 with I2C interface
 */

/* initialize parameters */
const double Servo::channels[] = {0,1,2,3, 7,6,5,4, 13,14,15, 10,9,8, 11,12};
const double Servo::ccws[]     = {1,1,-1,1, -1,1,-1,-1, -1,-1,1, 1,1,-1, 1,1};
const double Servo::offsets[]  = {-18*2,-64,-39,-30*2, -22*2,21,42,-22*2, 5,20,-30, -13,27,-22, 0,15};

double Servo::current_degs[Servo::SERVONUM];
double Servo::end_degs[Servo::SERVONUM];

Servo::Servo(void) :
    loop_rate(1000.0/STEP_MS)
{
    /* create I2C interface */
    ROS_INFO("setting up I2C interface");
    i2c_fd = wiringPiI2CSetup(0x40);
    if(i2c_fd == -1)
        ROS_ERROR("failed to setup I2C interface");
    else
        ROS_INFO("successfully setup I2C interface");

    /* setup PCA9685 */
    setup_pca9685();
    set_pwm_freq(50.0);
    
    /* initialize variables */
    ROS_INFO("initializing variables");
    double tmp_array[] = {0.,0.,0.,0., 0.,0.,0.,0., 0.,0.,0.,0., 0.,0.,0.,0.};
    copy_array(current_degs, tmp_array, 0, SERVONUM);
    copy_array(end_degs, tmp_array, 0, SERVONUM);
}

Servo::~Servo(void)
{
    /* do nothing */
}

void Servo::setup_pca9685(void)
{
    ROS_INFO("setting up PCA9685");
    wiringPiI2CWrite(i2c_fd, 0x06);             // software reset
    wiringPiI2CWriteReg8(i2c_fd, 0x01, 0x04);   // switch to totem pole structure
    wiringPiI2CWriteReg8(i2c_fd, 0x00, 0x01);   // wake up and use inernal oscillator
    sleep(0.005);                               // wait for oscillator
}

void Servo::set_pwm_freq(const double freq_hz)
{
    ROS_INFO("chainging pwm frequency");
    double prescaleval = 25000000.0;    // 25MHz
    prescaleval /= 4096.0;              // 12-bit
    prescaleval /= freq_hz;
    prescaleval -= 1.0;
    unsigned int prescale = static_cast<unsigned int>(prescaleval);
    wiringPiI2CWriteReg8(i2c_fd, 0x00, 0x11);    // sleep
    wiringPiI2CWriteReg8(i2c_fd, 0xfe, prescale);
    wiringPiI2CWriteReg8(i2c_fd, 0x00, 0x01);    // wake up
    sleep(0.005);                               // wait for oscillator
    wiringPiI2CWriteReg8(i2c_fd, 0x00, 0x81);    // restart PWM
}

void Servo::set_pwm(const unsigned int channel, const unsigned int pwm)
{
    // PWM ON registers are not changed
    // only PWM OFF registers are changed
    wiringPiI2CWriteReg8(i2c_fd, 0x08+4*channel, pwm & 0xff);  // lower byte
    wiringPiI2CWriteReg8(i2c_fd, 0x09+4*channel, pwm >> 8);    // higher byte
}

/* 
 * calculate servo angles from foot position x y z and foot angle a
 * calculated angles are assigned to array degs
*/
void Servo::ik(const double x, const double y, const double z, const double a, double *degs)
{
    double l1 = 50.0;
    double l2 = 50.0;
    double l3 = 60.0;
    double l4 = 50.0;
    double l5 = 50.0;

    // degrees to radians
    double a_ = radians(a);

    // get th1 (thigh roll) rotation
    double th1 = atan2(y, -z);

    // get z_ axis
    double z_ = z / cos(th1);
    z_ -= -l1 -l3 -l5;

    // get th2 (thigh pitch) and th3 (ankle pitch)
    double ph1 = atan2(x, -z_);
    double l = -z_ / cos(ph1);
    double ph2 = acos( (l*l + l2*l2 - l4*l4) / (2*l*l2) );
    double ph3 = acos( (l*l + l4*l4 - l2*l2) / (2*l*l4) );
    double th2 = ph1 + ph2;
    double th3 = ph3 - ph1;

    // ankle roll
    double th4 = th1 + a_;

    // radians to degrees
    degs[0] = degrees(th1);
    degs[1] = degrees(th2);
    degs[2] = degrees(th3);
    degs[3] = degrees(th4);
}

void Servo::poweroff_servos(void)
{
    for(int i=0; i<SERVONUM; i++)
    {
        set_pwm(i, 0);
    }
    ros::spinOnce();
    loop_rate.sleep();
}

/* only set target angles */
void Servo::set_head_angles(const double target_deg_yaw, const double target_deg_pitch)
{
    end_degs[SERVONUM-2] = target_deg_yaw;
    end_degs[SERVONUM-1] = target_deg_pitch;
}

void Servo::set_body_angles(const double *target_degs)
{
    copy_array(end_degs, target_degs, 0, SERVONUM-2);
}

/* move servos to set target angles */
void Servo::move_servos(const unsigned int duration)
{
    // 1ms -> STEP_MS ms
    unsigned int nb_step = duration / STEP_MS;
    if(nb_step < 1) nb_step = 1;

    // ROS_INFO("copying current degs to start degs");
    double start_degs[SERVONUM];
    copy_array(start_degs, current_degs, 0, SERVONUM);
    for(int t = 1; t < nb_step+1; t++)
    {
        // update servo angles
        // ROS_INFO("update servo angles");
        for(int id = 0; id < SERVONUM; id++)
            current_degs[id] = ((end_degs[id]-start_degs[id]) * t/nb_step + start_degs[id]);

        // copy to temporary array and correct certain angles
        // ROS_INFO("copy to temporary array and correct certain angles");
        double degs_tmp[SERVONUM];
        for(int id = 0; id < SERVONUM; id++)
            degs_tmp[id] = current_degs[id];
        degs_tmp[0] *= 2.0;
        degs_tmp[3] *= 2.0;
        degs_tmp[4] *= 2.0;
        degs_tmp[7] *= 2.0;

        // move servos
        // ROS_INFO("move servos");
        for(int id = 0; id < SERVONUM; id++)
            set_pwm( channels[id], deg2width((degs_tmp[id]+offsets[id])*ccws[id]) );

        ros::spinOnce();
        loop_rate.sleep();
    }
}

/* set and move servos */
void Servo::set_and_move_servos(const double *target_degs)
{
    set_body_angles(target_degs);
    move_servos();
}

void Servo::set_and_move_servos(const unsigned int duration, const double *target_degs)
{
    set_body_angles(target_degs);
    move_servos(duration);
}

void Servo::set_and_move_servos(const unsigned int duration, const double *target_poss, const double *target_degs)
{
    /* calculate inverse kinematics */
    // ROS_INFO("calculate inverse kinematics");
    double left_leg_degs[4];
    double right_leg_degs[4];
    ik(target_poss[0], target_poss[1], target_poss[2], target_poss[3], left_leg_degs);
    ik(target_poss[4], target_poss[5], target_poss[6], target_poss[7], right_leg_degs);

    /* assiging servo angles */
    // ROS_INFO("assiging servo angles");
    double target_degs_tmp[SERVONUM];
    for(int id = 0; id < 4; id++)
    {
        target_degs_tmp[id] = left_leg_degs[id];
        target_degs_tmp[id+4] = -right_leg_degs[id];
    }
    for(int id = 0; id < 6; id++)
    {
        target_degs_tmp[id+8] = target_degs[id];
    }

    /* set and move servos */
    // ROS_INFO("set and move servos");
    set_body_angles(target_degs_tmp);
    move_servos(duration);
    // ROS_INFO("done");
}
