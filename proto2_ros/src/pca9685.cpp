#include <proto2_ros/pca9685.hpp>
#include <wiringPiI2C.h>
#include <math.h>
#include <unistd.h>
#include <ros/ros.h>

/*
 * this module controls servo motors from PCA9685 with I2C interface
 */

/* initialize parameters */
const unsigned int Servo::STEP_MS = 10;
const unsigned int Servo::SERVONUM = 16;
const double Servo::DEGMIN = -90.0;
const double Servo::DEGMAX = 90.0;
const double Servo::SERVOMIN = 103.0;
const double Servo::SERVOMAX = 490.0;

const double Servo::ids[] = {0,1,2,3, 15,14,13,12, 4,5,6, 11,10,9, 8,7};
const double Servo::ccws[] = {1.,1.,-1.,1., -1.,1.,-1.,-1, -1.,-1.,1., 1.,1.,-1., 1.,-1.};
const double Servo::offsets[] = {-15.,-42.,70.,-40., 78.,60.,-18.,77., 23.,5.,-4., 15.,58.,52., 26.,36.};


Servo::Servo(void)
{
    /* create I2C interface */
    i2c_fd = wiringPiI2CSetup(0x48);
    if(Servo::i2c_fd == -1)
        ROS_ERROR("Failed to setup I2C interface");
    else
        ROS_INFO("Setup I2C interface");

    /* setup PCA9685 */
    // TODO: set PWM frequency
    
    /* initialize variables */
    double tmp_array[] = {0.,0.,0.,0., 0.,0.,0.,0., 0.,0.,0.,0., 0.,0.,0.,0.};
    copy_array(current_degs, tmp_array);
    copy_array(end_degs, tmp_array);
}

Servo::~Servo(void)
{
    /* do nothing */
}

void Servo::set_pwm(const unsigned int id, const unsigned int pwm)
{
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
}

/* */
void Servo::set_head_angles(const double end_deg_yaw, const double end_deg_pitch)
{
}

void Servo::set_body_angles(const double *end_degs)
{
}

/* */
void Servo::move_servos(const unsigned int duration)
{
    // 1ms -> STEP_MS ms
    unsigned int nb_step = duration / STEP_MS;
    if(nb_step < 1) nb_step = 1;

    double start_degs[SERVONUM];
    copy_array(start_degs, current_degs);
    for(int t = 1; t < nb_step+1; t++)
    {
        // update servo angles
        for(int id = 0; id < SERVONUM; id++)
            current_degs[id] = ((end_degs[id]-start_degs[id]) * t/nb_step + start_degs[id]);

        // copy to temporary array and correct certain angles
        double degs_tmp[SERVONUM];
        for(int id = 0; id < SERVONUM; id++)
            degs_tmp[id] = current_degs[id];
        degs_tmp[0] *= 2.0;
        degs_tmp[3] *= 2.0;
        degs_tmp[4] *= 2.0;
        degs_tmp[7] *= 2.0;

        // move servos
        for(int id = 0; id < SERVONUM; id++)
            set_pwm( ids[id], deg2width(degs_tmp[id]*ccws[id]+offsets[id]) );

        sleep(STEP_MS / 1000.0);
    }
}

/* */
void Servo::set_and_move_servos(const double *end_degs)
{
    set_body_angles(end_degs);
    move_servos();
}

void Servo::set_and_move_servos(const unsigned int duration, const double *end_degs)
{
    set_body_angles(end_degs);
    move_servos(duration);
}

void Servo::set_and_move_servos(const unsigned int duration, const double *end_poss, const double *end_degs)
{
    double degs[4];
    ik(end_poss[0], end_poss[1], end_poss[2], end_poss[3], degs);
}
