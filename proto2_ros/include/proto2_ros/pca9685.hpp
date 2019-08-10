#ifndef PCA9685_HPP
#define PCA9685_HPP

#include <ros/ros.h>
#include <math.h>

class Servo
{
private:
    /* assign values of src array to dst array */
    inline void copy_array(double *dst, const double *src, const unsigned int begin_id, const unsigned int nb_val)
    {
        for(int id = begin_id; id < begin_id + nb_val; id++)
            dst[id] = src[id];
    }

    /* convert degrees to radians */
    inline double radians(double deg)
    {
        return ( deg * M_PI / 180.0 );
    }

    /* convert degrees to radians */
    inline double degrees(double rad)
    {
        return ( rad * 180.0 / M_PI );
    }

    /* convert angles in degrees to pulses width in 2048 resolution */
    inline unsigned int deg2width(const double deg)
    {
        return ( static_cast<unsigned int>((SERVOMAX - SERVOMIN) / (DEGMAX-DEGMIN) * (deg-DEGMIN) + SERVOMIN) );
    }

    void setup_pca9685(void);
    void set_pwm_freq(const double freq_hz);
    void set_pwm(const unsigned int channel, const unsigned int pwm);

    /* solve inverse kinematics */
    void ik(const double x, const double y, const double z, const double a, double *degs);

    int i2c_fd;

    static const unsigned int STEP_MS = 10;
    static const unsigned int SERVONUM = 16;
    static const double DEGMIN = -90.0;
    static const double DEGMAX = 90.0;
    static const double SERVOMIN = 139.8;
    static const double SERVOMAX = 565.5;

    static const double channels[];
    static const double ccws[];
    static const double offsets[];

    static double current_degs[];
    static double end_degs[];

    ros::Rate loop_rate;

public:
    Servo(void);
    ~Servo(void);

    /* power off all servos */
    void poweroff_servos(void);

    /* set servo angles without moving */
    void set_body_angles(const double *target_degs);

    /* move all servos to set angles */
    void move_servos(const unsigned int duration = 1);  // duration in ms

    /* set body angles and move all servos */
    void set_and_move_servos(const double *target_degs);
    void set_and_move_servos(const unsigned int duration, const double *target_degs);
    void set_and_move_servos(const unsigned int duration, const double *target_poss, const double *target_degs);

    static double vr_control_degs[];
    static bool is_vr_control_enabled;
    static const bool is_vr_control_applicable[];
};

#endif