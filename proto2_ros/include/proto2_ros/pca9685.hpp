#ifndef PCA9685_HPP
#define PCA9685_HPP

#include <math.h>

class Servo
{
private:
    /* assign values of src array to dst array */
    inline void copy_array(double *dst, double *src)
    {
        for(int id = 0; id < SERVONUM; id++)
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

    /* set pwm to PCA9685 with I2C interface */
    void set_pwm(const unsigned int id, const unsigned int pwm);

    /* solve inverse kinematics */
    void ik(const double x, const double y, const double z, const double a, double *degs);

    int i2c_fd;

    static const unsigned int STEP_MS;
    static const unsigned int SERVONUM;
    static const double DEGMIN;
    static const double DEGMAX;
    static const double SERVOMIN;
    static const double SERVOMAX;

    static const double ids[];
    static const double ccws[];
    static const double offsets[];

    double current_degs[];
    double end_degs[];

public:
    Servo(void);
    ~Servo(void);

    /* power off all servos */
    void poweroff_servos(void);

    /* set servo angles without moving */
    void set_head_angles(const double end_deg_yaw, const double end_deg_pitch);
    void set_body_angles(const double *end_degs);

    /* move all servos to set angles */
    void move_servos(const unsigned int duration = 1);  // duration in ms

    /* set body angles and move all servos */
    void set_and_move_servos(const double *end_degs);
    void set_and_move_servos(const unsigned int duration, const double *end_degs);
    void set_and_move_servos(const unsigned int duration, const double *end_poss, const double *end_degs);
};

#endif