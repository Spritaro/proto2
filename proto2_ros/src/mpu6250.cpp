#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include <WiringPiSPI.h>

/*
 * this ROS node reads IMU sensor (MPU9050) with SPI interface and 
 * publishes imu/data_raw (sensor_msgs/Imu) message for imu_filter_madgwick
 */

/* SPI communication with MPU6250
 *     first byte contains SPI address
 *     first bit indicates Read (1) or Write (0) operation
 * refer to 
 *     MPU-9250 Product Specification
 *     MPU-9250 Register Map and Descriptions */
int readByte(const int reg)
{
    unsigned char buffer[2];
    buffer[0] = 0x80 | reg; // fill read bit
    buffer[1] = 0x00;
    wiringPiSPIDataRW (/*channel*/ 0, /*data*/ buffer, /*len*/ 2);
    return buffer[1];
}

void writeByte(const int reg, const int val)
{
    unsigned char buffer[2];
    buffer[0] = 0x7F & reg; // clear write bit
    buffer[1] = val;
    wiringPiSPIDataRW (/*channel*/ 0, /*data*/ buffer, /*len*/ 2);
}

void readAllSensors(sensor_msg::Imu &imu)
{
    unsigned char buffer[15];
    buffer[0] = 0x10 | 0x3B;
    wiringPiSPIDataRW (/*channel*/ 0, /*data*/ buffer, /*len*/ 15);
    imu.linear_acceleration.x = (buffer[1]  << 8) & buffer[2];
    imu.linear_acceleration.y = (buffer[3]  << 8) & buffer[4];
    imu.linear_acceleration.z = (buffer[5]  << 8) & buffer[6];
    imu.angular_velocity.x    = (buffer[9]  << 8) & buffer[10];
    imu.angular_velocity.x    = (buffer[11] << 8) & buffer[12];
    imu.angular_velocity.x    = (buffer[13] << 8) & buffer[14];
}

int main(int argc, char **argv)
{
    /* setup ROS node */
    ros::init(argc, argv, "imu_publisher");
    ros::NodeHandle n;
    ros::Publisher imu_pub = n.advertise<sensor_msg::Imu>("imu_publisher", 1);
    ros::Rate loop_rate(100);   // 100Hz

    /* setup SPI interface
     *     use channel 0
     *     communicate at 1MHz */
    if(wiringPiSPISetup (/* channel */ 0, /* speed */ 1000000); == -1)
    {
        // error
        ROS_ERROR("Failed to setup SPI interface");
        return(0);
    }

    /* setup MPU6250 */
    ROS_INFO("WHO AM I %d", readByte(0x075) );      // read WHO AM I register
    writeByte(0x6B, 0x81);  // reset device
    writeByte(0x1B, 0x18);  // set gyro scale +2000dps, enable DLPF
    writeByte(0x1C, 0x18);  // set accel scale to +/-16G

    while(ros::ok())
    {
        sensor_msg::Imu imu;

        readAllSensors(imu);

        imu_pub.publish(imu);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return(0);
}