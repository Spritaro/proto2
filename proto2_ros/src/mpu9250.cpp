#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include <wiringPiSPI.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

/*
 * this ROS node reads IMU sensor (MPU9050) with SPI interface and 
 * publishes imu/data_raw (sensor_msgs/Imu) message for imu_filter_madgwick
 */

/* SPI communication with MPU9250
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

void readAllSensors(sensor_msgs::Imu &imu)
{
    // bit * 16g(156.8m/s2) / 32768bit
    imu.linear_acceleration.x = static_cast<int16_t>((readByte(0x3b) << 8) | readByte(0x3c)) * 156.8 / 32768.0;
    imu.linear_acceleration.y = static_cast<int16_t>((readByte(0x3d) << 8) | readByte(0x3e)) * 156.8 / 32768.0;
    imu.linear_acceleration.z = static_cast<int16_t>((readByte(0x3f) << 8) | readByte(0x40)) * 156.8 / 32768.0;
    // bit * 2000dps / 32768bit
    imu.angular_velocity.x    = static_cast<int16_t>((readByte(0x43) << 8) | readByte(0x44)) * 2000.0 / 32768.0;
    imu.angular_velocity.y    = static_cast<int16_t>((readByte(0x45) << 8) | readByte(0x46)) * 2000.0 / 32768.0;
    imu.angular_velocity.z    = static_cast<int16_t>((readByte(0x47) << 8) | readByte(0x48)) * 2000.0 / 32768.0;
}

int main(int argc, char **argv)
{
    /* setup ROS node */
    ros::init(argc, argv, "imu_publisher");
    ros::NodeHandle n;
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
    ros::Rate loop_rate(100);   // 100Hz

    /* setup SPI interface
     *     use channel 0
     *     communicate at 1MHz */
    int fd = wiringPiSPISetup (/* channel */ 0, /* speed */ 1000000);
    if(fd == -1)
    {
        // error
        ROS_ERROR("Failed to setup SPI interface");
        return(0);
    }
    int spiMode = 0;
    ioctl(fd, SPI_IOC_WR_MODE, &spiMode);

    /* setup MPU9250 */
    writeByte(0x6A, 0x10);  // SPI mode only
    ROS_INFO("WHO AM I %d", readByte(0x75) );      // read WHO AM I register 0111 0101 should return 0x71
    writeByte(0x1B, 0x18);  // set gyro scale +2000dps, enable DLPF
    writeByte(0x1C, 0x18);  // set accel scale to +/-16G
    writeByte(0x6B, 0x00);  // start device, internal oscillator

    sensor_msgs::Imu imu;
    while(ros::ok())
    {
        readAllSensors(imu);

        imu.header.stamp = ros::Time::now();
        imu.header.frame_id = "map";
        imu_pub.publish(imu);
        // ROS_INFO("acc x %f y %f z %f ang x %f y %f z %f", 
        //     imu.linear_acceleration.x,
        //     imu.linear_acceleration.y,
        //     imu.linear_acceleration.z,
        //     imu.angular_velocity.x,
        //     imu.angular_velocity.y,
        //     imu.angular_velocity.z );

        ros::spinOnce();
        loop_rate.sleep();
    }

    return(0);
}
