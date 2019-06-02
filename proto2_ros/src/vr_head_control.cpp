#include "proto2_ros/vr_head_control.hpp"
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

VrHeadControl::VrHeadControl(ros::NodeHandle n) :
        is_vr_head_control_enabled(true)
{
    sub = n.subscribe("oculus/pose_stamped", 1, &VrHeadControl::vr_head_callback, this);
}

void VrHeadControl::vr_head_callback(const geometry_msgs::PoseStamped posestamped)
{
    // if disabled, set to default head angles
    if(is_vr_head_control_enabled == false)
    {
        set_head_angles(0.0, 0.0);
        return;
    }

    // convert  geometry_msgs::Quaternion to tf::Quaterion
    tf2::Quaternion quaternion;
    tf2::fromMsg(posestamped.pose.orientation, quaternion);

    // convert tf::Quaternion to euler angles
    double roll_tmp, pitch_tmp, yaw_tmp;
    double roll, pitch, yaw;
    // tf2::Matrix3x3(quaternion).getRPY(roll_tmp, pitch_tmp, yaw_tmp);
    tf2::Matrix3x3(quaternion).getEulerYPR(yaw_tmp, pitch_tmp, roll_tmp);

    // convert radians to degrees
    roll  =  pitch_tmp * 180.0 / M_PI;
    pitch = -roll_tmp  * 180.0 / M_PI;
    yaw   =  yaw_tmp   * 180.0 / M_PI;

    // limit
    if(yaw < angle_yaw_min) yaw = angle_yaw_min;
    if(yaw > angle_yaw_max) yaw = angle_yaw_max;
    if(pitch < angle_pitch_min) pitch = angle_pitch_min;
    if(pitch > angle_pitch_max) pitch = angle_pitch_max;
    
    // set head angles
    ROS_INFO("VR head rpy %d %d %d", static_cast<int>(roll), static_cast<int>(pitch), static_cast<int>(yaw));
    set_head_angles(yaw, pitch);
}

void VrHeadControl::enable_vr_head_control(void)
{
    is_vr_head_control_enabled = true;
}

void VrHeadControl::disable_vr_head_control(void)
{
    is_vr_head_control_enabled = false;
    set_head_angles(0.0, 0.0);
}