#include "proto2_ros/vr_control.hpp"
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

VrControl::VrControl(ros::NodeHandle n)
{
    sub_head = n.subscribe("/oculus/head_set/pose_stamped", 1, &VrControl::vr_head_callback, this);
    sub_right_hand = n.subscribe("/oculus/right_controller/pose_stamped", 1, &VrControl::vr_right_hand_callback, this);
    sub_left_hand = n.subscribe("/oculus/left_controller/pose_stamped", 1, &VrControl::vr_left_hand_callback, this);
}

void VrControl::vr_head_callback(const geometry_msgs::PoseStamped posestamped)
{
    ROS_INFO("VR head");

    // if disabled, do not update angles
    if(is_vr_control_enabled == false)
        return;

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
    vr_control_degs[14] = yaw;
    vr_control_degs[15] = pitch;
}


void VrControl::vr_right_hand_callback(const geometry_msgs::PoseStamped posestamped)
{
    ROS_INFO("VR right hand");

    // if disabled, do not update angles
    if(is_vr_control_enabled == false)
        return;

    // solve IK

    // convert radians to degrees

    // limit
    
    // set head angles
    vr_control_degs[8] = 0;
    vr_control_degs[9] = 0;
    vr_control_degs[10] = 0;
}

void VrControl::vr_left_hand_callback(const geometry_msgs::PoseStamped posestamped)
{
    ROS_INFO("VR left hand");

    // if disabled, do not update angles
    if(is_vr_control_enabled == false)
        return;

    // set head angles
    vr_control_degs[11] = 0;
    vr_control_degs[12] = 0;
    vr_control_degs[13] = 0;
}

void VrControl::enable_vr_control(void)
{
    is_vr_control_enabled = true;
}

void VrControl::disable_vr_control(void)
{
    is_vr_control_enabled = false;
}