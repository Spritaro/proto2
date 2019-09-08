#include "proto2_base/vr_control.hpp"
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

VrControl::VrControl(ros::NodeHandle n)
{
    sub_head       = n.subscribe("/oculus/head_set/pose_stamped",         1, &VrControl::vr_head_callback,       this);
    sub_right_hand = n.subscribe("/oculus/right_controller/pose_stamped", 1, &VrControl::vr_right_hand_callback, this);
    sub_left_hand  = n.subscribe("/oculus/left_controller/pose_stamped",  1, &VrControl::vr_left_hand_callback,  this);
    sub_joy        = n.subscribe("/oculus/joy",                           1, &VrControl::vr_joy_callback,        this);

    // create empty joy message
    for(uint8_t i=0; i<VR_BUTTON_NUM; i++)
        vr_joy.buttons.push_back(0);
    for(uint8_t i=0; i<VR_AXIS_NUM; i++)
        vr_joy.axes.push_back(0.0);
}

void VrControl::ik_for_left_hand(double x, double y, double z, double *degs)
{
    // mをmmに変換
    x *= 1000.0;
    y *= 1000.0;
    z *= 1000.0;

    // コントローラの原点のオフセット
    double oculus_offset_x =    0.0;
    double oculus_offset_y =  200.0;
    double oculus_offset_z = -200.0;
    x -= oculus_offset_x;
    y -= oculus_offset_y;
    z -= oculus_offset_z;

    // 人間サイズの座標をロボット用に変換
    double oculus_arm_length = 600;
    double robot_arm_length = 180;
    x *= robot_arm_length / oculus_arm_length;
    y *= robot_arm_length / oculus_arm_length;
    z *= robot_arm_length / oculus_arm_length;

    double l1y =   0.0;
    double l1z =  15.0;
    double l2  =  80.0;
    double l3  = 100.0;

    // shoulder pitch
    double shoulder_pitch_ang = atan2(x, -z);

    // shoulder roll
    double l = sqrt(x*x + y*y + z*z);
    double z_ = -sqrt(x*x + z*z);
    double shoulder_roll_tmp_1 = atan2(y, -z_);
    double shoulder_roll_tmp_2 = acos( (l2*l2 + l*l - l3*l3) / (2*l2*l) );
    double shoulder_roll_ang = shoulder_roll_tmp_1 + shoulder_roll_tmp_2;

    // elbow
    double elbow_ang = acos( (l2*l2 + l3*l3 - l*l) / (2*l2*l3) );

    // radians to degrees
    degs[0] = degrees(shoulder_pitch_ang);
    degs[1] = degrees(shoulder_roll_ang);
    degs[2] = degrees(elbow_ang);
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
    roll  =  roll_tmp  * 180.0 / M_PI;
    pitch =  pitch_tmp * 180.0 / M_PI;
    yaw   =  yaw_tmp   * 180.0 / M_PI;

    // limit
    if(yaw < angle_yaw_min) yaw = angle_yaw_min;
    if(yaw > angle_yaw_max) yaw = angle_yaw_max;
    if(pitch < angle_pitch_min) pitch = angle_pitch_min;
    if(pitch > angle_pitch_max) pitch = angle_pitch_max;
    
    // set head angles
    //ROS_INFO("VR head rpy %d %d %d", static_cast<int>(roll), static_cast<int>(pitch), static_cast<int>(yaw));
    vr_control_degs[14] = yaw;
    vr_control_degs[15] = pitch;
}

void VrControl::vr_right_hand_callback(const geometry_msgs::PoseStamped posestamped)
{
    ROS_INFO("VR right hand xyz %d %d %d", static_cast<int>(posestamped.pose.position.x*1000),
                                          static_cast<int>(posestamped.pose.position.y*1000),
                                          static_cast<int>(posestamped.pose.position.z*1000));

    // if disabled, do not update angles
    if(is_vr_control_enabled == false)
        return;
    
    // solve IK (use ik_for_left_hand function)
    double right_arm_degs[3];
    ik_for_left_hand(posestamped.pose.position.x,
                     -posestamped.pose.position.y,
                     posestamped.pose.position.z, right_arm_degs);
    right_arm_degs[1] -= 90;
    right_arm_degs[2] -= 90;

    // limit
    if(right_arm_degs[0] < -90)
        right_arm_degs[0]= -90;
    else if(right_arm_degs[0] > 90)
        right_arm_degs[0]= 90;

    if(right_arm_degs[1] < -90)
        right_arm_degs[1]= -90;
    else if(right_arm_degs[0] > 90)
        right_arm_degs[1]= 90;

    if(right_arm_degs[2] < -30)
        right_arm_degs[2]= -30;
    else if(right_arm_degs[2] > 90)
        right_arm_degs[2]= 90;
    
    // set head angles
    vr_control_degs[11] = right_arm_degs[0];
    vr_control_degs[12] = right_arm_degs[1];
    vr_control_degs[13] = right_arm_degs[2];
}

void VrControl::vr_left_hand_callback(const geometry_msgs::PoseStamped posestamped)
{
    ROS_INFO("VR left hand xyz %d %d %d", static_cast<int>(posestamped.pose.position.x*1000),
                                          static_cast<int>(posestamped.pose.position.y*1000),
                                          static_cast<int>(posestamped.pose.position.z*1000));

    // if disabled, do not update angles
    if(is_vr_control_enabled == false)
        return;
    
    // solve IK
    double left_arm_degs[3];
    ik_for_left_hand(posestamped.pose.position.x,
                     posestamped.pose.position.y,
                     posestamped.pose.position.z, left_arm_degs);
    left_arm_degs[1] -= 90;
    left_arm_degs[2] -= 90;

    // limit
    if(left_arm_degs[0] < -90)
        left_arm_degs[0]= -90;
    else if(left_arm_degs[0] > 90)
        left_arm_degs[0]= 90;

    if(left_arm_degs[1] < -90)
        left_arm_degs[1]= -90;
    else if(left_arm_degs[0] > 90)
        left_arm_degs[1]= 90;

    if(left_arm_degs[2] < -30)
        left_arm_degs[2]= -30;
    else if(left_arm_degs[2] > 90)
        left_arm_degs[2]= 90;

    // set head angles
    vr_control_degs[8] = left_arm_degs[0];
    vr_control_degs[9] = left_arm_degs[1];
    vr_control_degs[10] = left_arm_degs[2];
}

/* update gamepad input */
void VrControl::vr_joy_callback(const sensor_msgs::Joy joy_tmp)
{
    // ROS_INFO("%d %d %d %d", joy_tmp.buttons[0], joy_tmp.buttons[1], joy_tmp.buttons[2], joy_tmp.buttons[3]);
    vr_joy = joy_tmp;
}

void VrControl::enable_vr_control(void)
{
    is_vr_control_enabled = true;
}

void VrControl::disable_vr_control(void)
{
    is_vr_control_enabled = false;
}