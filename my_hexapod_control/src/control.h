#ifndef CONTROL_H
#define CONTROL_H

#include <cmath>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <hexapod_msgs/Pose.h>
#include <hexapod_msgs/RPY.h>
#include <hexapod_msgs/LegsJoints.h>
#include <hexapod_msgs/FeetPositions.h>
#include <hexapod_msgs/Sounds.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <string>
#include <boost/format.hpp>

class Control
{
public:
  Control( void );
  void partitionCmd_vel( geometry_msgs::Twist *cmd_vel );  //把速度离散化？
  void publishJointStates( const hexapod_msgs::LegsJoints &legs, int &ori_period_, std::vector<int> &cycle_leg_number_, const hexapod_msgs::FeetPositions *feet); 
  void robotInit();
  int MASTER_LOOP_RATE;  // Master loop rate
  hexapod_msgs::Pose body_;    // Body link rotation,没有用到
  hexapod_msgs::LegsJoints legs_; //各个关节的角度信息
  hexapod_msgs::FeetPositions feet_; //足端轨迹
  geometry_msgs::Twist cmd_vel_;
private:
  double VELOCITY_DIVISION;
  int NUMBER_OF_LEGS;        // Number of legs
  int NUMBER_OF_LEG_JOINTS;  // Number of leg segments
  int STICK_FORCE; //吸盘吸附力大小
  geometry_msgs::Twist cmd_vel_incoming_;
  int setup_;
  
  // 订阅速度信息
  ros::Subscriber cmd_vel_sub_;
  void cmd_velCallback( const geometry_msgs::TwistConstPtr &cmd_vel_msg );
  
    // Node Handle
  ros::NodeHandle nh_;
   
  // 发布每个关节的角度话题
  std::string leg_topic[24]; 
  ros::Publisher leg_roll_p[6];
  ros::Publisher leg_pitch1_p[6];
  ros::Publisher leg_pitch2_p[6];
  ros::Publisher leg_pitch3_p[6];
  std_msgs::Float64 leg_roll[6];
  std_msgs::Float64 leg_pitch1[6];
  std_msgs::Float64 leg_pitch2[6];
  std_msgs::Float64 leg_pitch3[6];
  
  //吸盘吸附力Client
  std::string force_to_stick[6];
  ros::ServiceClient force_client = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  gazebo_msgs::ApplyBodyWrench srv;
  
  ros::Publisher feet_position;
  
};

#endif


