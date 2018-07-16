#ifndef GAI_H_
#define GAI_H_

#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <hexapod_msgs/FeetPositions.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
 
class Gait
{
public:
  Gait(void);
  void gaitCycle( const geometry_msgs::Twist &cmd_vel, hexapod_msgs::FeetPositions *feet );  //摆动腿和支撑腿切换
  double cycle_period_;
  int origin_period_;
  std::vector<int> cycle_leg_number_;
private:
  void cyclePeriod( const geometry_msgs::Pose2D &base, hexapod_msgs::FeetPositions *feet);  //每条摆动腿和支撑腿一个周期内的步幅控制
  geometry_msgs::Pose2D smooth_base_;
  ros::Time current_time_, last_time_;
  int CYCLE_LENGTH;
  int NUMBER_OF_LEGS;
  double LEG_LIFT_HEIGHT;
  bool is_travelling_ ;
  int extra_gait_cycle_;    // Forcing some extra timed cycles
  bool in_cycle_;           // True if the robot is in a gait cycle
  bool stop_cycle_;
  bool stop_cycle_start;
  bool stop_finished;
  bool start_cycle;
  geometry_msgs::Pose2D base;

}; 

#endif





