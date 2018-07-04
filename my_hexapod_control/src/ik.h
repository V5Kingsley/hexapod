#ifndef IK_H_
#define IK_H_

#include <cmath>
#include <ros/ros.h>
#include <hexapod_msgs/Pose.h>
#include <hexapod_msgs/LegsJoints.h>
#include <hexapod_msgs/FeetPositions.h>

struct Trig
{
  double sine;
  double cosine;
};
 
class IK
{
public:
  IK( void );
  void calculateIK( const hexapod_msgs::FeetPositions &feet, hexapod_msgs::LegsJoints *legs );
private:
  Trig getSinCos( double angle_rad );
  std::vector<double> COXA_TO_CENTER_X, COXA_TO_CENTER_Y;
  std::vector<double> INIT_COXA_ANGLE;
  std::vector<double> INIT_FOOT_POS_X, INIT_FOOT_POS_Y, INIT_FOOT_POS_Z;
  double COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, TARSUS_LENGTH;
  int NUMBER_OF_LEGS; 
};

#endif


