#include <ros/ros.h>
#include "gait.h"
#include "ik.h"
#include "control.h"

int main (int argc, char **argv)
{
  ros::init( argc, argv, "my_hexapod_controller" );
  Control control;
  Gait gait;
  IK ik;

  ros::AsyncSpinner spinner( 2); // Using 2 threads
  spinner.start();
  ros::Rate loop_rate( control.MASTER_LOOP_RATE );
   int setup=1;
   //初始化机器人位姿
  while( ros::ok() )
  {
    if(setup == 0)
    {
      control.partitionCmd_vel( &control.cmd_vel_ );
    gait.gaitCycle( control.cmd_vel_, &control.feet_); //周期步态
    ik.calculateIK( control.feet_, &control.legs_ ); //逆运动学求解关节角度
    control.publishJointStates( control.legs_, gait.origin_period_, gait.cycle_leg_number_, &control.feet_); //发布关节角度话题
    }
    else
    {
      ROS_INFO("hexapod is standing up.");
      control.robotInit();
      setup = 0;
    }
    loop_rate.sleep(); 
  }
  
   return 0; 
  
}