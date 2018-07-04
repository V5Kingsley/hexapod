#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include<stdlib.h>
#include <time.h>

int main(int argc, char**argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle n;
  ros::Publisher leg1_roll_p=n.advertise<std_msgs::Float64>("/hexapod/leg1_roll_joint_position_controller/command",1);
  ros::Publisher leg1_pitch1_p=n.advertise<std_msgs::Float64>("/hexapod/leg1_pitch1_joint_position_controller/command",1);
  ros::Publisher leg1_pitch2_p=n.advertise<std_msgs::Float64>("/hexapod/leg1_pitch2_joint_position_controller/command",1);
  
  ros::Publisher leg2_roll_p=n.advertise<std_msgs::Float64>("/hexapod/leg2_roll_joint_position_controller/command",1);
  ros::Publisher leg2_pitch1_p=n.advertise<std_msgs::Float64>("/hexapod/leg2_pitch1_joint_position_controller/command",1);
  ros::Publisher leg2_pitch2_p=n.advertise<std_msgs::Float64>("/hexapod/leg2_pitch2_joint_position_controller/command",1);
  
  ros::Publisher leg3_roll_p=n.advertise<std_msgs::Float64>("/hexapod/leg3_roll_joint_position_controller/command",1);
  ros::Publisher leg3_pitch1_p=n.advertise<std_msgs::Float64>("/hexapod/leg3_pitch1_joint_position_controller/command",1);
  ros::Publisher leg3_pitch2_p=n.advertise<std_msgs::Float64>("/hexapod/leg3_pitch2_joint_position_controller/command",1);
  
  ros::Publisher leg4_roll_p=n.advertise<std_msgs::Float64>("/hexapod/leg4_roll_joint_position_controller/command",1);
  ros::Publisher leg4_pitch1_p=n.advertise<std_msgs::Float64>("/hexapod/leg4_pitch1_joint_position_controller/command",1);
  ros::Publisher leg4_pitch2_p=n.advertise<std_msgs::Float64>("/hexapod/leg4_pitch2_joint_position_controller/command",1);
  
  ros::Publisher leg5_roll_p=n.advertise<std_msgs::Float64>("/hexapod/leg5_roll_joint_position_controller/command",1);
  ros::Publisher leg5_pitch1_p=n.advertise<std_msgs::Float64>("/hexapod/leg5_pitch1_joint_position_controller/command",1);
  ros::Publisher leg5_pitch2_p=n.advertise<std_msgs::Float64>("/hexapod/leg5_pitch2_joint_position_controller/command",1);
  
  ros::Publisher leg6_roll_p=n.advertise<std_msgs::Float64>("/hexapod/leg6_roll_joint_position_controller/command",1);
  ros::Publisher leg6_pitch1_p=n.advertise<std_msgs::Float64>("/hexapod/leg6_pitch1_joint_position_controller/command",1);
  ros::Publisher leg6_pitch2_p=n.advertise<std_msgs::Float64>("/hexapod/leg6_pitch2_joint_position_controller/command",1);
 
  ros::Duration wait(0.1);
  
  std_msgs::Float64 leg1_roll;
  std_msgs::Float64 leg1_pitch1;
  std_msgs::Float64 leg1_pitch2;
  
  std_msgs::Float64 leg2_roll;
  std_msgs::Float64 leg2_pitch1;
  std_msgs::Float64 leg2_pitch2;
  
  std_msgs::Float64 leg3_roll;
  std_msgs::Float64 leg3_pitch1;
  std_msgs::Float64 leg3_pitch2;
  
  std_msgs::Float64 leg4_roll;
  std_msgs::Float64 leg4_pitch1;
  std_msgs::Float64 leg4_pitch2;
  
  std_msgs::Float64 leg5_roll;
  std_msgs::Float64 leg5_pitch1;
  std_msgs::Float64 leg5_pitch2;
  
  std_msgs::Float64 leg6_roll;
  std_msgs::Float64 leg6_pitch1;
  std_msgs::Float64 leg6_pitch2;
  

  float ran;
  leg1_roll.data=0;
  while(ros::ok())
  {
    srand((unsigned)time(NULL));
    ran=-0.7+rand()/(double)(RAND_MAX/1.4);
    leg1_roll.data+=ran;
    leg1_roll_p.publish(leg1_roll);
    srand((unsigned)time(NULL));
    ran=0+rand()/(double)(RAND_MAX/1.4);
    leg1_pitch1.data+=ran;
    leg1_pitch1_p.publish(leg1_pitch1);
    srand((unsigned)time(NULL));
    ran=-0.7+rand()/(double)(RAND_MAX/1.4);
    leg1_pitch2.data+=ran;
    leg1_pitch2_p.publish(leg1_pitch2);
    wait.sleep();
    
    srand((unsigned)time(NULL));
    ran=-0.7+rand()/(double)(RAND_MAX/1.4);
    leg2_roll.data+=ran;
    leg2_roll_p.publish(leg2_roll);
    srand((unsigned)time(NULL));
    ran=0+rand()/(double)(RAND_MAX/1.4);
    leg2_pitch1.data+=ran;
    leg2_pitch1_p.publish(leg2_pitch1);
    srand((unsigned)time(NULL));
    ran=-0.7+rand()/(double)(RAND_MAX/1.4);
    leg2_pitch2.data+=ran;
    leg2_pitch2_p.publish(leg2_pitch2);
    wait.sleep();
    
    srand((unsigned)time(NULL));
    ran=-0.7+rand()/(double)(RAND_MAX/1.4);
    leg3_roll.data+=ran;
    leg3_roll_p.publish(leg3_roll);
    srand((unsigned)time(NULL));
    ran=0+rand()/(double)(RAND_MAX/1.4);
    leg3_pitch1.data+=ran;
    leg3_pitch1_p.publish(leg3_pitch1);
    srand((unsigned)time(NULL));
    ran=-0.7+rand()/(double)(RAND_MAX/1.4);
    leg3_pitch2.data+=ran;
    leg3_pitch2_p.publish(leg3_pitch2);
    wait.sleep();
    
    srand((unsigned)time(NULL));
    ran=-0.7+rand()/(double)(RAND_MAX/1.4);
    leg4_roll.data+=ran;
    leg4_roll_p.publish(leg4_roll);
    srand((unsigned)time(NULL));
    ran=0+rand()/(double)(RAND_MAX/1.4);
    leg4_pitch1.data+=ran;
    leg4_pitch1_p.publish(leg4_pitch1);
    srand((unsigned)time(NULL));
    ran=-0.7+rand()/(double)(RAND_MAX/1.4);
    leg4_pitch2.data+=ran;
    leg4_pitch2_p.publish(leg4_pitch2);
    wait.sleep();
    
    srand((unsigned)time(NULL));
    ran=-0.7+rand()/(double)(RAND_MAX/1.4);
    leg5_roll.data+=ran;
    leg5_roll_p.publish(leg5_roll);
    srand((unsigned)time(NULL));
    ran=0+rand()/(double)(RAND_MAX/1.4);
    leg5_pitch1.data+=ran;
    leg5_pitch1_p.publish(leg5_pitch1);
    srand((unsigned)time(NULL));
    ran=-0.7+rand()/(double)(RAND_MAX/1.4);
    leg5_pitch2.data+=ran;
    leg5_pitch2_p.publish(leg5_pitch2);
     wait.sleep();
     
    srand((unsigned)time(NULL));
    ran=-0.7+rand()/(double)(RAND_MAX/1.4);
    leg6_roll.data+=ran;
    leg6_roll_p.publish(leg6_roll);
    srand((unsigned)time(NULL));
    ran=0+rand()/(double)(RAND_MAX/1.4);
    leg6_pitch1.data+=ran;
    leg6_pitch1_p.publish(leg6_pitch1);
    srand((unsigned)time(NULL));
    ran=-0.7+rand()/(double)(RAND_MAX/1.4);
    leg6_pitch2.data+=ran;
    leg6_pitch2_p.publish(leg6_pitch2);
    wait.sleep();
  }
  
}
