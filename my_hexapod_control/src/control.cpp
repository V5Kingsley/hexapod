#include "control.h"

Control::Control( void )
{
  ros::param::get( "NUMBER_OF_LEGS", NUMBER_OF_LEGS );
  ros::param::get( "NUMBER_OF_LEG_SEGMENTS", NUMBER_OF_LEG_JOINTS );
  ros::param::get( "MASTER_LOOP_RATE", MASTER_LOOP_RATE );
   ros::param::get( "VELOCITY_DIVISION", VELOCITY_DIVISION );
   ros::param::get( "STICK_FORCE", STICK_FORCE );
   ros::param::get("JOINT_NAME", joint_name_);
    // Topics we are subscribing
    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>( "/cmd_vel", 1, &Control::cmd_velCallback, this );
    
    //发布的话题
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
    
    boost::format roll;
    boost::format pitch1;
    boost::format pitch2;
    boost::format pitch3;
    for ( int leg_index = 0,  j =1;  leg_index < NUMBER_OF_LEGS;  leg_index ++, j ++ )
    {
       roll = boost::format("/hexapod/leg%d_roll_joint_position_controller/command") % j;
       pitch1 = boost::format("/hexapod/leg%d_pitch1_joint_position_controller/command") % j;
       pitch2 = boost::format("/hexapod/leg%d_pitch2_joint_position_controller/command") % j; 
       pitch3 = boost::format("/hexapod/leg%d_pitch3_joint_position_controller/command") % j;
      leg_topic[leg_index] = roll.str();
      leg_topic[leg_index+1] = pitch1.str();
      leg_topic[leg_index+2] = pitch2.str();
      leg_topic[leg_index+3] = pitch3.str();
      leg_roll_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index], 10 );
      leg_pitch1_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index+1], 10 );
      leg_pitch2_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index+2], 10 );
      leg_pitch3_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index+3], 10 );
    }
    
    //吸盘
    boost::format stick;
    for ( int leg_index = 0,  j=1; leg_index < NUMBER_OF_LEGS; leg_index++, j++ )
    {
      stick = boost::format( "leg%d_stick3" ) % j;
      force_to_stick[leg_index] = stick.str();
    }
    
    feet_position = nh_.advertise<hexapod_msgs::FeetPositions>("feet_position", 10);
}

void Control::robotInit()
{
   //关节转动角度
  for ( int leg_index=0; leg_index<NUMBER_OF_LEGS; leg_index++ )
  {
    leg_roll[leg_index].data = 0.0;
    leg_pitch1[leg_index].data = 0.0;
    leg_pitch2[leg_index].data = 0.0;
    leg_pitch3[leg_index].data = 0.0;
  }
       for ( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++  )
       {
	 leg_roll_p[leg_index].publish( leg_roll[leg_index] );
	 leg_pitch1_p[leg_index].publish( leg_pitch1[leg_index] );
	 leg_pitch2_p[leg_index].publish( leg_pitch2[leg_index] );
	 leg_pitch3_p[leg_index].publish( leg_pitch3[leg_index] );
      }
  
}
 

void Control::publishJointStates( const hexapod_msgs::LegsJoints &legs, int &origin_period_, std::vector<int> &cycle_leg_number_, const hexapod_msgs::FeetPositions *feet, sensor_msgs::JointState *joint_state )
{
  for ( int leg_index=0; leg_index<NUMBER_OF_LEGS; leg_index++ )
  {
    leg_roll[leg_index].data = legs.leg[leg_index].coxa;
    leg_pitch1[leg_index].data = legs.leg[leg_index].femur;
    leg_pitch2[leg_index].data = legs.leg[leg_index].tibia;
    leg_pitch3[leg_index].data = legs.leg[leg_index].tarsus;
  }
  
  //发布关节角度话题
  for ( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++  )
  {
    leg_roll_p[leg_index].publish( leg_roll[leg_index] );
    leg_pitch1_p[leg_index].publish( leg_pitch1[leg_index] );
    leg_pitch2_p[leg_index].publish( leg_pitch2[leg_index] );
    leg_pitch3_p[leg_index].publish( leg_pitch3[leg_index] );
  }
  feet_position.publish(*feet);
  
  joint_state->header.stamp = ros::Time::now();
  int i = 0;
  joint_state->name.resize(36);
  joint_state->position.resize(36);  
  for(int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
  {
   joint_state->name[i] = joint_name_[i];
   joint_state->position[i] = legs.leg[leg_index].coxa;
   i++;
   joint_state->name[i] = joint_name_[i];
   joint_state->position[i] = legs.leg[leg_index].femur;
   i++;
   joint_state->name[i] = joint_name_[i];
   joint_state->position[i] = legs.leg[leg_index].tibia;
   i++;
   joint_state->name[i] = joint_name_[i];
   joint_state->position[i] = legs.leg[leg_index].tarsus;
   i++;
   //吸盘
   joint_state->name[i] = joint_name_[i];
   joint_state->position[i] = 0;
   i++;
   //吸盘
   joint_state->name[i] = joint_name_[i];
   joint_state->position[i] = 0;
   i++;
  }
  joint_state_pub_.publish(*joint_state);
  

   //吸盘吸附力控制
  if ( origin_period_ == 1 )
  {

     for ( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++ )
  {

     if (  cycle_leg_number_[leg_index] == 0)
     {
      srv.request.body_name = force_to_stick[leg_index];
      srv.request.reference_frame = force_to_stick[leg_index];
      srv.request.wrench.force.z = 0;
      srv.request.start_time.sec = 0;
      srv.request.duration.sec = 100;
      force_client.call(srv);
     }  
     if (  cycle_leg_number_[leg_index] == 1 )
    {
      srv.request.body_name = force_to_stick[leg_index];
      srv.request.reference_frame = force_to_stick[leg_index];
      srv.request.wrench.force.z = - STICK_FORCE;
      srv.request.start_time.sec = 0;
      srv.request.duration.sec = 100;
      force_client.call(srv);
    }
  }
    ros::Duration(1).sleep(); 
  }
    
}

//订阅发布的速度信息
void Control::cmd_velCallback( const geometry_msgs::TwistConstPtr &cmd_vel_msg )
{
  if (cmd_vel_msg->linear.x>0.1||cmd_vel_msg->linear.x<-0.1)
  {
    ROS_FATAL("The linear.x exceeds the upper limit, set it to max: 0.1.");
    cmd_vel_incoming_.linear.x = (cmd_vel_msg->linear.x > 0 ? 0.1 : -0.1);
  }
  else
  {
    cmd_vel_incoming_.linear.x = cmd_vel_msg->linear.x;
  }
  
    if (cmd_vel_msg->linear.y>0.1|| cmd_vel_msg->linear.y<-0.1)
  {
    ROS_FATAL("The linear.y exceeds the upper limit, set it to max: 0.1.");
    cmd_vel_incoming_.linear.y = (cmd_vel_msg->linear.y > 0 ? 0.1 : -0.1);;
  }
  else
  {
    cmd_vel_incoming_.linear.y = cmd_vel_msg->linear.y;
  }
  
  if (cmd_vel_msg->angular.z>0.2 || cmd_vel_msg->angular.z<-0.2)
  {
    ROS_FATAL("The angular.z exceeds the upper limit, set it to max: 0.2.");
    cmd_vel_incoming_.angular.z = (cmd_vel_msg->angular.z > 0 ? 0.2 : -0.2);;
  }
  else
  {
    cmd_vel_incoming_.angular.z = cmd_vel_msg->angular.z;
  }
}

//速度转化成歩幅
void Control::partitionCmd_vel( geometry_msgs::Twist *cmd_vel )
{
    // Instead of getting delta time we are calculating with a static division
    double dt = VELOCITY_DIVISION;

    double delta_th = cmd_vel_incoming_.angular.z * dt;
    double delta_x = ( cmd_vel_incoming_.linear.x * cos( delta_th ) - cmd_vel_incoming_.linear.y * sin( delta_th ) ) * dt;
    double delta_y = ( cmd_vel_incoming_.linear.x * sin( delta_th ) + cmd_vel_incoming_.linear.y * cos( delta_th ) ) * dt;
    cmd_vel->linear.x = delta_x;
    cmd_vel->linear.y = delta_y;
    cmd_vel->angular.z = delta_th;
}
