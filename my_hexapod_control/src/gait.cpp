#include "gait.h"
static const double PI=3.141592653;

Gait::Gait( void )
{
  ros::param::get( "CYCLE_LENGTH", CYCLE_LENGTH );
  ros::param::get( "LEG_LIFT_HEIGHT", LEG_LIFT_HEIGHT );
  ros::param::get( "LEG_GAIT_ORDER", cycle_leg_number_ );
  ros::param::get( "NUMBER_OF_LEGS", NUMBER_OF_LEGS );
  cycle_period_=0;
  current_time_=ros::Time::now();
  last_time_=ros::Time::now();
  stop_cycle_ = 0;
  stop_cycle_start = 0;
  stop_finished = 0;
}

//每条摆动腿和支撑腿一个周期内的步幅控制
void Gait::cyclePeriod( const geometry_msgs::Pose2D &base, hexapod_msgs::FeetPositions *feet )
{
  double period_distance, period_height;
  if (stop_cycle_start == 1)
  {
    if (cycle_period_ == 0)
    {
      period_distance = 0;
      period_height = 0;
      
    }
    else
    {
      period_distance = -cos( 0.5 * cycle_period_*PI/CYCLE_LENGTH );  //每条腿PI/CYCLE_LENGTH时间的步幅
      period_height = 0.5 * sin( cycle_period_*PI/CYCLE_LENGTH ); //摆动腿PI/CYCLE_LENGTH时间抬起的高度
    }
  }
  
    if (stop_cycle_start == 0 && start_cycle == 0)
  {
    period_distance = -cos( cycle_period_*PI/CYCLE_LENGTH );  //每条腿PI/CYCLE_LENGTH时间的步幅
    period_height = sin( cycle_period_*PI/CYCLE_LENGTH ); //摆动腿PI/CYCLE_LENGTH时间抬起的高度
  }
  
  if(start_cycle == 1)
  {
    period_distance = sin( 0.5 * cycle_period_*PI/CYCLE_LENGTH );  //每条腿PI/CYCLE_LENGTH时间的步幅
    period_height = 0.5 * sin( cycle_period_*PI/CYCLE_LENGTH ); //摆动腿PI/CYCLE_LENGTH时间抬起的高度
  }



  //摆动腿和支撑腿歩幅控制
  for( int leg_index=0; leg_index<NUMBER_OF_LEGS; leg_index++ )
  {
    //摆动腿
    if( cycle_leg_number_[leg_index]==0 )
    {
      feet->foot[leg_index].position.x = base.x * period_distance;
      feet->foot[leg_index].position.y = base.y * period_distance;
      feet->foot[leg_index].position.z = LEG_LIFT_HEIGHT * period_height;
      feet->foot[leg_index].orientation.yaw = base.theta * period_distance;
    }
    //支撑腿
    if( cycle_leg_number_[leg_index]==1 )
    {
      feet->foot[leg_index].position.x = -base.x * period_distance;
      feet->foot[leg_index].position.y = -base.y * period_distance;
      feet->foot[leg_index].position.z = 0.0;
      feet->foot[leg_index].orientation.yaw = -base.theta * period_distance;
    }
  }
  
  if ( stop_cycle_ == 1 && stop_cycle_start == 1 && cycle_period_ == 0)
  {
    stop_cycle_ = 0;
    stop_cycle_start = 0;
    stop_finished = 1;
  }
  
  if(start_cycle == 1 && cycle_period_ == (CYCLE_LENGTH - 1))
  {
    start_cycle = 0;
  }
}
 
//摆动腿和支撑腿切换
void Gait::gaitCycle( const geometry_msgs::Twist &cmd_vel, hexapod_msgs::FeetPositions *feet )
{
  if( (base.x - cmd_vel.linear.x) != 0 || (base.y - cmd_vel.linear.y) != 0 || (base.theta - cmd_vel.angular.z) != 0)
  {
    if (cmd_vel.linear.x == 0 && cmd_vel.linear.y == 0 && cmd_vel.angular.z == 0 && stop_cycle_start == 0 && stop_finished == 0)
  {
    stop_cycle_ = 1;
    if(cycle_period_ == 1)
    {
      stop_cycle_start = 1;
    }
  }
  
  if (stop_cycle_ == 0 )
  {
    base.x = cmd_vel.linear.x ; 
    base.y = cmd_vel.linear.y ;
    base.theta = cmd_vel.angular.z ;
    stop_finished = 0;
  }
  
  if(smooth_base_.x == 0 && smooth_base_.y == 0 && smooth_base_.theta == 0)
  {
    start_cycle = 1;
  }
  
  }

  if (smooth_base_.x == 0 && smooth_base_.y == 0 && smooth_base_.theta == 0)
  {
    smooth_base_.x = base.x;
    smooth_base_.y = base.y;
    smooth_base_.theta = base.theta;
  }
  else
  {
// Low pass filter on the values to avoid jerky movements due to rapid value changes
    smooth_base_.x = base.x * 0.05 + ( smooth_base_.x * ( 1.0 - 0.05 ) );
    smooth_base_.y = base.y * 0.05 + ( smooth_base_.y * ( 1.0 - 0.05 ) );
    smooth_base_.theta = base.theta * 0.05 + ( smooth_base_.theta * ( 1.0 - 0.05 ) );
  }
    
    if (base.x == 0 && base.y == 0 && base.theta == 0)
    {
      smooth_base_.x = 0;
      smooth_base_.y = 0;
      smooth_base_.theta = 0;
    }
    
    // Check to see if we are actually travelling
    if( ( std::abs( smooth_base_.y ) > 0.001 ) || // 1 mm
        ( std::abs( smooth_base_.x ) > 0.001 ) || // 1 mm
        ( std::abs( smooth_base_.theta ) > 0.00436332313 ) ) // 0.25 degree
    {
        is_travelling_ = true;
    }
    else
    {
        is_travelling_ = false;  
    }
    if ( is_travelling_ == true )
    {
      //给下一个period/CYCLE_LENGTH足端歩幅
      cyclePeriod( smooth_base_, feet);
      cycle_period_++;
    }
    
    //一个周期结束后更换摆动腿组和支撑腿组
    if( cycle_period_==CYCLE_LENGTH )
    {
      cycle_period_=0;
      std::reverse( cycle_leg_number_.begin(), cycle_leg_number_.end() );
    } 
    
}



