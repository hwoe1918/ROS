#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h> //cmd_vel
#include <sensor_msgs/Range.h>   //ultrasonic sensor message


//#define frequency_odom_pub 50 //Hz

geometry_msgs::Twist cmd_vel_msg;
std_msgs::Bool flag_AEB;
std_msgs::Bool flag_STOP;
std_msgs::Float32 delta_range;
std_msgs::Float32 old_sonar_range;
nav_msgs::Odometry pos, delta_pos, past_pos;

float x = 0.0, y = 0.0;
const float move_to_x = 4.0;
void UltraSonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  //ROS_INFO("Sonar Seq : [%d]", msg->header.seq);
  //ROS_INFO("Sonar Range : [%f]", msg->range);
  if(msg->range <= 1.8)
  {
	  //ROS_INFO("AEB Activation!!");
	  flag_AEB.data = true;
  }
  else
  {
	  flag_AEB.data = false;
  }
  delta_range.data = msg->range-old_sonar_range.data;
  //ROS_INFO("delta_range : [%f]", delta_range.data);
  old_sonar_range.data = msg->range;
}

void UltraSonarCallback2(const sensor_msgs::Range::ConstPtr& msg)
{
  //ROS_INFO("Sonar2 Seq : [%d]", msg->header.seq);
  //ROS_INFO("Sonar2 Range : [%f]", msg->range);
}

void CarControlCallback(const geometry_msgs::Twist& msg)
{
	//ROS_INFO("Cmd_vel = linear x [%f]", msg.linear.x);
	cmd_vel_msg.linear.x = msg.linear.x;
	//ROS_INFO("Cmd_vel = angular x [%f]", msg.angular.z);
	cmd_vel_msg.angular.z = msg.angular.z;
}

void odomCallback(const nav_msgs::Odometry& msg)
{
	//ROS_INFO("%.2lf %.2lf", msg.pose.pose.position.x, msg.pose.pose.position.y);
	pos.pose.pose.position.x = msg.pose.pose.position.x;
	pos.pose.pose.position.y = msg.pose.pose.position.y;
	
	//초기값
	if(x == 0 && y == 0)
	{
		x = msg.pose.pose.position.x;
		y = msg.pose.pose.position.y;
		cmd_vel_msg.linear.x = 1;
	}
	
	if(move_to_x <= ( pos.pose.pose.position.x - x))
		flag_STOP.data = true;
}

int main(int argc, char **argv)
{
  int count = 0;
  old_sonar_range.data = 0;
  flag_STOP.data = false;
  past_pos.pose.pose.position.x = past_pos.pose.pose.position.y = 0.0;
  delta_pos.pose.pose.position.x = delta_pos.pose.pose.position.y = 0.0;
   
  std::string odom_sub_topic = "/ackermann_steering_controller/odom";
  
  ros::init(argc, argv, "aeb_controller2");

  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("/range", 1000, UltraSonarCallback);
  ros::Subscriber sonar_sub = n.subscribe("/RangeSonar1", 1000, UltraSonarCallback2);
  ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 10, &CarControlCallback);
  ros::Subscriber sub_odom = n.subscribe(odom_sub_topic, 10, &odomCallback);
  
  ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("ackermann_steering_controller/cmd_vel", 10);
  ros::Publisher pub_aeb_activation_flag = n.advertise<std_msgs::Bool>("aeb_activation_flag", 1);
  ros::Publisher pub_delta_range = n.advertise<std_msgs::Float32>("delta_range", 1);
  ros::Publisher pub_delta_pos = n.advertise<nav_msgs::Odometry>(odom_sub_topic, 10);
  
  ros::Rate loop_rate(10); 
  
  while(ros::ok())
  {
	  //m/s
	  if((count%10)==0)
	  {
		 delta_pos.pose.pose.position.x = pos.pose.pose.position.x - past_pos.pose.pose.position.x;
		 delta_pos.pose.pose.position.y = pos.pose.pose.position.y - past_pos.pose.pose.position.y;
		 
		 pub_aeb_activation_flag.publish(flag_AEB);
		 pub_delta_pos.publish(delta_pos);
		 
		 past_pos.pose.pose.position.x = pos.pose.pose.position.x;
		 past_pos.pose.pose.position.y = pos.pose.pose.position.y;
		 //ROS_INFO("%.2lf %.2lf", delta_pos.pose.pose.position.x, delta_pos.pose.pose.position.y);
	  }
	  
	  if(flag_AEB.data == true)
	  {
		 cmd_vel_msg.linear.x = 0;
		 pub_cmd_vel.publish(cmd_vel_msg);
	  }
	  
	  else
	  {
		 pub_cmd_vel.publish(cmd_vel_msg);
	  }
	  
	  if(flag_STOP.data == true)
	  {
		  cmd_vel_msg.linear.x = 0;
		  pub_cmd_vel.publish(cmd_vel_msg);
	  }
	  
	  pub_delta_range.publish(old_sonar_range);
	  
	  loop_rate.sleep();
	  ros::spinOnce();	
	  count++;
  }
  return 0;
}
