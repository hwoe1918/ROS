#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h> //cmd_vel
#include <sensor_msgs/Range.h>   //ultrasonic sensor message

std_msgs::Bool flag_AEB;

void UltraSonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	ROS_INFO("Sonar Seq: [%d]", msg->header.seq);
	ROS_INFO("Sonar Range: [%f]", msg->range);
	
	if(msg->range <= 1.0)
	{
		ROS_INFO("AEB_Activeation!!");
		flag_AEB.data = true;
	}
	else
	{
		flag_AEB.data = false;
	}
}
void UltraSonarCallback2(const std_msgs::Int32::ConstPtr& msg)
{
	ROS_INFO("Sonar Seq: [%d]", msg->data);
}
void UltraSonarCallback3(const std_msgs::Float32::ConstPtr& msg)
{
	ROS_INFO("Sonar Range: [%f]", msg->data);
	
	if(msg->data <= 1.0)
	{
		ROS_INFO("AEB_Activeation!!");
		flag_AEB.data = true;
	}
	else
	{
		flag_AEB.data = false;
	}
}

int main(int argc, char **argv)
{
  int count = 0;
  ros::init(argc, argv, "aeb_controller");

  ros::NodeHandle n;

  ros::Rate loop_rate(2); 
  
  ros::Subscriber sub1 = n.subscribe("range", 1000, UltraSonarCallback);
  ros::Subscriber sub2 = n.subscribe("seq", 1000, UltraSonarCallback2);
  ros::Subscriber sub3 = n.subscribe("range2", 1000, UltraSonarCallback3);
  
  while(ros::ok())
  {
	loop_rate.sleep();
	ros::spinOnce();
	++count;	
  }
  
  
  return 0;
}
