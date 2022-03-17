#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h> //cmd_vel
#include <sensor_msgs/Range.h>   //ultrasonic sensor message


void UltraSonarCallback2(const std_msgs::Int32::ConstPtr& msg)
{
	ROS_INFO("Sonar Seq: [%d]", msg->data);
}

void UltraSonarCallback3(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data)
	{
		ROS_INFO("AEB_Activation!!");
	}
	else
	{
		ROS_INFO("Not AEB_Activation!!");
	}
}


int main(int argc, char **argv)
{
  int count = 0;
  
  ros::init(argc, argv, "aeb_controller2");

  ros::NodeHandle n;

  ros::Rate loop_rate(1); 
  
  ros::Subscriber sub1 = n.subscribe("seq", 1000, UltraSonarCallback2);
  ros::Subscriber sub2 = n.subscribe("range2", 1000, UltraSonarCallback3);
  
  while(ros::ok())
  {
	loop_rate.sleep();
	ros::spinOnce();	
	count++;
  }
  
  return 0;
}
