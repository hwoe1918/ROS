#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h> //cmd_vel
#include <sensor_msgs/Range.h>   //ultrasonic sensor message

std_msgs::Bool flag_AEB;

void UltraCallBack(const sensor_msgs::Range::ConstPtr& msg)
{
  ROS_INFO("Sonar Range : [%f]", msg->range);
  if(msg->range <= 1.0)
  {
		flag_AEB.data = true;
  }
  else
  {
		flag_AEB.data = false;
  }
}

int main(int argc, char **argv)
{
  std_msgs::Int32 seq;
  
  int count=0; 
  
  ros::init(argc, argv, "aeb_controller1");

  ros::NodeHandle n;

  ros::Rate loop_rate(1); 
  
  ros::Subscriber sub = n.subscribe("range", 1000, UltraCallBack);
  
  ros::Publisher pub1 = n.advertise<std_msgs::Bool>("range2", 1000);
  ros::Publisher pub2 = n.advertise<std_msgs::Int32>("seq", 1000);
  
  while (ros::ok())
  {
	  seq.data = count;
	  
	  pub1.publish(flag_AEB);
	  pub2.publish(seq);
	  
	  loop_rate.sleep();
	  ros::spinOnce();
	  ++count;
  }
  return 0;
}
