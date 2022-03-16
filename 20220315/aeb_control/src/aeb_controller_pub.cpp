
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h> //cmd_vel
#include <sensor_msgs/Range.h>   //ultrasonic sensor message

int main(int argc, char **argv)
{
  std_msgs::Float32 aeb;
  std_msgs::Int32 seq;
  
  int count=0; 
  
  ros::init(argc, argv, "aeb_controller");

  ros::NodeHandle n;

  ros::Rate loop_rate(2); 
  
  ros::Publisher pub1 = n.advertise<std_msgs::Float32>("range2", 1000);
  ros::Publisher pub2 = n.advertise<std_msgs::Int32>("seq", 1000);
  
  srand(time(NULL));
  
  while (ros::ok())
  {
	  aeb.data = (double)(rand()%50)/10;
	  seq.data = count;
	  
	  pub1.publish(aeb);
	  pub2.publish(seq);
	  
	  loop_rate.sleep();
	  ros::spinOnce();
	  ++count;
  }
  return 0;
}
