#include "ros/ros.h"
#include <sensor_msgs/Range.h>   //ultrasonic sensor message

sensor_msgs::Range avg_range;
float range1, range2, range3, range4, range5;

void RangeCallback(const sensor_msgs::Range::ConstPtr& msg)
{
	range1 = range2 = range3 = range4 = range5 = msg->range;
	avg_range.range = (range1 + range2 + range3 + range4 + range5)/5.0;
	//ROS_INFO("[%f]",avg_range.range);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sonar_avg_filter");

  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("/range", 1000, RangeCallback);
  
  ros::Publisher pub_avg_range = n.advertise<sensor_msgs::Range>("/range_avg", 1000);

  ros::Rate loop_rate(10); 
  
  while(ros::ok())
  {
	  pub_avg_range.publish(avg_range);
	  
	  loop_rate.sleep();
	  ros::spinOnce();	
  }
  return 0;
}
