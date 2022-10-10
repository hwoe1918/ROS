#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int16.h"
#include <unistd.h>

#define RAD2DEG(x) ((x)*180./M_PI)


geometry_msgs::Twist twist;
int i, asdf;
bool s = false;
bool s2 = false;
bool s3 = false;


void twistCallback(const sensor_msgs::Joy& msg) {
	if(msg.buttons[6] != 0) {
		s = true;
	}
	else {
		s = false;
	}
}

void twistCallback1(const geometry_msgs::Twist& msg){
	if(s)
	{
		twist.angular.z = msg.angular.z;
		twist.linear.x = msg.linear.x * 1.4;
		if(twist.angular.z != 0) {
			twist.linear.x *= 2;
		}
	
		ROS_INFO("JoyCallback");
	}
	
	else
		s = false;
}



void twistCallback2(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if(s != true)
    {
		int count = (int)( 360. / RAD2DEG(msg->angle_increment));
		int sum=0; 
    
		for(int i = 0; i < count; i++)
		{
			float degree = RAD2DEG(msg->angle_min + msg->angle_increment * i);
        
			if( ((degree<=150) && (degree>=0)) ||  ( (degree<=360)&&(degree>=360-150)))
			{
				if(msg->ranges[i] <= 0.1f) 
				{
					sum++;
					ROS_INFO("LidarScan");
				}
			}
		}

		if(sum >= 20)   
		{
			s2 = true;
			ROS_INFO("LidarCallback : [%d]", sum);
			twist.linear.x = 0.0f;
		}
		else              
			s2 = false;
	}
}


void twistCallback3(const std_msgs::Int16& msg){
	if(s != true)
	{
		if(msg.data <= 10)
		{
			ROS_INFO("Sonar : [%d cm]", msg.data);
			twist.linear.x = 0.0f;
			s3 = true;
		}
		else
			s3 = false;
	}
}

void twistCallback4(const geometry_msgs::Twist& msg){
	if(s != true)
	{
		if((s2 != true) && (s3 != true)){
			twist.angular.z = msg.angular.z;
			//twist.linear.x = -(abs(msg.angular.z*1.4));
			twist.linear.x = -(msg.linear.x);
			
			//가로선 여기가 문제네
		
			
			ROS_INFO("CameraCallBack");
		}
		else
			twist.linear.x = 0;
	}
}

void asdfCallback(const std_msgs::Int16& msg) {
	asdf = msg.data;
}

int main(int argc, char **argv) {
	i = 0;
	int count = 0;
	
	ros::init(argc, argv, "cmd_vel_mux");
	ros::NodeHandle n;
	
	ros::Subscriber sub1 = n.subscribe("/joy", 10, &twistCallback);
	
	//Joystick
	ros::Subscriber sub2 = n.subscribe("/cmd_vel/xbox", 10, &twistCallback1);
	
	//Lidar
	ros::Subscriber sub_Lidar = n.subscribe("scan", 1000, &twistCallback2);
	
	//Camera, WayPoint & teleopKeyboard
	ros::Subscriber sub3 = n.subscribe("/cmd_vel", 10, &twistCallback4);
	ros::Subscriber asdf_sub = n.subscribe("/asdf", 10, &asdfCallback);
	
	//Sonar
	ros::Subscriber sub_Sonar = n.subscribe("Sonar_Distance_Int16", 10, &twistCallback3);
	
	//Waypoint
	//ros::Subscriber range_pub = n.subscribe("getRange",10);
	
	
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/Car_Control", 1000);
	
	ros::Rate loop_rate(20);

	while (ros::ok())
    {		
		//if(asdf == 1) {
			//twist.linear.x = 0;
			
		//}
	
	
		pub.publish(twist);
	
		loop_rate.sleep();
		ros::spinOnce();
		++count;	
	}
	return 0;
}

  


