#include "ros/ros.h"
#include "std_msgs/String.h"

#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>


/*
설 명 : I2C를 사용하여 데이타를 전송하는 예제이다.
*/
#include <string.h>  
#include <unistd.h>  
#include <errno.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <linux/i2c-dev.h>  
#include <sys/ioctl.h>  
#include <fcntl.h>  
#include <unistd.h>  

#include <sstream>

//i2c address  
#define ADDRESS 0x05

#define ANGLE 77

//I2C bus  
static const char *deviceName = "/dev/i2c-0";


#define RAD2DEG(x) ((x)*180./M_PI)

int steering_angle = ANGLE;
int motor_speed = 0;

int steering_angle_old = ANGLE;
int motor_speed_old = 0;

std_msgs::Int16 sonar;





unsigned char protocol_data[7] = {'S',0,0,'D',0,0,'#'};

int file_I2C;

int open_I2C(void)
{
   int file;  
   
    if ((file = open( deviceName, O_RDWR ) ) < 0)   
    {  
        fprintf(stderr, "I2C: Failed to access %s\n", deviceName);  
        exit(1);  
    }  
    printf("I2C: Connected\n");  
  
   
    printf("I2C: acquiring buss to 0x%x\n", ADDRESS);  
    if (ioctl(file, I2C_SLAVE, ADDRESS) < 0)   
    {  
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", ADDRESS);  
        exit(1);  
    } 
    
    return file; 
}




void close_I2C(int fd)
{
   close(fd);
}

void CarControlCallback(const geometry_msgs::Twist& msg)
{
	
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
   steering_angle = (int)(msg.angular.z+ANGLE) ;
   
   
   if(steering_angle >= 35+ANGLE)  steering_angle = ANGLE+35;
   if(steering_angle <=-35+ANGLE)  steering_angle = ANGLE-35;
   
   motor_speed = (int)(msg.linear.x);
   if(motor_speed>=255)   motor_speed = 150;
   if(motor_speed<=-255)  motor_speed = -150;
	
}


void CarSteerControlCallback(const std_msgs::Int16& angle)
{
  steering_angle = (int)(angle.data*10+ANGLE) ;
  
  if(steering_angle >= 40+ANGLE)  steering_angle = ANGLE+40;
  if(steering_angle <= -40+ANGLE)  steering_angle = ANGLE-40;
  
}




int main(int argc, char **argv)
{
  char buf[1];
  ros::init(argc, argv, "Car_Control");

  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("/Car_Control",1000, &CarControlCallback);
  //ros::Subscriber sub1 = n.subscribe("/cmd_vel/xbox", 10, &CarControlCallback);
  //ros::Subscriber sub2 = n.subscribe("/Car_Control_cmd/SteerAngle_Int16",10, &CarSteerControlCallback);  
  //ros::Subscriber sub3 = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &scanCallback);
  ros::Subscriber sub4 = n.subscribe("/cmd_vel", 10, &CarControlCallback);
  //ros::Subscriber sub5 = n.subscribe("/joy", 10, &CarControlCallback);
  
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher car_control_pub1 = n.advertise<std_msgs::String>("Car_Control/SteerAngle_msgs", 10);
  ros::Publisher car_control_pub2 = n.advertise<std_msgs::String>("Car_Control/Speed_msgs", 10);
  ros::Publisher car_control_pub3 = n.advertise<std_msgs::Int16>("Car_Control/SteerAngle_Int16", 10);
  ros::Publisher car_control_pub4 = n.advertise<std_msgs::Int16>("Car_Control/Speed_Int16", 10);
  ros::Publisher car_control_pub5 = n.advertise<std_msgs::Int16>("Sonar_Distance_Int16", 10);
 
	//ROS_INFO("xbox : %f", sub1);
	//ROS_INFO("key : %f", sub4);
	//ROS_INFO("joy : %f", sub5);
	
 
  ros::Rate loop_rate(20);  // 10
  file_I2C = open_I2C();
  if(file_I2C < 0)
  {
	  ROS_ERROR_STREAM("Unable to open I2C");
	  return -1;
  }
  else
  {
	  ROS_INFO_STREAM("I2C is Connected");
  }


  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg; 

    std_msgs::Int16 steerangle;
    std_msgs::Int16 carspeed;
    std::stringstream ss;    
    std::string data;
    data = std::to_string(steering_angle);

    steerangle.data = steering_angle;
    carspeed.data = motor_speed;
    ss<<data;
    msg.data = ss.str();
   //ROS_INFO("Steer : %s", msg.data.c_str());

    car_control_pub1.publish(msg);
	
    data = std::to_string(motor_speed);
    msg.data = data;
    car_control_pub2.publish(msg);
    ROS_INFO("Speed : %s", msg.data.c_str());
    
    protocol_data[1] = (steering_angle&0xff00)>>8 ;
    protocol_data[2] = (steering_angle&0x00ff);
    protocol_data[4] = (motor_speed&0xff00)>>8 ;
    protocol_data[5] = (motor_speed&0x00ff);
    
    ROS_INFO("data = %c %d %d %c %d %d %c", protocol_data[0], protocol_data[1], protocol_data[2], protocol_data[3], protocol_data[4], protocol_data[5], protocol_data[6]);
    
    if(steering_angle != steering_angle_old) 
    {
       //write_serial(protocol_data,5);
       write(file_I2C, protocol_data, 7);
    }

     if(motor_speed != motor_speed_old)
    {
      //write_serial(protocol_data,5);
       write(file_I2C, protocol_data, 7);
    }
    
    read(file_I2C,buf,1);
    sonar.data = buf[0];
	car_control_pub5.publish(sonar);

    steering_angle_old = steering_angle;
    motor_speed_old = motor_speed; 
     
    car_control_pub3.publish(steerangle);
    car_control_pub4.publish(carspeed);
    //ROS_INFO("Sonar : %d", sonar.data);
    loop_rate.sleep();
    ros::spinOnce();
    ++count;
    
  }

  motor_speed = 0;
  protocol_data[4] = (motor_speed&0xff00)>>8 ;
  protocol_data[5] = (motor_speed&0x00ff);
  write(file_I2C, protocol_data, 1);
  read(file_I2C,buf,2);
  close_I2C(file_I2C);
  return 0;
}

