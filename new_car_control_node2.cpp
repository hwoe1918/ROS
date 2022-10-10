#define DEBUG 1
#define DEBUG_ROS_INFO 1 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
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

//I2C bus  
static const char *deviceName = "/dev/i2c-0";

#define MAX_L_STEER -30
#define MAX_R_STEER 30
#define STEER_NEUTRAL_ANGLE 77

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

// unit : m

int  Base_Speed = 130;

/////////////////////IMU//////////////////////////
long encoder1data = 0;
long encoder2data = 0;

double roll,pitch,yaw,yaw_old;
double roll_d,pitch_d,yaw_d,yaw_d_old;
double delta_yaw_d, delta_yaw;
int  imu_read_flag_start = 0;
bool imu_init_angle_flag = 0;
float init_yaw_angle = 0.0;
float init_yaw_angle_d = 0.0;
float imu_offset_angle = 0;  // degree


//////////////////////////////// Odometry //////////////////////////////

struct BaseSensorData
{
    int encoder = 0;   
    int encoder_old = 0; 
}myBaseSensorData;

struct OdomCaculateData
{
    //motor params
    float distance_ratio=1140 ;//0.000176; //unit: m/encode  67.5 
    float wheel_distance= 0.226; //0.1495; //unit: m
    float encode_sampling_time=0.05; //unit: s-> 20hz
    float cmd_vel_linear_max=0.8; //unit: m/s
    float cmd_vel_angular_max=0.0; //unit: rad/s
    //odom result
    float position_x=0.0; //unit: m
    float position_y=0.0; //unit: m
    float oriention=0.0; //unit: rad
    float velocity_linear=0.0; //unit: m/s
    float velocity_angular=0.0; //unit: rad/s
    
}myOdomCaculateData;

void odometry_cal(int encoder_distance)
{		
  myBaseSensorData.encoder = encoder_distance;
  int delta_encoder = myBaseSensorData.encoder - myBaseSensorData.encoder_old;
  float base_link_delta_x;   float base_link_delta_y;  float base_link_delta_l;
  float odom_delta_x;        float odom_delta_y;  
  float radius;
  float temp = yaw;
  float temp_d = yaw_d;
  
  delta_yaw_d = yaw_d - yaw_d_old;
  
  delta_yaw = yaw - yaw_old;
  
  base_link_delta_l =  (float)delta_encoder ;  /// myOdomCaculateData.distance_
  
  if(fabs(delta_yaw)>1.0e-7) 
  {
	  
	 radius = base_link_delta_l / delta_yaw;
	 if(delta_yaw < 0)  base_link_delta_y = -radius*(1 - cos(delta_yaw));
	 else               base_link_delta_y =  radius*(1 - cos(delta_yaw));
	 
	 base_link_delta_x = radius * sin(delta_yaw);	
	 
	 //delta_x  = delta_d *cos(myOdomCaculateData.oriention - delta_theta);
     //delta_y  = delta_d *sin(myOdomCaculateData.oriention - delta_theta);      
  }
  else  
  {
	base_link_delta_x = base_link_delta_l;
	base_link_delta_y = 0.0;
  }  
    base_link_delta_x /= myOdomCaculateData.distance_ratio;
    base_link_delta_y /= myOdomCaculateData.distance_ratio;
    
    odom_delta_x =  base_link_delta_x * cos(yaw_old)  - base_link_delta_y * sin(yaw_old);   // rotation_matrix
	odom_delta_y =  base_link_delta_x * sin(yaw_old)  + base_link_delta_y * cos(yaw_old);
  
    //printf("encoder :  %d  %f  %f %f \n", delta_encoder ,yaw_d, yaw_d_old , delta_yaw_d);
    //printf("delta_x,y : [%f  %f] [%f %f]\n",  base_link_delta_x, base_link_delta_y, odom_delta_x , odom_delta_y);
  
  //printf("radius : %f \n", radius);
  // printf("delta_encoder %d %d\n",myBaseSensorData.encoder,myBaseSensorData.encoder_old);
  
    
  myOdomCaculateData.position_x += odom_delta_x; //unit: m
  myOdomCaculateData.position_y += odom_delta_y; //unit: m
  myOdomCaculateData.oriention   = yaw ; //unit: rad	
 // printf(" %6.3f %6.3f %6.3f\n", myOdomCaculateData.position_x, myOdomCaculateData.position_y,RAD2DEG(myOdomCaculateData.oriention));
  
  if(DEBUG == 1)printf(" %6.3lf %6.3lf %6.3lf \n", 	myOdomCaculateData.position_x, myOdomCaculateData.position_y, RAD2DEG(myOdomCaculateData.oriention));
     
  yaw_d_old = temp_d;       
  yaw_old = temp;      
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
	 char buf[8];
       
      tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
      tf2::Matrix3x3 m(q);     
            
      m.getRPY(roll, pitch, yaw);
      yaw = yaw - DEG2RAD(imu_offset_angle);
      roll_d  = RAD2DEG(roll);
      pitch_d = RAD2DEG(pitch);
      yaw_d   = RAD2DEG(yaw); 
      ROS_INFO("yaw_d: %lf \n", yaw_d);              
}

int steering_angle = STEER_NEUTRAL_ANGLE;
int motor_speed = 0;

int steering_angle_old =  STEER_NEUTRAL_ANGLE;
int motor_speed_old = 0;

std_msgs::Int16 sonar;

unsigned long  encoder_temp = 0;
int  encoder = 0;

unsigned char protocol_data[7] = {'S',0,0,'D',0,0,'#'}; // start byte '#' - end bytte '*'
// char frameid[] ="/sonar_range";

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

/*
설명 : 시리얼포트를 닫는다.
*/

void close_I2C(int fd)
{
   close(fd);
}

void CarControlCallback(const geometry_msgs::Twist& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  
   steering_angle = (int)(-msg.angular.z+STEER_NEUTRAL_ANGLE) ;
  
   
   if(steering_angle >= 35+STEER_NEUTRAL_ANGLE)  steering_angle = STEER_NEUTRAL_ANGLE+35;
   if(steering_angle <=-35+STEER_NEUTRAL_ANGLE)  steering_angle = STEER_NEUTRAL_ANGLE-35;
   
   Base_Speed  = (int)msg.linear.x;
   motor_speed = Base_Speed;
   if(motor_speed>=255)   motor_speed = 150;
   if(motor_speed<=-255)  motor_speed = -150;
   
}


void CarSteerControlCallback(const std_msgs::Int16& angle)
{
  steering_angle = (int)(angle.data+STEER_NEUTRAL_ANGLE) ;
  
  if(steering_angle >= 40+STEER_NEUTRAL_ANGLE)  steering_angle = STEER_NEUTRAL_ANGLE+40;
  if(steering_angle <=-40+STEER_NEUTRAL_ANGLE)  steering_angle = STEER_NEUTRAL_ANGLE-40;  
}

/*
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = (int)( 360. / RAD2DEG(scan->angle_increment));
    int sum=0; 
   // ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
   // ROS_INFO("%f %f",scan->scan_time , scan->time_increment);
   // ROS_INFO("angle_range, %f, %f %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max), RAD2DEG(scan->angle_increment));
  
    for(int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
    }
    
    
    for(int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        
        if(((degree>=90-15)&&(degree<=90+15)))
        {
         // ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
          if(scan->ranges[i] <= 0.5) 
          {
            sum++;
            //ROS_INFO("sum 2= %d", sum);
          }          
        }      
    }
     //ROS_INFO("sum = %d", sum);
     if(sum >=10)   
     {
       motor_speed = 0;
       ROS_INFO("Obstacle Detected !!");
     }
     else           motor_speed = Base_Speed;     
}
*/


int main(int argc, char **argv)
{
  char buf[1];
  ros::init(argc, argv, "new_car_control_node2");

  ros::NodeHandle n;  
  std::string cmd_vel_topic = "cmd_vel";
  std::string odom_pub_topic = "odom";
  std::string imu_topic = "handsfree/imu";
  /*other*/
  ros::param::get("~cmd_vel_topic", cmd_vel_topic);
  ros::param::get("~odom_pub_topic", odom_pub_topic);
  
  
  /////////////////////////////Subscriber/////////////////////////////////
  ros::Subscriber sub = n.subscribe("/Car_Control",1000, &CarControlCallback);
  //ros::Subscriber sub1 = n.subscribe("/cmd_vel", 10, &CarControlCallback);
  //ros::Subscriber sub2 = n.subscribe("/Car_Control_cmd/SteerAngle_Int16",10, &CarSteerControlCallback);  
  //ros::Subscriber sub3 = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &scanCallback);
  ros::Subscriber subIMU = n.subscribe(imu_topic, 20, &imuCallback);
  
  
  /////////////////////////////Publisher/////////////////////////////////
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher car_control_pub1 = n.advertise<std_msgs::String>("Car_Control/SteerAngle_msgs", 10);
  ros::Publisher car_control_pub2 = n.advertise<std_msgs::String>("Car_Control/Speed_msgs", 10);
  ros::Publisher car_control_pub3 = n.advertise<std_msgs::Int16>("Car_Control/SteerAngle_Int16", 10);
  ros::Publisher car_control_pub4 = n.advertise<std_msgs::Int16>("Car_Control/Speed_Int16", 10);
  ros::Publisher car_control_pub5 = n.advertise<std_msgs::Int16>("Sonar_Distance_Int16", 10);
  ros::Publisher car_control_pub6 = n.advertise<nav_msgs::Odometry>(odom_pub_topic, 20);
    
 /*
  ////////////////   sonar _sensor //////////////////////
  sensor_msgs::Range sonar_msg;
  sonar_msg.header.frame_id =  frameid;
  sonar_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_msg.field_of_view = (30.0/180.0) * 3.14;
  sonar_msg.min_range = 0.0;
  sonar_msg.max_range = 1.50;  //[unit :m]
  */
  //////////////////  odometry  //////////////////////
  std::string odom_frame_id = "odom";
  std::string odom_child_frame_id = "base_footprint";
 
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

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  static tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom;
  geometry_msgs::Quaternion odom_quat;
  
  //covariance matrix
  float covariance[36] = {0.01,   0,    0,     0,     0,     0,  // covariance on gps_x
                            0,  0.01, 0,     0,     0,     0,  // covariance on gps_y
                            0,  0,    99999, 0,     0,     0,  // covariance on gps_z
                            0,  0,    0,     99999, 0,     0,  // large covariance on rot x
                            0,  0,    0,     0,     99999, 0,  // large covariance on rot y
                            0,  0,    0,     0,     0,     0.01};  // large covariance on rot z 
  //load covariance matrix
  for(int i = 0; i < 36; i++)
  {
      odom.pose.covariance[i] = covariance[i];;
  }     
  
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    
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
    ROS_INFO("Steer : %s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    car_control_pub1.publish(msg);   
  
    data = std::to_string(motor_speed);
    msg.data = data;
    car_control_pub2.publish(msg);
    ROS_INFO("Speed : %s", msg.data.c_str());
    
    //motor_speed -= (int)fabs((float)(steering_angle-96) * 0.4);

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
    
    read(file_I2C,buf,6);
    //
    sonar.data = (buf[0]*256+buf[1]);
    
    encoder =  buf[2]*256*256*256 + buf[3]*65536 + buf[4]*256 + buf[5];
    
    car_control_pub5.publish(sonar);
	
    steering_angle_old = steering_angle;
    motor_speed_old = motor_speed ; 
    
    ROS_INFO("Encoder : %ld", encoder);
    
    if(DEBUG == 1)
    {    
	   printf("sonar %6.3lf \n",sonar);     printf("encoder temp %ld \n",encoder_temp); printf("encoder %ld \n",encoder);
	}
    
    car_control_pub3.publish(steerangle);//
    car_control_pub4.publish(carspeed);//
    
    myBaseSensorData.encoder = encoder; 
    printf("\n\nencoder_distance : %f\n\n", (float)myBaseSensorData.encoder/1140);
	odometry_cal(encoder);
	myBaseSensorData.encoder_old =  myBaseSensorData.encoder;
	
	
    //odom_oriention trans to odom_quat
    odom_quat = tf::createQuaternionMsgFromYaw(myOdomCaculateData.oriention);//yaw trans quat
 
    //pub tf(odom->base_footprint)
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = odom_frame_id;     
    odom_trans.child_frame_id = odom_child_frame_id;       
    odom_trans.transform.translation.x = myOdomCaculateData.position_x;
    odom_trans.transform.translation.y = myOdomCaculateData.position_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    //pub odom
    odom.header.stamp = ros::Time::now(); 
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = odom_child_frame_id;       
    odom.pose.pose.position.x = myOdomCaculateData.position_x;     
    odom.pose.pose.position.y = myOdomCaculateData.position_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;       
    //odom.twist.twist.linear.x = myOdomCaculateData.velocity_linear;
    //odom.twist.twist.angular.z = myOdomCaculateData.velocity_angular;
    odom_broadcaster.sendTransform(odom_trans);
    
    car_control_pub6.publish(odom);
    
    loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }

  motor_speed = 0;

  protocol_data[4] = (motor_speed&0xff00)>>8 ;
  protocol_data[5] = (motor_speed&0x00ff);
  
  write(file_I2C, protocol_data, 6);
  read(file_I2C,buf,6);
  close_I2C(file_I2C);
  return 0;
}

