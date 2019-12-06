#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <sstream>


#define WIDTH 160
#define HEIGHT 120
#define FIFO_SIZE 480
#define NUM_READINGS 30

void sendPose(float cum_dist);
void sendFrame();
void rpmCallback(const std_msgs::Int32::ConstPtr& msg);
void ultraReflectCallback(const std_msgs::Float64::ConstPtr& msg);
void currVelCallback(const std_msgs::Float64::ConstPtr& msg);
void goalDistCallback(const std_msgs::Float64::ConstPtr& msg);
void relaDistCallback(const std_msgs::Float64::ConstPtr& msg);
void cumDistCallback(const std_msgs::Float64::ConstPtr& msg);
void imageDataCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg);
void syncCallback(const std_msgs::Bool::ConstPtr& msg);
void flawInfoCallback(const geometry_msgs::Point::ConstPtr& msg);


ros::Publisher frame_pub;
ros::Publisher pose_pub;

std::vector<uint8_t> frame; 	
				
sensor_msgs::Image img;
geometry_msgs::PoseStamped pose_data;
sensor_msgs::LaserScan laser_data;
geometry_msgs::Point flaw_info;

size_t size = WIDTH*HEIGHT;
volatile bool sync_ros;
int count;
float goal_dist;
float cum_dist_;
ros::Time t;

int main(int argc, char **argv){

	int laser_freq = 100;
	sync_ros = false;
	count = 0;
	cum_dist_ = 0;

	laser_data.ranges.clear();
	laser_data.intensities.clear();

	for(int i = 0; i < NUM_READINGS; i++){

		laser_data.ranges.push_back(1);
		laser_data.intensities.push_back(0);
	}	
	
	ros::init(argc, argv, "supervisor");

	ros::NodeHandle nh;
	
	frame_pub = nh.advertise<sensor_msgs::Image>("frame", 1000);
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1000);
	ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
	ros::Subscriber flaw_info_sub = nh.subscribe("flaw_info", 100, flawInfoCallback);
	ros::Subscriber cum_dist_sub = nh.subscribe("cum_dist", 100, cumDistCallback);
	ros::Subscriber rela_dist_sub = nh.subscribe("rela_dist", 100, relaDistCallback);
	ros::Subscriber rpm_sub = nh.subscribe("rpm", 100, rpmCallback);
	ros::Subscriber curr_vel_sub = nh.subscribe("curr_vel", 100, currVelCallback);  
	ros::Subscriber image_data_sub = nh.subscribe("image_data", 100, imageDataCallback);
	ros::Subscriber sync_sub = nh.subscribe("sync", 100, syncCallback);	
	ros::Subscriber goal_dist_sub = nh.subscribe("goal_dist", 100, goalDistCallback);			

	ros::Rate loop_rate(10);

	while(ros::ok())
	{
	
		laser_data.header.stamp = ros::Time::now();
		laser_data.header.frame_id = "/laser_frame";			

		laser_data.angle_min = -3.14;
		laser_data.angle_max = 3.14;
		laser_data.angle_increment = 0.105;
		laser_data.time_increment = 0.01; 
		laser_data.range_min = 0.1; //fake
		laser_data.range_max = 10;

		laser_pub.publish(laser_data);

		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0 ;
}
void sendPose(float cum_dist){ 	
	if(ros::ok() && pose_pub){
		pose_data.header.stamp = ros::Time::now();
//		pose_data.header.frame_id = "/pose_frame";
		
		pose_data.pose.position.x = cum_dist;
		pose_data.pose.position.y = 0;
		pose_data.pose.position.z = 0;
		pose_data.pose.orientation.x = 0;
		pose_data.pose.orientation.y = 0;
		pose_data.pose.orientation.z = 0;		
		pose_data.pose.orientation.w = 1;
		
		pose_pub.publish(pose_data);
		ros::spinOnce();
	}
}


void sendFrame(){
	
	if(ros::ok() && frame_pub){
		img.header.stamp = ros::Time::now();
		img.header.frame_id = "/frame";

		img.height = HEIGHT;
		img.width = WIDTH;
		img.encoding = "mono8";
		img.is_bigendian = false;
		img.step = WIDTH;
		img.data = frame;		

		frame_pub.publish(img);
//		ros::spinOnce();
	}
}
void flawInfoCallback(const geometry_msgs::Point::ConstPtr& msg){}
void rpmCallback(const std_msgs::Int32::ConstPtr& msg){}

void currVelCallback(const std_msgs::Float64::ConstPtr& msg){}

void goalDistCallback(const std_msgs::Float64::ConstPtr& msg){
	float goal_dist_ = msg->data;
	goal_dist = goal_dist_;
}
void cumDistCallback(const std_msgs::Float64::ConstPtr& msg){
	float d = msg->data;	
	sendPose(d);
}

void relaDistCallback(const std_msgs::Float64::ConstPtr& msg){
	float d = msg->data;
	//sendPose(d);
}

void ultraReflectCallback(const std_msgs::Float64::ConstPtr& msg){}

void syncCallback(const std_msgs::Bool::ConstPtr& msg){
	sync_ros = msg->data;
}

void imageDataCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg){

	if(sync_ros == true || count != 0){	
		for(int i = 0; i < FIFO_SIZE; i++){
			frame.push_back(msg->data[i]);
		
		} 
		ROS_INFO("frame size: %d", (int)frame.size());

		if(frame.size() == WIDTH*HEIGHT){
		
			sendFrame();
			frame.clear();
			count = 0;
		}
		count = 1;		
	}
}


