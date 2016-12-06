#ifndef MESSAGEHANDLER
#define MESSAGEHANDLER
#include <sstream>

#include <data_logger.h>
#include <privateFunctions.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace hss
{

class MessageHandler
{
public:
	MessageHandler();
	~MessageHandler();

private:

	DataLogger* log;
	mutex log_mutex;

	ros::Time startTime;
	ros::NodeHandle nh;
	
	ros::Subscriber imu_sub;
	ros::Subscriber gps_fix_sub;
	ros::Subscriber gps_vel_sub;
	ros::Subscriber svo_sub;
	ros::Subscriber vicon_sub;

	void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
	void gps_fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
	void gps_vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void svo_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void vicon_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

};
#endif
