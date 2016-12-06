#include <message_handler.h>

using namespace hss;
using namespace TNT;

typedef string String;

MessageHandler::MessageHandler()
{
	log = new DataLogger;
	log->start();
	
	startTime = ros::Time::now();
	log->addEntry( LOG_GLOBAL_TIME, (startTime.toSec()), 0.0 );

	imu_sub = nh.subscribe("mavros/imu/data", 10, 
			&MessageHandler::imu_callback, this);
	gps_fix_sub = nh.subscribe("mavros/global_position/raw/fix", 10, 
			&MessageHandler::gps_fix_callback, this);
	gps_vel_sub = nh.subscribe("mavros/global_position/raw/gps_vel", 10, 
			&MessageHandler::gps_vel_callback, this);
	svo_sub = nh.subscribe("svo/pose", 10,
			&MessageHandler::svo_callback, this);
	vicon_sub = nh.subscribe("vicon/pose", 10,
			&MessageHandler::vicon_callback, this);
}

MessageHandler::~MessageHandler()
{
	delete log;
}

void MessageHandler::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	double timestamp = (msg->header.stamp - startTime).toSec();

	Array2D<double> buffer = Zeros(10,1);
	buffer[0][0] = msg->orientation.x;
	buffer[1][0] = msg->orientation.y;
	buffer[2][0] = msg->orientation.z;
	buffer[3][0] = msg->orientation.w;
	buffer[4][0] = msg->angular_velocity.x;
	buffer[5][0] = msg->angular_velocity.y;
	buffer[6][0] = msg->angular_velocity.z;
	buffer[7][0] = msg->linear_acceleration.x;
	buffer[8][0] = msg->linear_acceleration.y;
	buffer[9][0] = msg->linear_acceleration.z;

	log_mutex.lock();
	log->addEntry( LOG_MAVROS_IMU, buffer, timestamp);
	log_mutex.unlock();
}

void MessageHandler::gps_fix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	double timestamp = (msg->header.stamp - startTime).toSec();

	Array2D<double> buffer = Zeros(4,1);
	buffer[0][0] = msg->latitude;
	buffer[1][0] = msg->longitude;
	buffer[2][0] = msg->altitude;
	buffer[3][0] = msg->status.status;

	log_mutex.lock();
	log->addEntry( LOG_MAVROS_GPS_FIX, buffer, timestamp);
	log_mutex.unlock();
}

void MessageHandler::gps_vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	double timestamp = (msg->header.stamp - startTime).toSec();

	Array2D<double> buffer = Zeros(6,1);
	buffer[0][0] = msg->twist.linear.x;
	buffer[1][0] = msg->twist.linear.y;
	buffer[2][0] = msg->twist.linear.z;
	buffer[3][0] = msg->twist.angular.x;
	buffer[4][0] = msg->twist.angular.y;
	buffer[5][0] = msg->twist.angular.z;

	log_mutex.lock();
	log->addEntry( LOG_MAVROS_GPS_VEL, buffer, timestamp);
	log_mutex.unlock();
}

void MessageHandler::svo_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	double timestamp = (msg->header.stamp - startTime).toSec();

	Array2D<double> buffer = Zeros(7,1);
	buffer[0][0] = msg->pose.pose.position.x;
	buffer[1][0] = msg->pose.pose.position.y;
	buffer[2][0] = msg->pose.pose.position.z;
	buffer[3][0] = msg->pose.pose.orientation.x;
	buffer[4][0] = msg->pose.pose.orientation.y;
	buffer[5][0] = msg->pose.pose.orientation.z;
	buffer[6][0] = msg->pose.pose.orientation.w;

	log_mutex.lock();
	log->addEntry( LOG_SVO_POSE, buffer, timestamp);
	log_mutex.unlock();
}

void MessageHandler::vicon_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	double timestamp = (msg->header.stamp - startTime).toSec();

	Array2D<double> buffer = Zeros(7,1);
	buffer[0][0] = msg->pose.position.x;
	buffer[1][0] = msg->pose.position.y;
	buffer[2][0] = msg->pose.position.z;
	buffer[3][0] = msg->pose.orientation.x;
	buffer[4][0] = msg->pose.orientation.y;
	buffer[5][0] = msg->pose.orientation.z;
	buffer[6][0] = msg->pose.orientation.w;

	log_mutex.lock();
	log->addEntry( LOG_VICON_POSE, buffer, timestamp);
	log_mutex.unlock();
}
