#ifndef DATALOGGER
#define DATALOGGER

#include <list>
#include <sstream>
#include <fstream>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <mxml.h>
#include <iostream>
#include <cmath>
#include <unistd.h>

#include "constants.h"
#include "privateFunctions.h"

#include <ros/ros.h>

namespace hss
{

class LogEntry
{
public:
	string str;
	LogIDs id;
	double timestamp;
};

class DataLogger
{
public:
    explicit DataLogger();
    virtual ~DataLogger();

    void addLine(const LogIDs &id, const string &str, const double &timestamp);
    void addEntry(const LogIDs &id, const string &str, const double &timestamp);
    void addEntry(const LogIDs &id, const Array2D<double> &data, const double &timestamp);
    void addEntry(const LogIDs &id, const double &data, const double &timestamp);

    void start()
	{ 
		 ROS_INFO("start data logger"); 
		 std::thread th(&DataLogger::run, this); 
		 th.detach(); 
		 ROS_INFO("logging thread detached");
	}
    void run();
    void shutdown()
	{
		ROS_INFO("shutdown data logger");
		mRunning = false;
		while(!mDone)
			this_thread::sleep_for( chrono::milliseconds(10) );
	}
    void clearLog()
	{
		shutdown(); 
		start();
	}


protected:
    string mDir;
    string mFilename;

    ofstream* mLogStream;

    void generateMatlabHeader();

    list<shared_ptr<LogEntry>> mLogQueue;

    bool mPaused, mRunning, mDone;

	std::mutex mMutex_addLine, mMutex_logQueue;
};

} //hss
#endif
