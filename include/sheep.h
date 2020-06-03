#ifndef SHEEP_H
#define SHEEP_H

#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <ctime>
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <helper_functions.h>

class Sheep
{
	std::string sheepName;
	ros::NodeHandle nh;
	ros::Subscriber odomSub;
	Eigen::Vector2f eta;
	Eigen::Vector2f z;
	Eigen::Vector2f zDot;
	Eigen::Vector2f zg;
	bool bearFound;
	bool dataSaved;
	bool firstMocap;
	bool chased;
	Eigen::Matrix4f Gammath;
	float kd;
	float betath;
	float kth;
	float Deltatth;
	Eigen::Matrix<float,4,2> thHat;
	Eigen::Matrix<float,4,3> basisMean;
	float basisVar;
	float thMax;
	std::mutex odomMutex,firstMutex,bearMutex,chasedMutex,thMutex,killSheepMutex,dataSaveMutex,SzMutex,cbMutex,destroySheepMutex;
	std::deque<Eigen::Vector2f> zBuffer;
	std::deque<Eigen::RowVector4f> SzBuffer;
	std::deque<ros::Time> timeBuffer;
	std::deque<float> dtBuffer;
	float unchasedTime;
	float chasedTime;
	ros::Time timeLast;
	bool killSheep;
	bool saveData;
	Eigen::Vector3f etaP;
	Eigen::Vector4f etaQ;
	ros::Time firstTime;
	Eigen::Matrix4f SzTSzSum;
	Eigen::Matrix<float,4,2> SzTDzSum;
	int stackIndex;
	float stackMinEig;
	int M;
	float lambda1;
	float T1;
	std::vector<Eigen::Vector2f> zData;
	std::vector<Eigen::Vector2f> zDotData;
	std::vector<float> timeData;
	std::vector<int> chasedData;
	std::vector<Eigen::Matrix<float,4,2>> thHatData;
	std::vector<Eigen::RowVector4f> SzData;

	float getminEig(Eigen::Matrix4f matrixSum);

	Eigen::RowVector2f getDz();

	Eigen::RowVector4f getscriptSz();

	void updateStack();

	// Eigen::Matrix<float,4,6> getthGamDot(Eigen::Matrix<float,4,6> thGam);
	//
	// Eigen::Matrix<float,4,6> integratethGam(Eigen::Matrix<float,4,6> thGam, float dt);

public:

	Sheep(std::string sheepNameInit, float kdInit, float betathInit, float kthInit, float GammathInit, float DeltatthInit,
	      Eigen::Vector2f zgInit, int MInit, float lambda1Init, bool saveDataInit,
	      float basisHeight, float basisCenters, float basisVarInit, float thInit, float thMaxInit);

	~Sheep();

	Eigen::RowVector4f getSz(Eigen::Vector2f y);

	void odomCB(const nav_msgs::Odometry::ConstPtr& msg);

	void saveRunData();

	Eigen::Vector2f getz();

	Eigen::Vector2f getzg();

	bool getfirstMocap();

	void setetaPose(Eigen::Vector3f etaPNew, Eigen::Vector4f etaQNew);

	void setchased(bool chasedNew);

	Eigen::MatrixXf getthHat();

	void setkillSheep();

	bool getdataSaved();

};

#endif
