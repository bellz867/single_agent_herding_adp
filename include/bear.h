#ifndef BEAR_H
#define BEAR_H

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
#include <random>
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <helper_functions.h>
#include <sheep.h>
#include <dynamics_estimator.h>

struct Bear
{
	std::string bearName;
	ros::NodeHandle nh;
	ros::Subscriber odomSub;
	ros::Publisher desOdomPub;
	ros::Publisher cmdPub;
	bool firstMocap;
	Sheep* sheep;
	Eigen::Matrix<float,4,3> basisMean;
	float basisVar;
	Eigen::Vector3f etaP;
	Eigen::Vector4f etaQ;
	Eigen::Vector3f etaPF;
	Eigen::Vector4f etaQF;
	Eigen::Vector4f etaQLast;
	float kd,betac,gamma1,kc1,kc2,ka1,ka2,knu;
	float PMag,PVar;
	int N;
	float keta1,keta2;
	Eigen::Matrix<float,7,7> Ka;
	Eigen::Matrix<float,7,7> Gammac;
	Eigen::Matrix<float,6,6> Qx;
	Eigen::Matrix<float,4,4> R;
	Eigen::Matrix<float,4,4> RI;
	Eigen::Matrix<float,6,4> G;
	Eigen::Matrix<float,4,6> GT;
	Eigen::Matrix<float,4,6> RIGT;
	Eigen::Matrix<float,6,6> GR;
	Eigen::Matrix<float,7,1> WcHat,WaHat;
	ros::Time firstTime;
	ros::Time timeLast;
	bool sheepFound;
	float height;
	float originRadius;
	Eigen::Vector4f wall;
	bool killBear;
	std::vector<float> timeData;
	std::vector<Eigen::Vector2f> etaData;
	std::vector<Eigen::Vector2f> etaDotData;
	std::vector<Eigen::Vector2f> uHatData;
	std::vector<Eigen::Matrix<float,7,1>> WcHatData;
	std::vector<Eigen::Matrix<float,7,1>> WaHatData;
	std::vector<Eigen::Vector2f> uWallData;
	std::vector<Eigen::Vector2f> mudData;
	std::vector<Eigen::Vector2f> muhatData;
	std::vector<Eigen::Matrix<float,6,1>> xData;
	bool saveData;
	bool dataSaved;
	bool firstx,firstetad;
	Eigen::Matrix<float,6,7> d;
	Eigen::Vector2f etad;
	DynamicsEstimator dynamicsEstimator;
	std::mutex destroyBearMutex;
	Eigen::Matrix<float,6,1> x0;


	// initialize constructor
	Bear(std::string bearNameInit, std::string sheepNameInit, float kdInit, float betathInit,
			float kthInit, float GammathInit, float betacInit, float gamma1Init, float GammacInit, int NInit, float kc1Init,
			float kc2Init, float ka1Init, float ka2Init, float knuInit, float heightInit, float originRadiusInit, bool saveDataInit,
			float lambda1Init, float DeltatthInit, std::vector<float> zgInit, int MInit, float basisCenters,
			float basisVarInit, float thInit, float thMaxInit, float KaInit, float keta1Init, float keta2Init, std::vector<float> QxInit,
			std::vector<float> RInit, float PMagInit, float PVarInit, std::vector<float> wallInit, float Wa, float Wc,
		  float kpl, float kdl, float gammal);

	~Bear();

	void wallCheck(Eigen::Vector2f& position, Eigen::Vector2f& velocity, Eigen::Vector4f wall, float dt);

	Eigen::RowVector4f getSz(Eigen::Vector2f zi);

	float getP(Eigen::Vector2f z, Eigen::Vector2f eta, Eigen::Vector2f zg);

	void odomCB(const nav_msgs::Odometry::ConstPtr& msg);

	// sends a kill command to all the sheep and bear and saves the data and kills the node
	void shutdownNode();
};

#endif
