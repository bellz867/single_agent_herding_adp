#ifndef DynamicsEstimator_H
#define DynamicsEstimator_H

#define _USE_MATH_DEFINES
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct DynamicsEstimator
{
	Eigen::Matrix<float,4,1> thetaHat;
	Eigen::Matrix<float,4,1> xdHat;
	Eigen::Matrix<float,4,4> Pd;
  Eigen::Matrix<float,4,1> xcHat;
	Eigen::Matrix<float,4,4> Pc;
	Eigen::Matrix<float,4,4> Q;
	Eigen::Matrix<float,2,2> R;
	Eigen::Matrix<float,4,4> Fd;
  Eigen::Matrix<float,4,4> Fc;
	Eigen::Matrix<float,2,4> H;
	Eigen::Matrix<float,4,2> HT;

	float kp;
  float kd;
	float gamma;
	bool firstUpdate;

	DynamicsEstimator();

  void initialize(float kpInit, float kdInit, float gammaInit);

	Eigen::Vector2f update(Eigen::Vector2f vd, Eigen::Vector2f v, Eigen::Vector2f vVar, float dt);

	Eigen::Matrix<float,4,1> xDot(Eigen::Matrix<float,4,1> x);
};

#endif
