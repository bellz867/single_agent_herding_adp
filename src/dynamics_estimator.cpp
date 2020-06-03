#include <dynamics_estimator.h>

DynamicsEstimator::DynamicsEstimator()
{
	thetaHat = 0.001*Eigen::Matrix<float,4,1>::Ones();
	xdHat = Eigen::Matrix<float,4,1>::Zero();
	xcHat = Eigen::Matrix<float,4,1>::Zero();
	firstUpdate = true;
	Pd = Eigen::Matrix<float,4,4>::Zero();
	Pc = Eigen::Matrix<float,4,4>::Zero();
	Q = Eigen::Matrix<float,4,4>::Zero();

	// velocity variance
	Pd.block(0,0,2,2) = 5.0*Eigen::Matrix2f::Identity();//covariance
	Pc.block(0,0,2,2) = 5.0*Eigen::Matrix2f::Identity();//covariance
	Q.block(0,0,2,2) = 5.0*Eigen::Matrix2f::Identity();//process covariance
	R = 5.0*Eigen::Matrix2f::Identity();//measurment covariance

	// acceleration variance
	Pd.block(2,2,2,2) = 5.0*Eigen::Matrix2f::Identity();//covariance
	Pc.block(2,2,2,2) = 5.0*Eigen::Matrix2f::Identity();//covariance
	Q.block(2,2,2,2) = 1.25*R;//process covariance

	//process jacobian
	Fd = Eigen::Matrix<float,4,4>::Identity();
	Fc = Eigen::Matrix<float,4,4>::Identity();

	//measruement jacobian
	H = Eigen::Matrix<float,2,4>::Zero();
	H.block(0,0,2,2) = Eigen::Matrix2f::Identity();
	HT = H.transpose();
}

void DynamicsEstimator::initialize(float kpInit, float kdInit, float gammaInit)
{
	kp = kpInit;
	kd = kdInit;
	gamma = gammaInit;
}

Eigen::Vector2f DynamicsEstimator::update(Eigen::Vector2f vd, Eigen::Vector2f vc, Eigen::Vector2f vVar, float dt)
{
	if (firstUpdate)
	{
		xdHat.segment(0,2) = vd;
		xcHat.segment(0,2) = vc;
		firstUpdate = false;
	}

	//update the estimator
	// std::cout << "\n vd-zLast.segment(0,2) dynamics \n" << (vd-zLast.segment(0,2))/dt << std::endl;
	// std::cout << "\n vc-zLast.segment(2,2) dynamics \n" << (vc-zLast.segment(2,2))/dt << std::endl;
	//predict
	R(0,0) = vVar(0);//measurment covariance
	R(1,1) = vVar(1);//measurment covariance
	Q.block(0,0,2,2) = 1.1*R;//process covariance
	Q.block(2,2,2,2) = 1.5*R;//process covariance

	Fd.block(0,2,2,2) = dt*Eigen::Matrix2f::Identity();
	xdHat += (xDot(xdHat)*dt);
	Pd = (Fd*Pd*Fd.transpose() + Q);
	Fc.block(0,2,2,2) = dt*Eigen::Matrix2f::Identity();
	xcHat += (xDot(xcHat)*dt);
	Pc = (Fc*Pc*Fc.transpose() + Q);

	//measurement
	Eigen::Matrix<float,2,2> argKd = H*Pd*HT+R;
	Eigen::JacobiSVD<Eigen::MatrixXf> svdargKd(argKd, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix<float,2,2> argKdI = svdargKd.solve(Eigen::Matrix<float,2,2>::Identity());
	Eigen::Matrix<float,4,2> Kd = Pd*HT*argKdI;
	Eigen::Matrix<float,2,2> argKc = H*Pc*HT+R;
	Eigen::JacobiSVD<Eigen::MatrixXf> svdargKc(argKc, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix<float,2,2> argKcI = svdargKc.solve(Eigen::Matrix<float,2,2>::Identity());
	Eigen::Matrix<float,4,2> Kc = Pc*HT*argKcI;

	xdHat += (Kd*(vd-xdHat.segment(0,2)));
	Pd = (Eigen::Matrix<float,4,4>::Identity() - Kd*H)*Pd;
	xcHat += (Kc*(vc-xcHat.segment(0,2)));
	Pc = (Eigen::Matrix<float,4,4>::Identity() - Kc*H)*Pc;


	std::cout << "\n vd dynamics \n" << vd << std::endl;
	std::cout << "\n vc dynamics \n" << vc << std::endl;
	std::cout << "\n xdHat dynamics \n" << xdHat << std::endl;
	std::cout << "\n xcHat dynamics \n" << xcHat << std::endl;
	std::cout << "\n Pd dynamics \n" << Pd << std::endl;
	std::cout << "\n Pc dynamics \n" << Pc << std::endl;
	std::cout << "\n Fd dynamics \n" << Fd << std::endl;
	std::cout << "\n Fc dynamics \n" << Fc << std::endl;

	//update dynamics estimator
	Eigen::Vector2f vdHat = xdHat.segment(0,2);
	Eigen::Vector2f vcHat = xcHat.segment(0,2);
	Eigen::Vector2f vdDotHat = xdHat.segment(2,2);
	Eigen::Vector2f vcDotHat = xcHat.segment(2,2);
	Eigen::Vector2f e = vdHat - vcHat;
	Eigen::Vector2f eDot = vdDotHat - vcDotHat;
	// Eigen::Vector2f vdDot = Eigen::Vector2f::Zero();
	Eigen::Matrix<float,2,4> Yd = Eigen::Matrix<float,2,4>::Zero();
	Eigen::Matrix<float,2,4> Yc = Eigen::Matrix<float,2,4>::Zero();
	for (int ii = 0; ii < 2; ii++)
	{
		Yd(ii,ii) = vdDotHat(ii);
		Yd(ii,ii+2) = vcHat(ii);
		Yc(ii,ii) = vcDotHat(ii);
		Yc(ii,ii+2) = vcHat(ii);
	}

	std::cout << "\n e dynamics \n" << e << std::endl;
	std::cout << "\n eDot dynamics \n" << eDot << std::endl;

	// Eigen::Vector2f u = Yd*thetaHat + k*e;
	// Eigen::Vector2f u = kp*e + kd*eDot;
	Eigen::Vector2f u = kp*e + kd*eDot + Yd*thetaHat;
	if (fabsf(u(0)) > 1.0)
	{
		u(0) = std::copysign(1.0,u(0));
	}
	if (fabsf(u(1)) > 1.0)
	{
		u(1) = std::copysign(1.0,u(1));
	}

	std::cout << "\n u dynamics feedforward \n" << (Yd*thetaHat) << std::endl;
	std::cout << "\n u dynamics feedback e \n" << (kp*e) << std::endl;
	std::cout << "\n u dynamics feedback eDot \n" << (kd*eDot) << std::endl;

	Eigen::Matrix<float,4,1> thetaHatDot = gamma*Yd.transpose()*e + gamma*(Yc.transpose()*u - Yc.transpose()*Yc*thetaHat);
	Eigen::Matrix<float,4,1> thetaHatNew = thetaHat + thetaHatDot*dt;

	for (int ii = 0; ii < 4; ii++)
	{
		if (thetaHatNew(ii) < 0.001)
		{
			thetaHatNew(ii) = 0.001;
		}
	}

	// thetaHat += (thetaHatDot*dt);
	thetaHat = thetaHatNew;
	std::cout << "\n thetaHat dynamics \n" << thetaHat << std::endl;

	return u;
}

Eigen::Matrix<float,4,1> DynamicsEstimator::xDot(Eigen::Matrix<float,4,1> x)
{
	Eigen::Matrix<float,4,1> xDot = Eigen::Matrix<float,4,1>::Zero();
	xDot.segment(0,2) = x.segment(2,2);
	return xDot;
}
