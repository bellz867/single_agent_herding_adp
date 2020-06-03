#include <sheep.h>

// initialize constructor
Sheep::Sheep(std::string sheepNameInit, float kdInit, float betathInit, float kthInit, float GammathInit, float DeltatthInit,
      Eigen::Vector2f zgInit, int MInit, float lambda1Init, bool saveDataInit,
      float basisHeight, float basisCenters, float basisVarInit, float thInit, float thMaxInit)
{
	sheepName = sheepNameInit;
	odomSub = nh.subscribe(sheepName+"/odomEKF",1,&Sheep::odomCB,this);   // subscribe to sheep pose from mocap

	// intialize
  firstMocap = true;
  z = Eigen::Vector2f::Zero();
	eta = Eigen::Vector2f::Zero();
  zDot = Eigen::Vector2f::Zero();
	zg = zgInit;
  kd = kdInit;

  chased = true;
  unchasedTime = 0.0;
	chasedTime = 0.0;
  killSheep = false;
	saveData = saveDataInit;
  dataSaved = false;

	etaP = Eigen::Vector3f::Zero();
	etaQ = Eigen::Vector4f::Zero();
	etaQ(0) = 1.0;

  thMax = thMaxInit;
  betath = betathInit;
  kth = kthInit;
  Gammath = GammathInit*Eigen::Matrix4f::Identity();
  Deltatth = DeltatthInit;
  lambda1 = lambda1Init;
  T1 = 0.0;
	M = MInit;
  SzTSzSum = Eigen::Matrix4f::Zero();
  SzTDzSum = Eigen::Matrix<float,4,2>::Zero();
	stackIndex = 0;
	stackMinEig = 0;

  //4 basis points located below quad in 4 cubic quadrants
  basisVar = basisVarInit;

  // front left ++
  basisMean(0,0) = basisCenters;
  basisMean(0,1) = basisCenters;
  basisMean(0,2) = -basisHeight;
  thHat(0,0) = thInit;
  thHat(0,1) = thInit;

  // back left -+
  basisMean(1,0) = -basisCenters;
  basisMean(1,1) = basisCenters;
  basisMean(1,2) = -basisHeight;
  thHat(1,0) = -thInit;
  thHat(1,1) = thInit;

  // back right --
  basisMean(2,0) = -basisCenters;
  basisMean(2,1) = -basisCenters;
  basisMean(2,2) = -basisHeight;
  thHat(2,0) = -thInit;
  thHat(2,1) = -thInit;

  // front right +-
  basisMean(3,0) = basisCenters;
  basisMean(3,1) = -basisCenters;
  basisMean(3,2) = -basisHeight;
  thHat(3,0) = thInit;
  thHat(3,1) = -thInit;
}

Sheep::~Sheep()
{
  std::lock_guard<std::mutex> destroySheepGuard(destroySheepMutex);
  if (!dataSaved && saveData)
  {
    saveRunData();
  }
}

// eigenvalue callback
float Sheep::getminEig(Eigen::Matrix4f matrixSum)
{
  //get the minimum eigenvalue for the j stack
  Eigen::Vector4f matrixSumEig = matrixSum.eigenvalues().real();
  return matrixSumEig.minCoeff();
}

// get the basis functions
Eigen::RowVector4f Sheep::getSz(Eigen::Vector2f zi)
{
  std::lock_guard<std::mutex> SzMutexGuard(SzMutex);
	Eigen::RowVector4f Szi = Eigen::RowVector4f::Zero();
  Eigen::Vector3f etazDiff = Eigen::Vector3f(zi(0),zi(1),0.0) - etaP;
	Eigen::Vector3f X = rotatevec(etazDiff,getqInv(etaQ));
	for (int jj = 0; jj < 4; jj++)
	{
    Eigen::Vector3f basisMeanjj = basisMean.block(jj,0,1,3).transpose();
		float XDiff = (X - basisMeanjj).transpose()*(X - basisMeanjj);
		Szi(jj) = 1.0/sqrtf(2.0*M_PIl*basisVar)*exp(-1.0/(2.0*basisVar)*pow(XDiff,2));
	}
	return Szi;
}

// get the z difference approx
Eigen::RowVector2f Sheep::getDz()
{
  Eigen::RowVector2f Dz = zBuffer.at(zBuffer.size()-1).transpose()-zBuffer.at(0).transpose();
	return kd*Dz;
}

// get integral of Sz approx
Eigen::RowVector4f Sheep::getscriptSz()
{
	Eigen::RowVector4f scriptSz = Eigen::RowVector4f::Zero();
	for (int i = 0; i < dtBuffer.size(); i++)
	{
		scriptSz += (dtBuffer.at(i)*SzBuffer.at(i));
	}
	return scriptSz;
}

//update a stack if the new value increases the minimum eigenvalue and the position change of the
//sheep is not too small or large
void Sheep::updateStack()
{
  if (stackIndex < M)
  {
    Eigen::RowVector2f DzWorld = getDz();//change in z must be converted to body frame from world frame for weights
    Eigen::Vector3f DzCol = rotatevec(Eigen::Vector3f(DzWorld(0),DzWorld(1),0.0),getqInv(etaQ));
    Eigen::RowVector4f scriptSz = getscriptSz();
  	Eigen::RowVector2f Dz(DzCol(0),DzCol(1));
    Eigen::Vector4f scriptSzT = scriptSz.transpose();
  	Eigen::Matrix4f SzTSz = scriptSzT*scriptSz;
  	Eigen::Matrix<float,4,2> SzTDz = scriptSzT*Dz;

  	if ((0.1 < Dz.norm()) && (Dz.norm() < (thMax*Deltatth)))
  	{
      Eigen::Matrix4f SzTSzSumNew = SzTSzSum + SzTSz;
      float minEigNew = getminEig(SzTSzSumNew);
      // float minEigOld = getminEig(SzTSzSum);

      if (minEigNew > stackMinEig)
      {
        SzTSzSum = SzTSzSumNew;
        SzTDzSum += SzTDz;
        stackMinEig = minEigNew;
        stackIndex++;
      }
  	}
  }
}

// //theta and gamma dot
// Eigen::Matrix<float,4,6> Sheep::getthGamDot(Eigen::Matrix<float,4,6> thGam)
// {
//   Eigen::Matrix<float,4,2> thHati = thGam.block(0,0,4,2);
//   Eigen::Matrix<float,4,4> Gammathi = thGam.block(0,2,4,4);
// 	Eigen::Matrix<float,4,2> thHatDot = kth*Gammathi*(SzTDzSum - SzTSzSum*thHati);
//   Eigen::Matrix<float,4,4> GammathDot = betath*Gammathi - kth*Gammathi*SzTSzSum*Gammathi;
//
//   Eigen::Matrix<float,4,6> thGamDot;
//   thGamDot.block(0,0,4,2) = thHatDot;
//   thGamDot.block(0,2,4,4) = GammathDot;
// 	return thGamDot;
// }
//
// //integrate theta and gamma
// Eigen::Matrix<float,4,6> Sheep::integratethGam(Eigen::Matrix<float,4,6> thGam, float dt)
// {
// 	Eigen::Matrix<float,4,6> C1 = getthGamDot(thGam);
// 	Eigen::Matrix<float,4,6> C2 = getthGamDot(thGam+dt*C1/2.0);
// 	Eigen::Matrix<float,4,6> C3 = getthGamDot(thGam+dt*C2/2.0);
// 	Eigen::Matrix<float,4,6> C4 = getthGamDot(thGam+dt*C3);
// 	Eigen::Matrix<float,4,6> thGamDot = (C1 + 2.0*C2 + 2.0*C3 + C4)/6.0;
//
// 	return (thGam + dt*thGamDot);
// }

// sheep odom callback
void Sheep::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  ros::Time timeNew = msg->header.stamp;
  // std::lock_guard<std::mutex> cbMutexGuard(cbMutex);
  z = Eigen::Vector2f(msg->pose.pose.position.x,msg->pose.pose.position.y);
  zDot = Eigen::Vector2f(msg->twist.twist.linear.x,msg->twist.twist.linear.y);

	if (firstMocap)
	{
		firstMocap = false;
		timeLast = timeNew;
		firstTime = timeNew;
    ROS_INFO("sheep found");
	}

  float dt = (timeNew - timeLast).toSec();
	timeLast = timeNew;

  if (!bearFound)
  {
    ROS_ERROR("sheep waiting for bear");
    return;
  }

	Eigen::RowVector4f Sz = getSz(z);

	zBuffer.push_back(z);
	SzBuffer.push_back(Sz);
  timeBuffer.push_back(timeNew);
  dtBuffer.push_back(dt);

  // ROS_INFO("sheep saved buffer");

	if (timeBuffer.size() >= 3)
	{
		// while the buffer is too big pop off the oldest data as long as it wont make
		// the time on the buffer too small. compare with the second oldest data to ensure
		// the buffer stays large enough
		while ((timeBuffer.at(timeBuffer.size()-1) - timeBuffer.at(1)).toSec() > Deltatth)
		{
      zBuffer.pop_front();
      SzBuffer.pop_front();
      timeBuffer.pop_front();
      dtBuffer.pop_front();
		}

    //if chased try to update the stack
    if (chased)
    {
      updateStack();
    }
	}

  // ROS_INFO("sheep saved checked buffer");

	// update theta and gamma
  Eigen::Matrix<float,4,2> thHatDot = kth*Gammath*(SzTDzSum - SzTSzSum*thHat);
  Eigen::Matrix<float,4,4> GammathDot = betath*Gammath - kth*Gammath*SzTSzSum*Gammath;
  thHat += (thHatDot*dt);
  Gammath += (GammathDot*dt);

  // std::cout << "\n thHat \n" << thHat << std::endl;
  // std::cout << "\n Gammath \n" << Gammath << std::endl;

  // Eigen::Matrix<float,4,6> thGam;
  // thGam.block(0,0,4,2) = thHat;
  // thGam.block(0,2,4,4) = Gammath;
  // thGam = integratethGam(thGam,dt);
  // thHat = thGam.block(0,0,4,2);
  // Gammath = thGam.block(0,2,4,4);

	// if the kill sheep has not been signaled then keep saving data otherwise save the data if it hasnt been saved yet
	if (!killSheep)
	{
    zData.push_back(z);
  	zDotData.push_back(zDot);
  	timeData.push_back((timeNew-firstTime).toSec());     // time data for the save
    int amIChased = 0;
		if (chased)
		{
			amIChased = 1;
		}
		chasedData.push_back(amIChased);
  	thHatData.push_back(thHat);
  	SzData.push_back(Sz);
	}
	else
	{
		if (!dataSaved && saveData)
		{
			saveRunData();
		}
		dataSaved = true;
	}
}

// save the data
void Sheep::saveRunData()
{
	std::ofstream sheepFile("/home/ncr/ncr_ws/src/single_agent_herding_adp/experiments/"+sheepName+".txt");
	if (sheepFile.is_open())
	{
		sheepFile << "time," << "z0," << "z1," << "zDot0," << "zDot1," << "chased," << "thHat00," << "thHat10,"
              << "thHat20," << "thHat30," << "thHat01," << "thHat11," << "thHat21," << "thHat31,"
              << "Sz0," << "Sz0," << "Sz1," << "Sz2," << "Sz3," << "\n";

		for (int i = 0; i < zData.size(); i++)
		{
			float timei = timeData.at(i);
			Eigen::Vector2f zi = zData.at(i);
      Eigen::Vector2f zDoti = zDotData.at(i);
			int chasedi = chasedData.at(i);
      Eigen::Matrix<float,4,2> thHati = thHatData.at(i);
      Eigen::RowVector4f Szi = SzData.at(i);
      sheepFile << timei << "," << zi(0) << "," << zi(1) << "," << zDoti(0) << "," << zDoti(1) << "," << chasedi << "," << thHati(0,0) << "," << thHati(1,0) << ","
                << thHati(2,0) << "," << thHati(3,0) << "," << thHati(0,1) << "," << thHati(1,1) << "," << thHati(2,1) << "," << thHati(3,1) << ","
                << Szi(0) << "," << Szi(1) << "," << Szi(2) << "," << Szi(3) << "," << "\n";
		}
		sheepFile.close();
	}
}

Eigen::Vector2f Sheep::getz()
{
  std::lock_guard<std::mutex> odomMutexGuard(odomMutex);
	return z;
}

Eigen::Vector2f Sheep::getzg()
{
	return zg;
}

bool Sheep::getfirstMocap()
{
  std::lock_guard<std::mutex> firstMutexGuard(firstMutex);
	return firstMocap;
}

void Sheep::setetaPose(Eigen::Vector3f etaPNew, Eigen::Vector4f etaQNew)
{
  std::lock_guard<std::mutex> bearMutexGuard(bearMutex);
	etaP = etaPNew;
	etaQ = etaQNew;

  if (!bearFound)
  {
    bearFound = true;
  }
}

void Sheep::setchased(bool chasedNew)
{
  std::lock_guard<std::mutex> chasedMutexGuard(chasedMutex);
	chased = chasedNew;
}

Eigen::MatrixXf Sheep::getthHat()
{
  std::lock_guard<std::mutex> thMutexGuard(thMutex);
	return thHat;
}

void Sheep::setkillSheep()
{
  std::lock_guard<std::mutex> killSheepMutexGuard(killSheepMutex);
	killSheep = true;
}

bool Sheep::getdataSaved()
{
  std::lock_guard<std::mutex> dataSaveMutexGuard(dataSaveMutex);
	return dataSaved;
}
