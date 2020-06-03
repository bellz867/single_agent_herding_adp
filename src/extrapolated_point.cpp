#include <extrapolated_point.h>

// initialize constructor
Bear::Bear()
{
	bearName = bearNameInit;
	odomSub = nh.subscribe(bearName+"/odomEKF",1,&Bear::odomCB,this);
	desOdomPub = nh.advertise<nav_msgs::Odometry>(bearName+"/desOdom",1);

	for (int jj = 0; jj < 4; jj++)
	{
		wall(jj) = wallInit.at(jj);
	}

	firstMocap = true;
	kd = kdInit;
	betac = betacInit;
	gamma1 = gamma1Init;
	Gammac = GammacInit*Eigen::Matrix<float,7,7>::Identity();
	WaHat = Wa*Eigen::Matrix<float,7,1>::Ones();
	WcHat = Wc*Eigen::Matrix<float,7,1>::Ones();
	knu = knuInit;
	kc1 = kc1Init;
	kc2 = kc2Init;
	ka1 = ka1Init;
	ka2 = ka2Init;
	height = heightInit;
	originRadius = originRadiusInit;
	saveData = saveDataInit;
	Ka = KaInit*Eigen::Matrix<float,7,7>::Identity();

	Qx = Eigen::Matrix<float,6,6>::Zero();
	R = Eigen::Matrix<float,4,4>::Zero();
	for (int jj = 0; jj < 6; jj++)
	{
		Qx(jj,jj) = QxInit.at(jj);

		if (jj < 4)
		{
			R(jj,jj) = RInit.at(jj);
		}
	}
	Eigen::JacobiSVD<Eigen::MatrixXf> svdR(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
	RI = svdR.solve(Eigen::Matrix<float,4,4>::Identity());

	G = Eigen::Matrix<float,6,4>::Zero();
	G.block(2,2,2,2) = Eigen::Matrix2f::Identity();
	G.block(4,0,2,2) = Eigen::Matrix2f::Identity();
	GT = G.transpose();
	RIGT = RI*GT;
	GR = G*RIGT;

	etad = Eigen::Vector2f::Zero();

	sheep = new Sheep(sheepNameInit,kdInit,betathInit,kthInit,GammathInit,DeltatthInit,
	                  Eigen::Vector2f(zgInit.at(0),zgInit.at(1)),MInit,lambda1Init,saveDataInit,
	                  height,basisCenters,basisVarInit,thInit,thMaxInit);

	//4 basis points located below quad in 4 cubic quadrants
  basisVar = basisVarInit;

  // front left ++
  basisMean(0,0) = basisCenters;
  basisMean(0,1) = basisCenters;
  basisMean(0,2) = -height;

  // back left -+
  basisMean(1,0) = -basisCenters;
  basisMean(1,1) = basisCenters;
  basisMean(1,2) = -height;

  // back right --
  basisMean(2,0) = -basisCenters;
  basisMean(2,1) = -basisCenters;
  basisMean(2,2) = -height;

  // front right +-
  basisMean(3,0) = -basisCenters;
  basisMean(3,1) = basisCenters;
  basisMean(3,2) = -height;

	etaP = Eigen::Vector3f::Zero();
	etaQ = Eigen::Vector4f::Zero();
	etaQ(0) = 1.0;
	etaQLast = etaQ;
	etaPF = Eigen::Vector3f(0.0,0.0,1.5);
	etaQF = etaQ;
	sheepFound = false;
  killBear = false;
	dataSaved = false;

	d = Eigen::Matrix<float,6,7>::Zero();
	d(0,0) = 1.0;
  for (int j = 1; j < 7; j++)
  {
    d(0,j) = -1.0/6.0;
  }

  for (int i = 1; i < 6; i++)
  {
		Eigen::Matrix<float,1,1> didim = d.block(0,i,i,1).transpose()*d.block(0,i,i,1);
    d(i,i) = sqrtf(1.0-didim(0,0));

    for (int j = i+1; j < 7; j++)
    {
			Eigen::Matrix<float,1,1> didjm = d.block(0,i,i,1).transpose()*d.block(0,j,i,1);
      d(i,j) = (-1.0/6.0-didjm(0,0))/d(i,i);
    }
  }
}

void Bear::wallCheck(Eigen::Vector2f& position, Eigen::Vector2f& velocity, Eigen::Vector4f wall, float dt)
{
	Eigen::Vector2f positionNew = position + velocity*dt;

	// check first position lower bound
	if ((positionNew(0) < wall(0)) && (velocity(0) < 0))
	{
		position(0) = wall(0);
		velocity(0) = 0;
	}
	// check first position upper bound
	if ((positionNew(0) > wall(1)) && (velocity(0) > 0))
	{
		position(0) = wall(1);
		velocity(0) = 0;
	}
	// check second position lower bound
	if ((positionNew(1) < wall(2)) && (velocity(1) < 0))
	{
		position(1) = wall(2);
		velocity(1) = 0;
	}
	// check second position upper bound
	if ((positionNew(1) > wall(3)) && (velocity(1) > 0))
	{
		position(1) = wall(3);
		velocity(1) = 0;
	}
}

// get the basis functions
Eigen::RowVector4f Bear::getSz(Eigen::Vector2f zi)
{
	Eigen::RowVector4f Szi = Eigen::RowVector4f::Zero();
	Eigen::Vector3f etazDiff = etaP-Eigen::Vector3f(zi(0),zi(1),0.0);
	Eigen::Vector3f X = rotatevec(etazDiff,etaQ);
	for (int jj = 0; jj < 4; jj++)
	{
    Eigen::Vector3f basisMeanjj = basisMean.block(jj,0,1,3).transpose();
		float XDiff = (X - basisMeanjj).transpose()*(X - basisMeanjj);
		Szi(jj) = 1.0/std::sqrt(2.0*M_PIl*basisVar)*std::exp(-1.0/(2.0*basisVar)*std::pow(XDiff,2));
	}
	return Szi;
}

// bear odom callback
void Bear::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	ros::Time timeNew = msg->header.stamp;
	etaP = Eigen::Vector3f(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
	etaQ = Eigen::Vector4f(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
	Eigen::Vector2f eta = etaP.segment(0,2);
	Eigen::Vector2f etaDot(msg->twist.twist.linear.x,msg->twist.twist.linear.y);

	// if this is the first mocap indicate and start time
	if (firstMocap)
	{
		timeLast = timeNew;
		firstTime = timeNew;
		etaQLast = etaQ;
		ROS_INFO("bear found");
		firstMocap = false;
	}

	if ((etaQLast + etaQ).norm() < (etaQLast - etaQ).norm())
	{
		etaQ *= -1.0;
	}
	etaQLast = etaQ;

	float dt = (timeNew - timeLast).toSec();
	timeLast = timeNew;

	if (dataSaved)
	{
		nav_msgs::Odometry uHatMsg;
		uHatMsg.header.stamp = msg->header.stamp;
		uHatMsg.header.frame_id = msg->header.frame_id;
		uHatMsg.child_frame_id = bearName+"_des";
		uHatMsg.pose.pose.position.x = etaPF(0);
		uHatMsg.pose.pose.position.y = etaPF(1);
		uHatMsg.pose.pose.position.z = etaPF(2);
		uHatMsg.pose.pose.orientation.w = etaQF(0);
		uHatMsg.pose.pose.orientation.x = etaQF(1);
		uHatMsg.pose.pose.orientation.y = etaQF(2);
		uHatMsg.pose.pose.orientation.z = etaQF(3);
		desOdomPub.publish(uHatMsg);

		//check if the sheep have saved the data/finished cleanly and if they have shutdown
		if (sheep->getdataSaved())
		{
			std::cout << bearName << " is victorious\n";
			odomSub.shutdown();
			desOdomPub.shutdown();
			ros::shutdown();
		}
		else
		{
			std::cout << "\n waiting for sheep to surrender \n";
		}
		return;
	}

	// update bear position for each sheep
	sheep->setetaPose(etaP,etaQ);

	// ROS_INFO("set sheep");

	// check if sheep found, if they are not then return
	if (!sheepFound)
	{
		if (sheep->getfirstMocap())
		{
			return;
		}
		ROS_INFO("bear found sheep");
		sheepFound = true;
	}

	//get acutal state and goal
	Eigen::Vector2f z = sheep->getz();
	Eigen::Vector2f zg = sheep->getzg();
	Eigen::Vector2f ez = z - zg;
	Eigen::Vector2f eeta = eta - etad;
	Eigen::Vector2f ed = etad - zg - kd*ez;

	// ROS_INFO("errors set");

	//generate random state from uniform around acutal state based on the size of the error
	Eigen::Vector2f zi = z + ez.norm()*Eigen::Vector2f::Random();
	Eigen::Vector2f ezi = zi - zg;
	Eigen::Vector2f edi = etad - zg - kd*ezi;

	// ROS_INFO("random errors set");

	//dynamics
	Eigen::Matrix<float,6,1> F1 = Eigen::Matrix<float,6,1>::Zero();
	Eigen::Matrix<float,6,1> F1i = Eigen::Matrix<float,6,1>::Zero();

	Eigen::Matrix<float,4,2> thHat = sheep->getthHat();
	Eigen::Vector2f fHatb = thHat.transpose()*getSz(z).transpose();
	Eigen::Vector2f fHatib = thHat.transpose()*getSz(zi).transpose();

	// ROS_INFO("got fHat");

	//rotate into world frame
	Eigen::Vector2f fHat = rotatevec(fHatb,etaQ);
	Eigen::Vector2f fHati = rotatevec(fHatib,etaQ);

	F1.segment(0,2) = (1.0/kd)*fHat;
	F1.segment(2,2) = -fHat;
	F1i.segment(0,2) = (1.0/kd)*fHati;
	F1i.segment(2,2) = -fHati;

	// ROS_INFO("got F1");

	//erros
	Eigen::Matrix<float,6,1> x;
	x.segment(0,2) = ez;
	x.segment(2,2) = ed;
	x.segment(4,2) = eeta;
	Eigen::Matrix<float,1,6> xT = x.transpose();
	float xTx = xT*x;
	float nu = knu*xTx/(1.0+xTx);
	Eigen::Matrix<float,1,6> gradnu = (2.0*knu/((1.0+xTx)*(1.0+xTx)))*xT;

	std::cout << "\n x \n" << x << std::endl;
	std::cout << "\n xTx \n" << xTx << std::endl;
	std::cout << "\n knu \n" << knu << std::endl;
	std::cout << "\n nu \n" << nu << std::endl;

	// ROS_INFO("got nu and grad nu");

	Eigen::Matrix<float,6,1> xi;
	xi.segment(0,2) = ezi;
	xi.segment(2,2) = edi;
	xi.segment(4,2) = eeta;
	Eigen::Matrix<float,1,6> xiT = xi.transpose();
	float xiTxi = xiT*xi;
	float nui = knu*xiTxi/(1.0+xiTxi);
	Eigen::Matrix<float,1,6> gradnui = (2.0*knu/((1.0+xiTxi)*(1.0+xiTxi)))*xiT;

	// ROS_INFO("got random nu and grad nu");

	//get input
	Eigen::Matrix<float,7,1> sigma,sigmai;
	Eigen::Matrix<float,7,6> gradsigma,gradsigmai;

	for (int ii = 0; ii < 7; ii++)
	{
		Eigen::Matrix<float,6,1> di = d.block(0,ii,6,1);
		sigma(ii) = xTx + nu*float(xT*di);
		sigmai(ii) = xiTxi + nui*float(xiT*di);
		gradsigma.block(ii,0,1,6) = xT + nu*di.transpose() + xT*(Eigen::Matrix<float,6,6>::Identity()+di*gradnu);
		gradsigmai.block(ii,0,1,6) = xiT + nui*di.transpose() + xiT*(Eigen::Matrix<float,6,6>::Identity()+di*gradnui);
	}

	std::cout << "\n sigma \n" << sigma << std::endl;
	std::cout << "\n gradsigma \n" << gradsigma << std::endl;

	// ROS_INFO("got sigma and grad sigma");

	Eigen::Matrix<float,6,7> gradsigmaT = gradsigma.transpose();
	Eigen::Matrix<float,6,7> gradsigmaiT = gradsigmai.transpose();

	// ROS_INFO("got sigma and grad sigma");

	Eigen::Matrix<float,4,1> muHat = -0.5*RIGT*gradsigmaT*WaHat;
	Eigen::Matrix<float,4,1> muiHat = -0.5*RIGT*gradsigmaiT*WaHat;

	Eigen::Vector2f uHat = muHat.segment(0,2) + muHat.segment(2,2);

	//update etad
	etad += (muHat.segment(2,2)*dt);

	//update weights
	Eigen::Matrix<float,1,6> gradVHat = WcHat.transpose()*gradsigma;
	Eigen::Matrix<float,1,6> gradViHat = WcHat.transpose()*gradsigmai;

	Eigen::Matrix<float,6,1> FGmu = F1 + G*muHat;
	Eigen::Matrix<float,6,1> FiGmui = F1i + G*muiHat;

	float delta = float(xT*Qx*x) + float(muHat.transpose()*R*muHat) + float(gradVHat*FGmu);
	float deltai = float(xiT*Qx*xi) + float(muiHat.transpose()*R*muiHat) + float(gradViHat*FiGmui);

	Eigen::Matrix<float,7,1> omega = gradsigma*FGmu;
	Eigen::Matrix<float,7,1> omegai = gradsigmai*FiGmui;

	Eigen::Matrix<float,1,7> omegaT = omega.transpose();
	Eigen::Matrix<float,1,7> omegaiT = omegai.transpose();

	float rho = 1.0+gamma1*float(omegaT*omega);
	float rho2 = rho*rho;
	float rhoi = 1.0+gamma1*float(omegaiT*omegai);
	float rhoi2 = rhoi*rhoi;

	Eigen::Matrix<float,7,1> WcHatDot = -Gammac*((kc1*delta/rho2)*omega + (kc2*deltai/rhoi2)*omegai);
	Eigen::Matrix<float,7,7> GammacDot = betac*Gammac - Gammac*((kc1/rho2)*omega*omegaT + (kc2/rhoi2)*omegai*omegaiT)*Gammac;
	Eigen::Matrix<float,7,7> GsigmaT = (gradsigma*GR*gradsigmaT).transpose();
	Eigen::Matrix<float,7,7> GsigmaiT = (gradsigmai*GR*gradsigmaiT).transpose();
	Eigen::Matrix<float,7,1> WaHatDot = Ka*(-ka1*(WaHat - WcHat) - ka2*WaHat + (kc1/(4.0*rho2))*GsigmaT*WaHat*omegaT*WcHat + (kc2/(4.0*rhoi2))*GsigmaiT*WaHat*omegaiT*WcHat);

	WcHat += (WcHatDot*dt);
	Gammac += (GammacDot*dt);
	WaHat += (WaHatDot*dt);

	std::cout << "\n WcHat \n" << WcHat << std::endl;
	std::cout << "\n Gammac \n" << Gammac << std::endl;
	std::cout << "\n WaHat \n" << WaHat << std::endl;
	std::cout << "\n omega \n" << omega << std::endl;
	std::cout << "\n delta \n" << delta << std::endl;

	Eigen::Vector3f bearx = rotatevec(Eigen::Vector3f(1.0,0.0,0.0),etaQ);
	Eigen::Vector2f zdiffeta = z - eta;
	float bearxdotzdiffeta = (bearx(0)*zdiffeta(0) + bearx(1)*zdiffeta(1))/(0.001+zdiffeta.norm());
	Eigen::Vector3f beraxcrosszdiffeta = getss(bearx)*Eigen::Vector3f(zdiffeta(0),zdiffeta(1),0.0);
	float headingd = acos(bearxdotzdiffeta)*std::copysign(1.0,beraxcrosszdiffeta(2));
	Eigen::Vector4f etaQe = Eigen::Vector4f(std::cos(0.5*headingd), 0.0, 0.0, std::sin(0.5*headingd));
	etaQe /= etaQe.norm();


	// float headingd = std::atan2(z(1)-eta(1),z(0)-eta(0));
	// Eigen::Vector4f etaQd = Eigen::Vector4f(std::cos(0.5*headingd), 0.0, 0.0, std::sin(0.5*headingd));
	// Eigen::Vector4f etaQe = getqMat(getqInv(etaQd))*etaQ;
	// etaQe /= etaQe.norm();

	//build and publish odom message
	nav_msgs::Odometry uHatMsg;
	uHatMsg.header.stamp = msg->header.stamp;
	uHatMsg.header.frame_id = msg->header.frame_id;
	uHatMsg.child_frame_id = bearName+"_des";
	if (killBear || (ez.norm() < originRadius))
	{
		etaPF(0) = etaP(0);
		etaPF(1) = etaP(1);
		etaQF = etaQ;

		uHatMsg.pose.pose.position.x = etaPF(0);
		uHatMsg.pose.pose.position.y = etaPF(1);
		uHatMsg.pose.pose.position.z = etaPF(2);
		uHatMsg.pose.pose.orientation.w = etaQF(0);
		uHatMsg.pose.pose.orientation.x = etaQF(1);
		uHatMsg.pose.pose.orientation.y = etaQF(2);
		uHatMsg.pose.pose.orientation.z = etaQF(3);
	}
	else
	{
		Eigen::Vector2f etaTemp = eta;
		Eigen::Vector2f etaDotTemp = uHat;
		wallCheck(etaTemp,etaDotTemp,wall,dt);

		//rotate into body frame then estiamte dynaimcs for compensation
		Eigen::Vector2f etaDotTempBody = rotatevec(etaDotTemp,getqInv(etaQ));
		Eigen::Vector2f etaDotBody = rotatevec(etaDot,getqInv(etaQ));
		Eigen::Vector2f etaDotOutputBody = dynamicsEstimator.update(etaDotTempBody,etaDotBody,dt);
		Eigen::Vector2f etaDotOutput = rotatevec(etaDotOutputBody,etaQ);

		uHatMsg.pose.pose.position.x = etaTemp(0);
		uHatMsg.pose.pose.position.y = etaTemp(1);
		uHatMsg.pose.pose.position.z = height;
		uHatMsg.pose.pose.orientation.w = etaQ(0);
		uHatMsg.pose.pose.orientation.x = etaQ(1);
		uHatMsg.pose.pose.orientation.y = etaQ(2);
		uHatMsg.pose.pose.orientation.z = etaQ(3);

		uHatMsg.twist.twist.linear.x = etaDotOutput(0);
		uHatMsg.twist.twist.linear.y = etaDotOutput(1);
		uHatMsg.twist.twist.angular.z = 2.0*etaQe(3);
	}
	desOdomPub.publish(uHatMsg);


	if (!dataSaved)
	{
		timeData.push_back((timeNew - firstTime).toSec());
		etaData.push_back(eta);
		etaDotData.push_back(etaDot);
		uHatData.push_back(uHat);
		WcHatData.push_back(WcHat);
		WaHatData.push_back(WaHat);

		std::cout << bearName << " time " << (timeNew - firstTime).toSec() << std::endl;

		if (ez.norm() < originRadius)
		{
			shutdownNode();
		}
	}
}

// sends a kill command to all the sheep and bear and saves the data and kills the node
void Bear::shutdownNode()
{
	killBear = true;
	sheep->setkillSheep();
	if (saveData && !dataSaved)
	{
		std::ofstream bearFile("/home/ncr/ncr_ws/src/single_agent_herding_adp/experiments/"+bearName+".txt");
		if (bearFile.is_open())
		{
			bearFile << "time," << "eta0," << "eta1," << "etaDot0," << "etaDot1," << "uHat0," << "uHat1,"
			<< "WcHat0," << "WcHat1," << "WcHat2," << "WcHat3," << "WcHat4," << "WcHat5," << "WcHat6,"
			<< "WaHat0," << "WaHat1," << "WaHat2," << "WaHat3," << "WaHat4," << "WaHat5," << "WaHat6," << "\n";
			for (int i = 0; i < timeData.size(); i++)
			{
				float timei = timeData.at(i);
				Eigen::Vector2f etai = etaData.at(i);
				Eigen::Vector2f etaDoti = etaDotData.at(i);
				Eigen::Vector2f uHati = uHatData.at(i);
				Eigen::Matrix<float,7,1> WcHati = WcHatData.at(i);
				Eigen::Matrix<float,7,1> WaHati = WaHatData.at(i);

				bearFile << timei << "," << etai(0) << "," << etai(1) << "," << etaDoti(0) << "," << etaDoti(1) << "," << uHati(0) << "," << uHati(1) << ","
				<< WcHati(0,0) << "," << WcHati(1,0) << "," << WcHati(2,0) << "," << WcHati(3,0) << "," << WcHati(4,0) << "," << WcHati(5,0) << "," << WcHati(6,0) << ","
				<< WaHati(0,0) << "," << WaHati(1,0) << "," << WaHati(2,0) << "," << WaHati(3,0) << "," << WaHati(4,0) << "," << WaHati(5,0) << "," << WaHati(6,0) << "," << "\n";
			}
			bearFile.close();
		}
	}
	dataSaved = true;
}
