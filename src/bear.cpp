#include <bear.h>

// initialize constructor
Bear::Bear(std::string bearNameInit, std::string sheepNameInit, float kdInit, float betathInit,
		float kthInit, float GammathInit, float betacInit, float gamma1Init, float GammacInit, int NInit, float kc1Init,
		float kc2Init, float ka1Init, float ka2Init, float knuInit, float heightInit, float originRadiusInit, bool saveDataInit,
		float lambda1Init, float DeltatthInit, std::vector<float> zgInit, int MInit, float basisCenters,
		float basisVarInit, float thInit, float thMaxInit, float KaInit, float keta1Init, float keta2Init, std::vector<float> QxInit,
		std::vector<float> RInit, float PMagInit, float PVarInit, std::vector<float> wallInit, float Wa, float Wc,
		float kpl, float kdl, float gammal)
{
	bearName = bearNameInit;
	odomSub = nh.subscribe(bearName+"/odomEKF",1,&Bear::odomCB,this);
	desOdomPub = nh.advertise<nav_msgs::Odometry>(bearName+"/desOdom",1);
	dynamicsEstimator.initialize(kpl,kdl,gammal);

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
	N = NInit;
	knu = knuInit;
	kc1 = kc1Init;
	kc2 = kc2Init;
	ka1 = ka1Init;
	ka2 = ka2Init;
	height = heightInit;
	originRadius = originRadiusInit;
	saveData = saveDataInit;
	Ka = KaInit*Eigen::Matrix<float,7,7>::Identity();
	keta1 = keta1Init;
	keta2 = keta2Init;

	PMag = PMagInit;
	PVar = PVarInit;
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
	G.block(2,2,2,2) = keta2*Eigen::Matrix2f::Identity();
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
  basisMean(3,0) = basisCenters;
  basisMean(3,1) = -basisCenters;
  basisMean(3,2) = -height;

	etaP = Eigen::Vector3f::Zero();
	etaQ = Eigen::Vector4f::Zero();
	etaQ(0) = 1.0;
	etaQLast = etaQ;
	etaPF = Eigen::Vector3f(0.0,0.0,height);
	etaQF = etaQ;
	sheepFound = false;
  killBear = false;
	dataSaved = false;
	firstx = true;
	firstetad = true;

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

Bear::~Bear()
{
	std::lock_guard<std::mutex> destroyBearGuard(destroyBearMutex);
	if (!dataSaved && saveData)
	{
		shutdownNode();
	}
	delete sheep;
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
	Eigen::Vector3f etazDiff = Eigen::Vector3f(zi(0),zi(1),0.0) - etaP;
	Eigen::Vector3f X = rotatevec(etazDiff,getqInv(etaQ));
	for (int jj = 0; jj < 4; jj++)
	{
    Eigen::Vector3f basisMeanjj = basisMean.block(jj,0,1,3).transpose();
		float XDiff = (X - basisMeanjj).transpose()*(X - basisMeanjj);
		Szi(jj) = 1.0/std::sqrt(2.0*M_PIl*basisVar)*std::exp(-1.0/(2.0*basisVar)*std::pow(XDiff,2));
	}
	return Szi;
}

float Bear::getP(Eigen::Vector2f z, Eigen::Vector2f eta, Eigen::Vector2f zg)
{
	// Eigen::Vector2f diff = eta - z;
	// float diffdiff = diff(0)*diff(0) + diff(1)*diff(1);
	// float P = PMag*1.0/std::sqrt(2.0*M_PIl*PVar)*std::exp(-1.0/(2.0*PVar)*std::pow(diffdiff,2));
	// float P = PMag*std::exp((-1.0/PVar)*std::pow(diffdiff,2));
	Eigen::Vector2f diff1 = eta - zg;
	Eigen::Vector2f diff2 = z - zg;
	float diff1Norm = sqrtf(diff1(0)*diff1(0) + diff1(1)*diff1(1));
	float diff2Norm = sqrtf(diff2(0)*diff2(0) + diff2(1)*diff2(1));
	float diffdiff = diff1(0)*diff2(0) + diff1(1)*diff2(1);
	float thetadiff = acos(diffdiff/(0.001+diff1Norm*diff2Norm));

	float P = PMag*thetadiff;
	return P;
}

// bear odom callback
void Bear::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
	ros::Time timeNew = msg->header.stamp;
	etaP = Eigen::Vector3f(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
	etaQ = Eigen::Vector4f(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
	Eigen::Vector2f eta = etaP.segment(0,2);
	Eigen::Vector2f etaDot(msg->twist.twist.linear.x,msg->twist.twist.linear.y);
	Eigen::Vector2f etaDotVar(msg->twist.covariance.at(0),msg->twist.covariance.at(7));

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

	std::cout << "\n**********\ntime from start " << (timeNew - firstTime).toSec() << std::endl;

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

	if (firstetad)
	{
		etad = eta;
	}
	// Eigen::Vector2f eeta = eta - etad;
	// Eigen::Vector2f ed = etad - zg - kd*ez;
	Eigen::Vector2f eeta = eta - etad;
	Eigen::Vector2f ed = etad - zg - kd*ez;

	// std::cout << "\n z \n" << z << std::endl;
	// std::cout << "\n zi \n" << zi << std::endl;

	//dynamics
	Eigen::Matrix<float,4,2> thHat = sheep->getthHat();

	std::cout << "\n thHat \n" << thHat << std::endl;
	Eigen::Matrix<float,6,1> F1 = Eigen::Matrix<float,6,1>::Zero();
	Eigen::Matrix<float,6,1> F2 = Eigen::Matrix<float,6,1>::Zero();
	Eigen::Vector2f fHatb = thHat.transpose()*getSz(z).transpose();

	//rotate into world frame
	Eigen::Vector2f fHat = rotatevec(fHatb,etaQ);

	F1.segment(0,2) = (1.0/kd)*fHat;
	F1.segment(2,2) = -fHat;
	F2.segment(2,2) = -keta1*etad;
	F2.segment(4,2) = keta1*etad;

	// ROS_INFO("got F1");
	std::cout << "\n fHatb \n" << fHatb << std::endl;
	std::cout << "\n fHat \n" << fHat << std::endl;
	std::cout << "\n z \n" << z  << std::endl;
	std::cout << "\n zg \n" << zg << std::endl;
	std::cout << "\n ez \n" << ez << std::endl;
	std::cout << "\n eta \n" << eta << std::endl;
	std::cout << "\n etad \n" << etad << std::endl;
	std::cout << "\n eeta \n" << eeta << std::endl;
	std::cout << "\n ed \n" << ed << std::endl;

	//erros
	Eigen::Matrix<float,6,1> x;
	x.segment(0,2) = ez;
	x.segment(2,2) = ed;
	x.segment(4,2) = eeta;

	if (firstx)
	{
		x0 = x;
		firstx = false;
	}

	Eigen::Matrix<float,6,1> s = x;
	if (x0.norm() > 0.0001)
	{
		s /= x0.norm();
	}

	std::cout << "\n x \n" << x << std::endl;
	std::cout << "\n s \n" << s << std::endl;

	Eigen::Matrix<float,1,6> sT = s.transpose();
	float sTs = sT*s;
	float nu = knu*sTs/(1.0+sTs);
	Eigen::Matrix<float,1,6> gradnu = (2.0*knu/((1.0+sTs)*(1.0+sTs)))*sT;

	//get input
	Eigen::Matrix<float,7,1> sigma;
	Eigen::Matrix<float,7,6> gradsigma;

	// //get input
	// Eigen::Matrix<float,7,1> sigmaexp;
	// Eigen::Matrix<float,7,6> gradsigmaexp;

	for (int ii = 0; ii < 7; ii++)
	{
		Eigen::Matrix<float,6,1> di = d.block(0,ii,6,1);
		// sigma(ii) = sTs + nu*float(sT*di);
		// sigmaexp(ii) = exp(sigma(ii))-1.0;
		// gradsigma.block(ii,0,1,6) = sT + nu*di.transpose() + sT*(Eigen::Matrix<float,6,6>::Identity()+di*gradnu);
		// gradsigmaexp.block(ii,0,1,6) = exp(sigma(ii))*gradsigma.block(ii,0,1,6);
		float sigmaii = sTs + nu*float(sT*di);
		sigma(ii) = exp(sigmaii)-1.0;
		Eigen::Matrix<float,1,6> gradsigmaii = sT + nu*di.transpose() + sT*(Eigen::Matrix<float,6,6>::Identity()+di*gradnu);
		gradsigma.block(ii,0,1,6) = exp(sigmaii)*gradsigmaii;
	}

	// Eigen::Matrix<float,1,6> xT = x.transpose();
	// float xTx = xT*x;
	// float nu = knu*xTx/(1.0+xTx);
	// Eigen::Matrix<float,1,6> gradnu = (2.0*knu/((1.0+xTx)*(1.0+xTx)))*xT;
	//
	// //get input
	// Eigen::Matrix<float,7,1> sigma;
	// Eigen::Matrix<float,7,6> gradsigma;
	//
	// for (int ii = 0; ii < 7; ii++)
	// {
	// 	Eigen::Matrix<float,6,1> di = d.block(0,ii,6,1);
	// 	sigma(ii) = xTx + nu*float(xT*di);
	// 	gradsigma.block(ii,0,1,6) = xT + nu*di.transpose() + xT*(Eigen::Matrix<float,6,6>::Identity()+di*gradnu);
	// }

	std::cout << "\n nu " << nu << std::endl;
	std::cout << "\n nu*cos(M_PIl/4.0) " << nu*cos(M_PIl/4.0) << std::endl;
	std::cout << "\n sigma \n" << sigma << std::endl;
	std::cout << "\n gradsigma \n" << gradsigma << std::endl;

	Eigen::Matrix<float,6,7> gradsigmaT = gradsigma.transpose();

	Eigen::Matrix<float,4,1> muHat = -0.5*RIGT*gradsigmaT*WaHat;

	Eigen::Vector2f uHat = muHat.segment(0,2) - keta1*etad + keta2*muHat.segment(2,2);

	//update etad
	etad += ((-keta1*etad + keta2*muHat.segment(2,2))*dt);

	//update weights
	Eigen::Matrix<float,1,6> gradVHat = WcHat.transpose()*gradsigma;

	Eigen::Matrix<float,6,1> FGmu = F1 - F2 + G*muHat;

	float P = getP(z,eta,zg);

	std::cout << "\n P " << P << std::endl;

	float delta = float(x.transpose()*Qx*x) + P + float(muHat.transpose()*R*muHat) + float(gradVHat*FGmu);
	// float delta = float(sT*Qx*s) + P + float(muHat.transpose()*R*muHat) + float(gradVHat*FGmu);

	Eigen::Matrix<float,7,1> omega = gradsigma*FGmu;

	Eigen::Matrix<float,1,7> omegaT = omega.transpose();

	float rho = 1.0+gamma1*float(omegaT*omega);
	float rho2 = rho*rho;

	Eigen::Matrix<float,7,1> WcSum = (kc1*delta/rho2)*omega;
	Eigen::Matrix<float,7,7> GammacSum = (kc1/rho2)*omega*omegaT;
	Eigen::Matrix<float,7,7> GsigmaT = (gradsigma*GR*gradsigmaT).transpose();
	Eigen::Matrix<float,7,1> WaSum = (kc1/(4.0*rho2))*GsigmaT*WaHat*omegaT*WcHat;
	for (int ii = 0; ii < N; ii++)
	{
		// //generate random state from uniform around acutal state based on the size of the largest square in nu
		// Eigen::Vector2f zi = z + nu*cos(M_PIl/4.0)*Eigen::Vector2f::Random();
		// Eigen::Vector2f ezi = zi - zg;
		// Eigen::Vector2f edi = etad - zg - kd*ezi;
		//
		// //dynamics
		// Eigen::Matrix<float,6,1> F1i = Eigen::Matrix<float,6,1>::Zero();
		// Eigen::Vector2f fHatib = thHat.transpose()*getSz(zi).transpose();
		//
		// //rotate into world
		// Eigen::Vector2f fHati = rotatevec(fHatib,etaQ);
		// F1i.segment(0,2) = (1.0/kd)*fHati;
		// F1i.segment(2,2) = -fHati;
		//
		// //errors
		// Eigen::Matrix<float,6,1> xi;
		// xi.segment(0,2) = ezi;
		// xi.segment(2,2) = edi;
		// xi.segment(4,2) = eeta;

		//generate random state from uniform around acutal state based on the size of the largest square in nu
		Eigen::Matrix<float,6,1> xi = x + nu*cos(M_PIl/4.0)*Eigen::Matrix<float,6,1>::Random();

		Eigen::Matrix<float,6,1> si = xi;
		if (x0.norm() > 0.0001)
		{
			si /= x0.norm();
		}

		//dynamics
		Eigen::Vector2f zi = xi.segment(0,2) + zg;
		Eigen::Matrix<float,6,1> F1i = Eigen::Matrix<float,6,1>::Zero();
		Eigen::Vector2f fHatib = thHat.transpose()*getSz(zi).transpose();

		//rotate into world
		Eigen::Vector2f fHati = rotatevec(fHatib,etaQ);
		F1i.segment(0,2) = (1.0/kd)*fHati;
		F1i.segment(2,2) = -fHati;

		Eigen::Vector2f etadi = xi.segment(2,2) - kd*xi.segment(0,2);
		Eigen::Matrix<float,6,1> F2i = Eigen::Matrix<float,6,1>::Zero();
		F2i.segment(2,2) = -keta1*etadi;
		F2i.segment(4,2) = keta1*etadi;

		// Eigen::Matrix<float,1,6> xiT = xi.transpose();
		// float xiTxi = xiT*xi;
		// float nui = knu*xiTxi/(1.0+xiTxi);
		// Eigen::Matrix<float,1,6> gradnui = (2.0*knu/((1.0+xiTxi)*(1.0+xiTxi)))*xiT;
		//
		// //get input
		// Eigen::Matrix<float,7,1> sigmai;
		// Eigen::Matrix<float,7,6> gradsigmai;
		//
		// for (int ii = 0; ii < 7; ii++)
		// {
		// 	Eigen::Matrix<float,6,1> di = d.block(0,ii,6,1);
		// 	sigmai(ii) = xiTxi + nui*float(xiT*di);
		// 	gradsigmai.block(ii,0,1,6) = xiT + nui*di.transpose() + xiT*(Eigen::Matrix<float,6,6>::Identity()+di*gradnui);
		// }

		Eigen::Matrix<float,1,6> siT = si.transpose();
		float siTsi = siT*si;
		float nui = knu*siTsi/(1.0+siTsi);
		Eigen::Matrix<float,1,6> gradnui = (2.0*knu/((1.0+siTsi)*(1.0+siTsi)))*siT;

		//get input
		Eigen::Matrix<float,7,1> sigmai;
		Eigen::Matrix<float,7,6> gradsigmai;

		// //get input
		// Eigen::Matrix<float,7,1> sigmaexpi;
		// Eigen::Matrix<float,7,6> gradsigmaexpi;

		for (int ii = 0; ii < 7; ii++)
		{
			Eigen::Matrix<float,6,1> di = d.block(0,ii,6,1);
			// sigmai(ii) = siTsi + nui*float(siT*di);
			// sigmaexp(ii) = exp(sigmai(ii))-1.0;
			// gradsigmai.block(ii,0,1,6) = siT + nui*di.transpose() + siT*(Eigen::Matrix<float,6,6>::Identity()+di*gradnui);
			// gradsigmaexp.block(ii,0,1,6) = exp(sigmai(ii))*gradsigmai.block(ii,0,1,6);
			float sigmaiii = siTsi + nui*float(siT*di);
			sigmai(ii) = exp(sigmaiii)-1.0;
			Eigen::Matrix<float,1,6> gradsigmaiii = siT + nui*di.transpose() + siT*(Eigen::Matrix<float,6,6>::Identity()+di*gradnui);
			gradsigmai.block(ii,0,1,6) = exp(sigmaiii)*gradsigmaiii;
		}

		Eigen::Matrix<float,6,7> gradsigmaiT = gradsigmai.transpose();
		Eigen::Matrix<float,4,1> muiHat = -0.5*RIGT*gradsigmaiT*WaHat;

		//get bellman and omega
		Eigen::Matrix<float,1,6> gradViHat = WcHat.transpose()*gradsigmai;
		Eigen::Matrix<float,6,1> FiGmui = F1i - F2i + G*muiHat;
		float Pi = getP(zi,eta,zg);
		float deltai = float(xi.transpose()*Qx*xi) + Pi + float(muiHat.transpose()*R*muiHat) + float(gradViHat*FiGmui);
		// float deltai = float(siT*Qx*si) + Pi + float(muiHat.transpose()*R*muiHat) + float(gradViHat*FiGmui);
		Eigen::Matrix<float,7,1> omegai = gradsigmai*FiGmui;
		Eigen::Matrix<float,1,7> omegaiT = omegai.transpose();
		float rhoi = 1.0+gamma1*float(omegaiT*omegai);
		float rhoi2 = rhoi*rhoi;

		//get sum terms
		Eigen::Matrix<float,7,7> GsigmaiT = (gradsigmai*GR*gradsigmaiT).transpose();
		WcSum += ((1.0/N)*(kc2*deltai/rhoi2)*omegai);
		GammacSum += ((1.0/N)*(kc2/rhoi2)*omegai*omegaiT);
		WaSum += ((1.0/N)*(kc2/(4.0*rhoi2))*GsigmaiT*WaHat*omegaiT*WcHat);

		// std::cout << "\n zix " << zi(0) << " ziy " << zi(1) << std::endl;
		// std::cout << "\n Vstari " << (WcHat.transpose()*sigmai) << std::endl;
	}
	std::cout << "\n zx " << z(0) << " zy " << z(1) << std::endl;
	std::cout << "\n Vstar " << (WcHat.transpose()*sigma) << std::endl;

	Eigen::Matrix<float,7,1> WcHatDot = -Gammac*WcSum;
	Eigen::Matrix<float,7,7> GammacDot = betac*Gammac - Gammac*GammacSum*Gammac;
	Eigen::Matrix<float,7,1> WaHatDot = Ka*(-ka1*(WaHat - WcHat) - ka2*WaHat + WaSum);

	WcHat += (WcHatDot*dt);
	Gammac += (GammacDot*dt);
	WaHat += (WaHatDot*dt);

	std::cout << "\n WcHat \n" << WcHat << std::endl;
	// std::cout << "\n Gammac \n" << Gammac << std::endl;
	std::cout << "\n WaHat \n" << WaHat << std::endl;
	std::cout << "\n omega \n" << omega << std::endl;
	std::cout << "\n delta \n" << delta << std::endl;
	std::cout << "\n muetaHat \n" << muHat.segment(0,2) << std::endl;
	std::cout << "\n mudHat \n" << muHat.segment(2,2) << std::endl;
	std::cout << "\n uHat \n" << uHat << std::endl;
	std::cout << "\n etaDot \n" << etaDot << std::endl;

	// Eigen::Vector3f bearx = rotatevec(Eigen::Vector3f(1.0,0.0,0.0),etaQ);
	// Eigen::Vector2f zdiffeta = z - eta;
	// float bearxdotzdiffeta = (bearx(0)*zdiffeta(0) + bearx(1)*zdiffeta(1))/(0.001+zdiffeta.norm());
	// Eigen::Vector3f beraxcrosszdiffeta = getss(bearx)*Eigen::Vector3f(zdiffeta(0),zdiffeta(1),0.0);
	// float headingd = acos(bearxdotzdiffeta)*std::copysign(1.0,beraxcrosszdiffeta(2));
	// Eigen::Vector4f etaQe = Eigen::Vector4f(std::cos(0.5*headingd), 0.0, 0.0, std::sin(0.5*headingd));
	// etaQe /= etaQe.norm();

	Eigen::Vector4f etaQd = Eigen::Vector4f(0.0, 0.0, 0.0, 1.0);

	if ((etaQ + etaQd).norm() < (etaQ - etaQd).norm())
	{
		etaQd *= -1.0;
	}

	Eigen::Vector4f etaQe = etaQd-etaQ;
	Eigen::Vector3f etawe = 0.5*B(etaQ).transpose()*etaQe;


	// float headingd = std::atan2(z(1)-eta(1),z(0)-eta(0));
	// Eigen::Vector4f etaQd = Eigen::Vector4f(std::cos(0.5*headingd), 0.0, 0.0, std::sin(0.5*headingd));
	// Eigen::Vector4f etaQe = getqMat(getqInv(etaQd))*etaQ;
	// etaQe /= etaQe.norm();

	//build and publish odom message
	nav_msgs::Odometry uHatMsg;
	uHatMsg.header.stamp = msg->header.stamp;
	uHatMsg.header.frame_id = msg->header.frame_id;
	uHatMsg.child_frame_id = bearName+"_des";
	Eigen::Vector2f uHatWall = Eigen::Vector2f::Zero();
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
		//rotate into body frame then estiamte dynaimcs for compensation
		Eigen::Vector2f uHatBody = rotatevec(uHat,getqInv(etaQ));
		Eigen::Vector2f etaDotBody = rotatevec(etaDot,getqInv(etaQ));

		std::cout << "\n uHatBody \n" << uHatBody << std::endl;
		std::cout << "\n etaDotBody \n" << etaDotBody << std::endl;

		Eigen::Vector2f uHatOutBody = dynamicsEstimator.update(uHatBody,etaDotBody,etaDotVar,dt);
		std::cout << "\n uHatOutBody \n" << uHatOutBody << std::endl;
		uHatWall = rotatevec(uHatOutBody,etaQ);
		std::cout << "\n uHatWall \n" << uHatWall << std::endl;

		Eigen::Vector2f etaWall = eta;
		wallCheck(etaWall,uHatWall,wall,dt);

		uHatMsg.pose.pose.position.x = etaWall(0);
		uHatMsg.pose.pose.position.y = etaWall(1);
		uHatMsg.pose.pose.position.z = height;
		uHatMsg.pose.pose.orientation.w = etaQ(0);
		uHatMsg.pose.pose.orientation.x = etaQ(1);
		uHatMsg.pose.pose.orientation.y = etaQ(2);
		uHatMsg.pose.pose.orientation.z = etaQ(3);

		uHatMsg.twist.twist.linear.x = uHatWall(0);
		uHatMsg.twist.twist.linear.y = uHatWall(1);
		uHatMsg.twist.twist.angular.z = etawe(2);
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
		uWallData.push_back(uHatWall);
		mudData.push_back(muHat.segment(2,2));
		muhatData.push_back(muHat.segment(0,2));
		xData.push_back(x);

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
			<< "WaHat0," << "WaHat1," << "WaHat2," << "WaHat3," << "WaHat4," << "WaHat5," << "WaHat6,"
			<< "uWall0," << "uWall1," << "mud0," << "mud1," << "muhat0," << "muhat1,"
			<< "x0," << "x1," << "x2," << "x3," << "x4," << "x5," << "\n";
			for (int i = 0; i < timeData.size(); i++)
			{
				float timei = timeData.at(i);
				Eigen::Vector2f etai = etaData.at(i);
				Eigen::Vector2f etaDoti = etaDotData.at(i);
				Eigen::Vector2f uHati = uHatData.at(i);
				Eigen::Matrix<float,7,1> WcHati = WcHatData.at(i);
				Eigen::Matrix<float,7,1> WaHati = WaHatData.at(i);
				Eigen::Vector2f uWalli = uWallData.at(i);
				Eigen::Vector2f mudi = mudData.at(i);
				Eigen::Vector2f muhati = muhatData.at(i);
				Eigen::Matrix<float,6,1> xi = xData.at(i);

				bearFile << timei << "," << etai(0) << "," << etai(1) << "," << etaDoti(0) << "," << etaDoti(1) << "," << uHati(0) << "," << uHati(1) << ","
				<< WcHati(0,0) << "," << WcHati(1,0) << "," << WcHati(2,0) << "," << WcHati(3,0) << "," << WcHati(4,0) << "," << WcHati(5,0) << "," << WcHati(6,0) << ","
				<< WaHati(0,0) << "," << WaHati(1,0) << "," << WaHati(2,0) << "," << WaHati(3,0) << "," << WaHati(4,0) << "," << WaHati(5,0) << "," << WaHati(6,0) << ","
				<< uWalli(0) << "," << uWalli(1) << "," << mudi(0) << "," << mudi(1) << "," << muhati(0) << "," << muhati(1) << ","
				<< xi(0,0) << "," << xi(1,0) << "," << xi(2,0) << "," << xi(3,0) << "," << xi(4,0) << "," << xi(5,0) << "," << "\n";
			}
			bearFile.close();
		}
	}
	dataSaved = true;
}
