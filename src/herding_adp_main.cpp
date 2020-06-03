#include <bear.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "herding_adp");

	//handle to launch file parameters
	ros::NodeHandle nhp("~");
	std::string bearName;
	nhp.param<std::string>("bearName",bearName,"bebop");
	std::vector<std::string> sheepName;
	nhp.param<std::vector<std::string>>("sheepName",sheepName,{"sheep"});
	float kd, betath, kth, Gammath, betac, gamma1, Gammac, kc1, kc2, ka1, ka2, knu, Ka, height, originRadius, Deltatth, lambda1, basisCenters, basisVar, th, thMax, Wa, Wc;
	int N,M;
	bool saveData;
  std::vector<float> wall;
	std::vector<float> zg;
  std::vector<float> Qx,R;
  float PMag,PVar;
  float kpl,kdl,gammal;
  float keta1,keta2;
	nhp.param<float>("kd",kd,1.0);
	nhp.param<float>("betath",betath,1.0);
  nhp.param<float>("kth",kth,1.0);
	nhp.param<float>("Gammath",Gammath,1.0);
	nhp.param<float>("betac",betac,1.0);
	nhp.param<float>("gamma1",gamma1,1.0);
  nhp.param<float>("Gammac",Gammac,1.0);
  nhp.param<int>("N",N,1);
  nhp.param<float>("Wa",Wa,1.0);
  nhp.param<float>("Wc",Wc,1.0);
	nhp.param<float>("kc1",kc1,1.0);
  nhp.param<float>("kc2",kc2,1.0);
  nhp.param<float>("ka1",ka1,1.0);
  nhp.param<float>("ka2",ka2,1.0);
  nhp.param<float>("knu",knu,1.0);
  nhp.param<float>("Ka",Ka,1.0);
  nhp.param<float>("keta1",keta1,0.0);
  nhp.param<float>("keta2",keta2,1.0);
  nhp.param<std::vector<float>>("Qx",Qx,{1.0,1.0,1.0,1.0,1.0,1.0});
  nhp.param<std::vector<float>>("R",R,{1.0,1.0,1.0,1.0});
  nhp.param<float>("PMag",PMag,1.0);
  nhp.param<float>("PVar",PVar,1.0);
  nhp.param<float>("height",height,1.0);
  nhp.param<float>("originRadius",originRadius,0.5);
  nhp.param<std::vector<float>>("zg",zg,{0.0,0.0});
  nhp.param<bool>("saveData",saveData,false);
  nhp.param<float>("lambda1",lambda1,0.001);
	nhp.param<float>("Deltatth",Deltatth,2.0);
	nhp.param<int>("M",M,10);
  nhp.param<float>("basisCenters",basisCenters,0.1);
  nhp.param<float>("basisVar",basisVar,0.1);
  nhp.param<float>("th",th,0.1);
  nhp.param<float>("thMax",thMax,0.1);
	nhp.param<std::vector<float>>("wall",wall,{-4.0,4.0,-1.5,1.5});
  nhp.param<float>("kpl",kpl,0.1);
  nhp.param<float>("kdl",kdl,0.1);
  nhp.param<float>("gammal",gammal,0.1);

	Bear bear(bearName, sheepName.at(0), kd, betath, kth, Gammath, betac, gamma1, Gammac, N, kc1,
	          kc2, ka1, ka2, knu, height, originRadius, saveData, lambda1, Deltatth, zg, M, basisCenters,
            basisVar, th, thMax, Ka, keta1, keta2, Qx, R, PMag, PVar, wall, Wa, Wc, kpl, kdl, gammal);



  ros::AsyncSpinner spinner(8);
  spinner.start();
  ros::waitForShutdown();

  //ros::spin();

  return 0;
}
