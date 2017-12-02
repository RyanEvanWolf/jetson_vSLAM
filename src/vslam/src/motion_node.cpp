#include <iostream>
#include <ros/ros.h>
#include <vSLAM_utils/extractMotion.h>


bool getMotion(vSLAM_utils::extractMotion::Request &req,
								vSLAM_utils::extractMotion::Response &res)
{
	
	return true;
}


int main(int argc, char *argv[])
{
	ros::init(argc,argv,"motion");
	ros::NodeHandle n;

	ros::ServiceServer service=n.advertiseService("extract/motion",getMotion);
	


	ros::spin();

	return 0;
}
