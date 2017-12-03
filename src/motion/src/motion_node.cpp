#include <iostream>
#include <algorithm>
#include <ros/ros.h>
#include <motion/essentialEstimation.h>
#include "five-point-nister/five-point.hpp"
#include "Structures/Transforms/Isometry.hpp"

bool getMotion(motion::essentialEstimation::Request &req,
								motion::essentialEstimation::Response &res)
{
	//get the motion from the left Camera

	cv::Mat prevPoints,currentPoints;
	prevPoints=cv::Mat(req.previous.left.features.size(),2,CV_64F);
	currentPoints=cv::Mat(req.current.right.features.size(),2,CV_64F);

	for(int index=0;index<req.current.left.features.size();index++)
	{
		currentPoints.at<double>(index,0)=req.current.left.features.at(index).x/req.current.left.features.at(index).z;
		currentPoints.at<double>(index,1)=req.current.left.features.at(index).y/req.current.left.features.at(index).z;
	}
	for(int index=0;index<req.previous.right.features.size();index++)
	{
		prevPoints.at<double>(index,0)=req.previous.right.features.at(index).x/req.previous.right.features.at(index).z;
		prevPoints.at<double>(index,1)=req.previous.right.features.at(index).y/req.previous.right.features.at(index).z;
	}
	cv::Point2d pp(5, 8); 
	cv::Mat mask;
	cv::Mat E = findEssentialMat(currentPoints, prevPoints, 300.0, pp, CV_RANSAC, 0.99, 3, mask ); 
	
	cv::Mat r1,r2,t;
	decomposeEssentialMat(E,r1,r2,t);
	std::cout<<mask<<std::endl;
	stereo::Isometry first(r1,t),second(r1,-t),third(r2,t),fourth(r2,-t);

	for(int index=0;index<4;index++)
	{
		std::cout<<"-------\n\n";
		if(mask.at<bool>(0,index))
		{
			cv::Mat homogPoint=cv::Mat(4,1,CV_64F);
			homogPoint.at<double>(0,0)=req.previous.left.landmarks.at(index).x;
			homogPoint.at<double>(1,0)=req.previous.left.landmarks.at(index).y;
			homogPoint.at<double>(2,0)=req.previous.left.landmarks.at(index).z;
			homogPoint.at<double>(3,0)=1.0;

			cv::Mat currenthomog=cv::Mat(4,1,CV_64F);
			currenthomog.at<double>(0,0)=req.current.left.landmarks.at(index).x;
			currenthomog.at<double>(1,0)=req.current.left.landmarks.at(index).y;
			currenthomog.at<double>(2,0)=req.current.left.landmarks.at(index).z;
			currenthomog.at<double>(3,0)=1.0;

			cv::Mat test1,test2,test3,test4;
			
			test1=(first.invert()).getH()*homogPoint;
			test2=(second.invert()).getH()*homogPoint;
			test3=(third.invert()).getH()*homogPoint;
			test4=(fourth.invert()).getH()*homogPoint;

			std::cout<<homogPoint<<std::endl;
			std::cout<<currenthomog<<std::endl;
			std::cout<<"**\n";
			std::cout<<first.getT()<<"\n"<<first.getR()<<"\n"<<test1<<"---"<<std::endl;
			std::cout<<second.getT()<<"\n"<<second.getR()<<"\n"<<test2<<"---"<<std::endl;
			std::cout<<third.getT()<<"\n"<<third.getR()<<"\n"<<test3<<"---"<<std::endl;
			std::cout<<fourth.getT()<<"\n"<<fourth.getR()<<"\n"<<test4<<"---"<<std::endl;
		}
	}

	return true;
}



int main(int argc, char *argv[])
{
	ros::init(argc,argv,"motion");
	ros::NodeHandle n;

	ros::ServiceServer service=n.advertiseService("extract/EssentialMotion",getMotion);
	


	ros::spin();

	return 0;
}
