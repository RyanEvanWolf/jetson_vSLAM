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


	cv::Mat outR,outT;
	cv::Mat E = findEssentialMat(currentPoints, prevPoints, 300.0, pp, CV_RANSAC, 0.99, 3, mask ); 
	recoverPose(E,currentPoints,prevPoints,outR,outT,300.0,pp,mask);

	cv::Mat P=cv::Mat::zeros(3,4,CV_64F);
	P.at<double>(0,0)=300.0;
	P.at<double>(0,2)=5;
	P.at<double>(1,1)=300.0;
	P.at<double>(1,2)=8;
	P.at<double>(2,2)=1;
	cv::Mat k=P(cv::Rect(0,0,3,3));


	int totalAverageSamples=0;

	cv::Mat average=cv::Mat::zeros(3,1,CV_64F);

	for(int index=0;index<req.current.left.landmarks.size();index++)
	{

		if(mask.at<bool>(0,index))
		{
			cv::Mat xnew,xold;
			//compute scale from projection 
			//projection pixel in previous frame
			xold=cv::Mat(3,1,CV_64F);
			xold.at<double>(0,0)=req.previous.right.features.at(index).x;
			xold.at<double>(1,0)=req.previous.right.features.at(index).y;
			xold.at<double>(2,0)=req.previous.right.features.at(index).z;

			xnew=cv::Mat(3,1,CV_64F);
			xnew.at<double>(0,0)=req.current.left.landmarks.at(index).x;
			xnew.at<double>(1,0)=req.current.left.landmarks.at(index).y;
			xnew.at<double>(2,0)=req.current.left.landmarks.at(index).z;
			average+=((k.inv()*xold-outR*xnew)*outT.inv(cv::DECOMP_SVD))*outT;
			totalAverageSamples++;
			if(totalAverageSamples==3)
			{
				index=req.current.left.landmarks.size();
			}
		}
	}

	if(totalAverageSamples>0)
	{
		average=average/double(totalAverageSamples); 
		//store and send back output in a ros message
		for(int row=0;row<3;row++)
		{
			for(int column=0;column<3;column++)
			{
				res.R.data[3*row +column]=outR.at<double>(row,column);
			}
		}
		res.T.x=average.at<double>(0,0);
		res.T.y=average.at<double>(1,0);
		res.T.z=average.at<double>(2,0);
		
		for(int index=0;index<mask.cols;index++)
		{
			res.mask.push_back(mask.at<bool>(0,index));	
		}
	}
	else
	{

		std::cout<<"no Inliers detected"<<std::endl;
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
