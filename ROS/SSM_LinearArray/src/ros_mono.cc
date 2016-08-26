/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
// TODO NEW: headers for Float32MultiArray Msg for DOA msgs
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM),mfDOAThreshold(2300){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

	void PassDOAMsg(const std_msgs::Float32MultiArray::ConstPtr& msgDOA);

    ORB_SLAM2::System* mpSLAM;

	double mfDOAThreshold;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

	// TODO NEW: DOA msg handler
    ros::Subscriber DOA_sub = nodeHandler.subscribe("/srp_phat_fd_value", 1, &ImageGrabber::PassDOAMsg, &igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}

void ImageGrabber::PassDOAMsg(const std_msgs::Float32MultiArray::ConstPtr& msgDOA){
	/*
	//cout<<"DOA msg received"<<endl;

	// find the max DOA that is larger than a threshold
    double DOA_data[37], DOA_data_max=0; 
	int DOA_max_idx = -1;
    for(int n=0;n<37;n++){
		if(n==0){
			DOA_data_max = mfDOAThreshold;
			//DOA_max_idx = 0;
		}
        DOA_data[n] = msgDOA->data[n];
		if(DOA_data[n]>DOA_data_max){
			DOA_max_idx = n;
			DOA_data_max = DOA_data[n];
		}
	}

	// if valid DOA found store it, else empty it.
	vector<double> vfCurrentDOA;
    if(DOA_max_idx>=0){
		vfCurrentDOA.push_back( (M_PI/180)*(5*double(DOA_max_idx-19)) );
		cout<<"current DOA is: "<<vfCurrentDOA[0]<<endl;
	}
	else{
		vfCurrentDOA.clear();
		cout<<"current DOA is empty"<<endl;
	}

	mpSLAM->SetCurrentDOA(vfCurrentDOA);
	*/

	vector<double> vfCurrentDOALik;
	for(int n=0;n<37;n++){
        vfCurrentDOALik.push_back(msgDOA->data[n]);
	}

	mpSLAM->SetCurrentDOALik(vfCurrentDOALik);
    
}
