/**
* This file is part of the SSM_LinearArray (Sound Sources Mapping
* using a Linear Microphone Array)
* developed by Daobilige Su <daobilige DOT su AT student DOT uts DOT edu DOT au>
*  
* This file is a modified version of the original file in ORB-SLAM2, 
* which is under GPLv3 licence. Therefore, this file also inherits 
* the GPLv3 licence. 
*
* The visual SLAM frontend/backend is part of ORB-SLAM2.
* The copyright of ORB-SLAM2 is described as follows:
*
* --
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
* --
*/

#include<iostream> 
#include<algorithm> 
#include<fstream> //used for saving keyframes into a file after the job is finished.
#include<chrono> 

#include<ros/ros.h> // necessary ROS header.
#include <cv_bridge/cv_bridge.h>// for image related operation
// message_filters is a ROS lib that is used to synchronized different msgs based on different policies. 
// its input is multiple meassages and output is one/multi message(s)
// Original Comments: A set of message filters which take in messages and may output those messages at a later time, based on the conditions that filter needs met.
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// headers for Float32MultiArray Msg for DOA msgs
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include<opencv2/core/core.hpp> // for image related operation

#include"../../../include/System.h" // for ORBSLAM connection

using namespace std;

// defined a Image Grabber class
// it has a member mpSLAM of type ORB_SLAM2::System, and a member function GrabRGBD, which take two synchronized msgs (RGB and Depth) and run ORBSLAM's TrackRGBD method.
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
	
	void PassDOAMsg(const std_msgs::Float32MultiArray::ConstPtr& msgDOA);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    // if there is not enough args, make a warning and exit.
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // create a ImageGrabber (linked to ORB_SLAM2::System SLAM), ready to take two msgs (RGBD/Depth) and run TrackRGBD.
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    // acquire color and depth images and synchronize them
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));// after synchronization, output two msgs to ImageGrabber::GrabRGBD method.
	
	// DOA msg handler
    ros::Subscriber sub = nh.subscribe("/srp_phat_fd_value", 1, &ImageGrabber::PassDOAMsg, &igb);

    // during operation, the system loops here.
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // execute ORBSLAM's TrackRGBD method.
    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}

void ImageGrabber::PassDOAMsg(const std_msgs::Float32MultiArray::ConstPtr& msgDOA){
	vector<double> vfCurrentDOALik;
	for(int n=0;n<37;n++){
        vfCurrentDOALik.push_back(msgDOA->data[n]);
	}

	mpSLAM->SetCurrentDOALik(vfCurrentDOALik);
}

