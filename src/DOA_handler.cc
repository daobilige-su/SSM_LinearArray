/**
* This file is part of the SSM_LinearArray (Sound Sources Mapping
* using a Linear Microphone Array)
* developed by Daobilige Su <daobilige DOT su AT student DOT uts DOT edu DOT au>
*  
* This file is under the GPLv3 licence. 
*/

#include "DOA_handler.h"

namespace ORB_SLAM2
{

DOA_handler::DOA_handler(){}

DOA_handler::DOA_handler(const std::string &strSettingPath) : mnCurrentSSID(0){

	msSettingPath = strSettingPath;
	cv::FileStorage fSettings(msSettingPath, cv::FileStorage::READ);

	// Gaussian Process Training data
	cv::Mat mSettingGPX;
	fSettings["SSL.MicArray.GPX"] >> mSettingGPX;
	cv::Mat mSettingGPY;
	fSettings["SSL.MicArray.GPY"] >> mSettingGPY;
	// Gaussian Process Params
	double fGPSELamda;
	fSettings["SSL.MicArray.GPSELamda"] >> fGPSELamda;
	double fGPSESigma;
	fSettings["SSL.MicArray.GPSESigma"] >> fGPSESigma;
	double fGPNoiseSigma;
	fSettings["SSL.MicArray.GPNoiseSigma"] >> fGPNoiseSigma;

	// GP Model and params
	mpGP = new libgp::GaussianProcess(1, "CovSum ( CovSEiso, CovNoise)");
	Eigen::VectorXd gpParams(mpGP->covf().get_param_dim());
	gpParams << fGPSELamda, fGPSESigma, fGPNoiseSigma;
	mpGP->covf().set_loghyper(gpParams);

	// GP training data
	Eigen::MatrixXd mGPX(mSettingGPX.rows,mSettingGPX.cols);
	Eigen::MatrixXd mGPY(mSettingGPY.rows,mSettingGPY.cols);
	for(int i=0;i<mSettingGPX.rows;i++){
		for(int j=0;j<mSettingGPX.cols;j++){
			mGPX(i,j) = mSettingGPX.at<double>(i,j);
			mGPY(i,j) = mSettingGPY.at<double>(i,j);
		}
	}

	for(int n=0;n<370;n++){
		double x[]={mGPX(n,0)*(M_PI/180.0)};
		double y=mGPY(n,0)*(M_PI/180.0);
		mpGP->add_pattern(x,y);
	}

	// TEST total squared error
	std::cout<<"-----"<<std::endl<<"GP MIC ARRAY REGRESSION:"<<std::endl<<std::endl;
	for(int i = 0; i < 37; i++) {
		double x[] = {(5.0*i-90.0)*(M_PI/180.0)};
		double f = mpGP->f(x)*(180.0/M_PI);
		double var = mpGP->var(x);
		std::cout << "x = " << x[0]*(180.0/M_PI) << ", f = " << f << ", sqrt_var = " << sqrt(var)*(180.0/M_PI) << std::endl;
	}
	std::cout<<"-----"<<std::endl;

	// Initialize Data Association Object
	mpSSDataAssociation = new SSDataAssociation(msSettingPath);
}

void DOA_handler::HandleNewDOAmsg(){

	// Data Association
	bool bDAValid = mpSSDataAssociation->ComputeSSID(mvfCurrentDOALik);
	mnCurrentSSID = mpSSDataAssociation->mnCurrentSSID;
	mfCurrentDOAToAssign = mpSSDataAssociation->mfCurrentDOAToAssign;
	mfCurrentDOAStdToAssign = mpSSDataAssociation->mfCurrentDOAStdToAssign;
	// Apply GP sensor model
	for(std::vector<double>::iterator it=mfCurrentDOAToAssign.begin(), itEnd=mfCurrentDOAToAssign.end(); it!=itEnd; it++){
		*it = ComputeDOAf(*it);
	}
	for(std::vector<double>::iterator it=mfCurrentDOAStdToAssign.begin(), itEnd=mfCurrentDOAStdToAssign.end(); it!=itEnd; it++){
		*it = ComputeDOAStd(*it);
	}

	if(bDAValid){

		if(mpTracker->mState==ORB_SLAM2::Tracking::OK){
			// run Multi Hypo EKF if needed.
			if(int(mvpMultiHypoSSL.size())<mnCurrentSSID)
			{
				// create and initialize a new multi hypo EKFs
				MultiHypoSSL pCurMultiHypoSSL(msSettingPath);

				double axis_angle = mfCurrentDOAToAssign[0];
				double axis_angle_std = mfCurrentDOAStdToAssign[0];

				pCurMultiHypoSSL.MultiHypoInitialize(mCurrentCameraFrame, axis_angle, axis_angle_std);

				mvpMultiHypoSSL.push_back(pCurMultiHypoSSL);

				std::cout<<"current mvpMultiHypoSSL size = "<<mvpMultiHypoSSL.size()<<std::endl;
			}
			else{
				// track with multi hypothesis
				if(mvpMultiHypoSSL.at(mnCurrentSSID-1).mnState==1){
					double axis_angle = mfCurrentDOAToAssign[0];
					double axis_angle_std = mfCurrentDOAStdToAssign[0];
					(mvpMultiHypoSSL.at(mnCurrentSSID-1)).MultiHypoTrack(mCurrentCameraFrame, axis_angle, axis_angle_std);
				}
				else{
					std::cout<<"current sound source converged, optimization will take place"<<std::endl;
				}
		
				std::cout<<"current mvpMultiHypoSSL size = "<<mvpMultiHypoSSL.size()<<std::endl;
			}
		}
	}

}

void DOA_handler::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

// communication window, the ROS package code will use this function to set current DOA to this class
void DOA_handler::SetCurrentDOALik(const std::vector<double> vfCurrentDOALik){
	//current DOA Likelihood
	mvfCurrentDOALik = vfCurrentDOALik;
}

void DOA_handler::setCurrentCameraFrame(const cv::Mat &Tcw)
{
	cv::Mat temp = Tcw.clone();
    Eigen::MatrixXd temp2 = Converter::toMatrix4d(temp);
	Eigen::MatrixXd temp2R = temp2.block(0,0,3,3);
	Eigen::MatrixXd temp2T = temp2.block(0,3,3,1);
	
	Eigen::MatrixXd temp3R = temp2R.transpose();
	Eigen::MatrixXd temp3T = -1.0*temp3R*temp2T;
	Eigen::MatrixXd temp3(4,4);
	temp3 << temp3R(0,0),temp3R(0,1),temp3R(0,2),temp3T(0,0),
			 temp3R(1,0),temp3R(1,1),temp3R(1,2),temp3T(1,0),
			 temp3R(2,0),temp3R(2,1),temp3R(2,2),temp3T(2,0),
			 0, 0, 0, 1;

	Eigen::MatrixXd temp4 = Converter::ORBSLAM2CameraFrameToSSLLinearFrame(temp3);

	mCurrentCameraFrame = temp4;
}

double DOA_handler::ComputeDOAStd(double axis_angle){
	double axis_angle_array[]={axis_angle};
	double axis_angle_std_from_GP = sqrt(mpGP->var(axis_angle_array));
	return axis_angle_std_from_GP;
}

double DOA_handler::ComputeDOAf(double axis_angle){
	double axis_angle_array[]={axis_angle};
	double axis_angle_f_from_GP = mpGP->f(axis_angle_array);
	return axis_angle_f_from_GP;
}

}
