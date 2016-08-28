/**
* This file is part of the SSM_LinearArray (Sound Sources Mapping
* using a Linear Microphone Array)
* developed by Daobilige Su <daobilige DOT su AT student DOT uts DOT edu DOT au>
*  
* This file is under the GPLv3 licence. 
*/


#ifndef SOUNDSOURCE_H
#define SOUNDSOURCE_H

#include <Eigen/Dense>
#include <math.h>
#include "Converter.h"
#include "EKFJacobians.h"

#include<string>
#include<opencv2/core/core.hpp>

namespace ORB_SLAM2
{

class SoundSource
{

public:
    // constructor
    SoundSource();
	SoundSource(const std::string &strSettingPath);
	// initialization
	bool initialize(Eigen::MatrixXd mCurrentCameraFrame, double circum_angle, double axis_angle, double axis_angle_std);
	// EKF track
	bool EKFTrack(Eigen::MatrixXd mCurrentCameraFrame, double axis_angle, double axis_angle_std);
	// Hypo prunning
	bool ChiSquareHypoPrunning(Eigen::MatrixXd mCurrentCameraFrame, double axis_angle, double axis_angle_std);

public:
	Eigen::MatrixXd mfmState;
	Eigen::MatrixXd mfmCov;
	bool mbValid;
	double mfSumChiSquare;
	int mnIniNum;
	int mnIniNumMin;

	std::string msSettingPath;
	int mnHypoNum;
	double mfDistMin;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
