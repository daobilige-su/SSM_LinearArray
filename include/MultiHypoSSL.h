/**
* This file is part of the SSM_LinearArray (Sound Sources Mapping
* using a Linear Microphone Array)
* developed by Daobilige Su <daobilige DOT su AT student DOT uts DOT edu DOT au>
*  
* This file is under the GPLv3 licence. 
*/


#ifndef MULTIHYPOSSL_H
#define MULTIHYPOSSL_H

#include <Eigen/Dense>
#include <math.h>
#include "SoundSource.h"
#include "EKFJacobians.h"

#include<string>
#include<opencv2/core/core.hpp>

namespace ORB_SLAM2
{

class MultiHypoSSL
{
public:
	MultiHypoSSL();
	MultiHypoSSL(const std::string &strSettingPath);
	bool MultiHypoInitialize(Eigen::MatrixXd mCurrentCameraFrame, double axis_angle, double axis_angle_std);
	bool MultiHypoTrack(Eigen::MatrixXd mCurrentCameraFrame, double axis_angle, double axis_angle_std);

public:
	int mnHypoNum;
	int mnState;//0 for NOT INITIALIZED, 1 for INITIALIZED, 2 for CONVERGED
	Eigen::MatrixXd mmConvergedState;
	Eigen::MatrixXd mmConvergedCov;
	std::vector<SoundSource> mvpSSLHypos;

	std::string msSettingPath;
};

}

#endif 
