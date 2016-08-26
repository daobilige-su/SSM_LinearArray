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
    //std::vector<double> mfCurrentDOA;
	Eigen::MatrixXd mfmState;
	Eigen::MatrixXd mfmCov;
	bool mbValid;
	double mfSumChiSquare;
	int mnIniNum;
	int mnIniNumMin;

	std::string msSettingPath;
	int mnHypoNum;
	double mfDistMin;
private:
    

};

}// namespace ORB_SLAM

#endif // SYSTEM_H
