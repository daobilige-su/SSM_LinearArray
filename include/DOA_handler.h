/**
* This file is part of the SSM_LinearArray (Sound Sources Mapping
* using a Linear Microphone Array)
* developed by Daobilige Su <daobilige DOT su AT student DOT uts DOT edu DOT au>
*  
* This file is under the GPLv3 licence. 
*/


#ifndef DOA_HANDLER_H
#define DOA_HANDLER_H

#include "Tracking.h"
#include <math.h>
#include "MultiHypoSSL.h"
#include "SoundSource.h"
#include <opencv2/core/core.hpp>
#include "Converter.h"
#include "SSDataAssociation.h"

#include "../Thirdparty/libgp/include/gp.h"

#include<string>
#include<opencv2/core/core.hpp>

namespace ORB_SLAM2
{
// forward declaration of Tracking class
class Tracking;

class DOA_handler
{

public:
    // constructor
    DOA_handler();
	DOA_handler(const std::string &strSettingPath);

    // handle new DOA messages (main loop of the thread).
	void SetTracker(Tracking *pTracker);

	void SetCurrentDOALik(const std::vector<double> vfCurrentDOALik);

	void HandleNewDOAmsg();

	bool AssociateData();

	void setCurrentCameraFrame(const cv::Mat &Tcw);

	double ComputeDOAStd(double axis_angle);
	double ComputeDOAf(double axis_angle);
public:
	int mnCurrentSSID;
	std::vector<double> mfCurrentDOAToAssign;
	std::vector<double> mfCurrentDOAStdToAssign;
	std::vector<MultiHypoSSL> mvpMultiHypoSSL;
	Eigen::MatrixXd mCurrentCameraFrame;// Tcw
	libgp::GaussianProcess* mpGP;
	std::string msSettingPath;
	SSDataAssociation* mpSSDataAssociation;

private:
    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

	std::vector<double> mvfCurrentDOALik;
};

}// namespace ORB_SLAM

#endif
