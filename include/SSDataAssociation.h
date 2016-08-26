#ifndef SSDataAssociation_H
#define SSDataAssociation_H

#include <Eigen/Dense>
#include <math.h>
//#include "SoundSource.h"
//#include "EKFJacobians.h"
#include <iostream>

#include<string>
#include<opencv2/core/core.hpp>

namespace ORB_SLAM2
{

class SSDataAssociation
{

public:
	// constructor
	SSDataAssociation(){};
	SSDataAssociation(const std::string &strSettingPath);
	
	bool ComputeSSID(const std::vector<double> vfCurrentDOALik);

public:
	std::vector<double> mvfCurrentDOALik;
	
	std::vector<double> mvfCurrentDOA;
	std::vector<double> mvfPreviousDOA;

	bool mbSoundSourceTrackFlag;
	int mnCurrentSSID;
	int mnCurrentSSIDValidNum;
	std::vector<double> mfCurrentDOAToAssign;
	std::vector<double> mfCurrentDOAStdToAssign;

	double mfDOALikMin;

	std::string msSettingPath;
private:
	

};

}

#endif
