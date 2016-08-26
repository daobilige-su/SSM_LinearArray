#include "SSDataAssociation.h"

namespace ORB_SLAM2
{

// constructor

SSDataAssociation::SSDataAssociation(const std::string &strSettingPath) : mbSoundSourceTrackFlag(false), mnCurrentSSID(0), mnCurrentSSIDValidNum(0){
	//mfDOALikMin = 1300;//kinect
	//mfDOALikMin = 2300;//ps3-eye

	msSettingPath = strSettingPath;
	cv::FileStorage fSettings(msSettingPath, cv::FileStorage::READ);

	fSettings["SSL.DataAssociation.fDOALikMin"] >> mfDOALikMin;
}

bool SSDataAssociation::ComputeSSID(const std::vector<double> vfCurrentDOALik){

	// (1) find the max DOA that is larger than a threshold
    double DOA_data[37], DOA_data_max=mfDOALikMin; 
	int DOA_max_idx = -1;
    for(int n=0;n<37;n++){
        DOA_data[n] = vfCurrentDOALik[n];
		if(DOA_data[n]>DOA_data_max){
			DOA_max_idx = n;
			DOA_data_max = DOA_data[n];
		}
	}

	// if valid DOA found store it, else empty it.
	std::vector<double> vfCurrentDOA;
    if(DOA_max_idx>=0){
		vfCurrentDOA.push_back( (M_PI/180)*(5*double(DOA_max_idx-19)) );
		std::cout<<"current DOA is: "<<vfCurrentDOA[0]<<std::endl;
	}
	else{
		vfCurrentDOA.clear();
		std::cout<<"current DOA is empty"<<std::endl;
	}




	// (2) Assign mvfPreviousDOA and mvfCurrentDOA
	
	// previous 10 DOAs
	if(mvfPreviousDOA.size()<10){
		if(mvfCurrentDOA.size()==0){
			mvfPreviousDOA.push_back(-1000);
		}
		else{
			mvfPreviousDOA.push_back(mvfCurrentDOA[0]);
		}
		//std::cout<<"pre 10"<<std::endl;
	}
	else{
		//std::cout<<"after 10"<<std::endl;
		for(int n=0;n<9;n++){
			mvfPreviousDOA[n] = mvfPreviousDOA[n+1];
		}
		if(mvfCurrentDOA.size()>0){
			mvfPreviousDOA[9] = mvfCurrentDOA[0];
		}
		else{
			mvfPreviousDOA[9] = -1000.0;
		}
	}
	mvfCurrentDOA = vfCurrentDOA;

	

	// (3) determine mfCurrentDOAToAssign and mfCurrentDOAStdToAssign
	double rad_diff = 0;
	double rad_diff_max = double(20.0*(M_PI/180.0));
	int consistant_num = 0;
	int consistant_num_min = 6;

	bool consistant_flag = 0;

	if(mvfCurrentDOA.size()>0){
		for(std::vector<double>::const_iterator it=mvfPreviousDOA.begin(), itEnd=mvfPreviousDOA.end(); it!=itEnd; it++){

			if(*it==(-1000.0)){
				continue;
			}
			else{
				rad_diff=(*it-mvfCurrentDOA[0]);
				if(rad_diff>M_PI)
					rad_diff = rad_diff-2*M_PI;
				if(rad_diff<-M_PI)
					rad_diff = rad_diff+2*M_PI;

				if(rad_diff<=rad_diff_max)
					++consistant_num;
				
			}
		}
		//std::cout<<"consistant_num: "<<consistant_num<<std::endl;
		if(consistant_num>=consistant_num_min)
			consistant_flag = 1;
	}

	//std::cout<<"consistant_flag: "<<consistant_flag<<std::endl;
	if(consistant_flag){
		if(mnCurrentSSIDValidNum==0){
			if(DOA_data_max>mfDOALikMin*1.2){
				mnCurrentSSID += 1;
				mnCurrentSSIDValidNum = 10;
				mbSoundSourceTrackFlag = true;
			}
		}
		else{
			mnCurrentSSIDValidNum = 10;
		}
	}
	else{
		if(mnCurrentSSIDValidNum>0){
			mnCurrentSSIDValidNum -= 1;
			if(mnCurrentSSIDValidNum==0){
				mbSoundSourceTrackFlag = false;
			}
		}
	}

	std::vector<double> previousDOAToAssign = mfCurrentDOAToAssign;
	std::vector<double> previousDOAStdToAssign = mfCurrentDOAStdToAssign;
	mfCurrentDOAToAssign.clear();
	mfCurrentDOAStdToAssign.clear();
	if(mnCurrentSSIDValidNum>0){
		if(consistant_flag){
			mfCurrentDOAToAssign.push_back(mvfCurrentDOA[0]);
			mfCurrentDOAStdToAssign.push_back(mvfCurrentDOA[0]);
		}
		else{
			if(mbSoundSourceTrackFlag){
				mfCurrentDOAToAssign = previousDOAToAssign;
				mfCurrentDOAStdToAssign = previousDOAStdToAssign;
			}
		}
		if(mbSoundSourceTrackFlag){
			std::cout<<"SSID: "<<mnCurrentSSID<<", Current DOA: "<<mfCurrentDOAToAssign[0]<<std::endl;
		}
	}

	return mbSoundSourceTrackFlag;

}

}
