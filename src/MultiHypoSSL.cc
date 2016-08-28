/**
* This file is part of the SSM_LinearArray (Sound Sources Mapping
* using a Linear Microphone Array)
* developed by Daobilige Su <daobilige DOT su AT student DOT uts DOT edu DOT au>
*  
* This file is under the GPLv3 licence. 
*/

#include "MultiHypoSSL.h"

namespace ORB_SLAM2
{

MultiHypoSSL::MultiHypoSSL(const std::string &strSettingPath) : mnState(0), mmConvergedState(Eigen::MatrixXd::Constant(3,1,0.0)), mmConvergedCov(Eigen::MatrixXd::Constant(3,3,0.0)){

	msSettingPath = strSettingPath;
	cv::FileStorage fSettings(msSettingPath, cv::FileStorage::READ);

	fSettings["SSL.nMultiHypoNum"] >> mnHypoNum;
}

bool MultiHypoSSL::MultiHypoInitialize(Eigen::MatrixXd mCurrentCameraFrame, double axis_angle, double axis_angle_std){

	for(int n=0;n<mnHypoNum;n++){
		// compute circum_angle and wrapToPi
		double circum_angle = (double(360/mnHypoNum)*n)*(M_PI/180.0);
		if(circum_angle>M_PI)
		{
			circum_angle -= 2*M_PI;
		}

		// create new sound source for each hypo and initialize them and add them to the vector
		SoundSource pSSLHypo(msSettingPath);

		pSSLHypo.initialize(mCurrentCameraFrame, circum_angle, axis_angle, axis_angle_std);

		mvpSSLHypos.push_back(pSSLHypo);

		mnState = 1;
	}

	return 1;
}

bool MultiHypoSSL::MultiHypoTrack(Eigen::MatrixXd mCurrentCameraFrame, double axis_angle, double axis_angle_std){

	int nValidNum=0;

	std::vector<bool> vbPreviousValid;
	for(std::vector<SoundSource>::iterator it=mvpSSLHypos.begin(), itEnd=mvpSSLHypos.end(); it!=itEnd; it++){
		vbPreviousValid.push_back(it->mbValid);

		if(it->mbValid){
			it->EKFTrack(mCurrentCameraFrame, axis_angle, axis_angle_std);
			if(it->mbValid){
				++nValidNum;
			}
		}
	}
	
	// remain at least one hypothesis
	int nHypoIdx = 0;
	double fSumChiSquareMin = 0;
	int fSumChiSquareMinIdx = 0;
	if(nValidNum==0){
		for(std::vector<SoundSource>::iterator it2=mvpSSLHypos.begin(), itEnd2=mvpSSLHypos.end(); it2!=itEnd2; it2++){
			if(vbPreviousValid.at(nHypoIdx)){
				if(fSumChiSquareMin==0){
					fSumChiSquareMin = it2->mfSumChiSquare;
					fSumChiSquareMinIdx = nHypoIdx;
				}
				else{
					if( (it2->mfSumChiSquare)<fSumChiSquareMin )
					{
						fSumChiSquareMin = it2->mfSumChiSquare;
						fSumChiSquareMinIdx = nHypoIdx;
					}
				}
			}
			++nHypoIdx;
		}
		mvpSSLHypos.at(fSumChiSquareMinIdx).mbValid = 1;
	}

	double LdMax = 0.2, fMeanStdThr = 0.05;

	bool bCovConverged=true;
	Eigen::MatrixXd F_IdpToEuc(3,8);
	Eigen::MatrixXd SoundSourceCovEuc(3,3);
	std::vector<Eigen::MatrixXd> vmValidMus;
	Eigen::MatrixXd SoundSourceState(8,1);	
	Eigen::MatrixXd SoundSourceStateGlobalXYZ(3,1);
	Eigen::MatrixXd SoundSourceStateLocalBearing(3,1);

	Eigen::MatrixXd HXyzW(3,1);
	double SigmaRho=0, SigmaD = 0, CosAlpha = 0, D1 = 0, Ld = 0;
	Eigen::MatrixXd m(3,1);

	for(std::vector<SoundSource>::iterator it3=mvpSSLHypos.begin(), itEnd3=mvpSSLHypos.end(); it3!=itEnd3; it3++){
		if(it3->mbValid){
			SoundSourceState = it3->mfmState;

			Eigen::MatrixXd temp(3,1);
			temp << SoundSourceState(3,0)-(M_PI/2.0), 0.0, SoundSourceState(4,0);
			Eigen::MatrixXd y_axis_coord_M = Converter::YPRToMatrix4d(temp);
			y_axis_coord_M.block(0,3,3,1) << SoundSourceState(0,0),SoundSourceState(1,0),SoundSourceState(2,0);

			Eigen::MatrixXd temp2(3,1);
			temp2 << cos(SoundSourceState(5,0))*cos(SoundSourceState(6,0)), sin(SoundSourceState(5,0)), cos(SoundSourceState(5,0))*sin(SoundSourceState(6,0));
			temp2 = (1.0/SoundSourceState(7,0))*temp2;
			Eigen::MatrixXd temp3(4,1);
			temp3 << temp2(0,0), temp2(1,0), temp2(2,0), 1; 
			Eigen::MatrixXd lm_global_xyz_hom = y_axis_coord_M*temp3;
			SoundSourceStateGlobalXYZ = lm_global_xyz_hom.block(0,0,3,1);
			HXyzW = SoundSourceState.block(0,0,3,1) - SoundSourceStateGlobalXYZ;
			SigmaRho = sqrt(it3->mfmCov(7,7));
			m = (SoundSourceStateGlobalXYZ - SoundSourceState.block(0,0,3,1))/(SoundSourceStateGlobalXYZ - SoundSourceState.block(0,0,3,1)).norm();
			SigmaD = SigmaRho/pow(SoundSourceState(7,0),2);
			CosAlpha = (m.transpose()*HXyzW/HXyzW.norm())(0,0);
			D1 = HXyzW.norm();

			Ld = ((4.0*SigmaD)/D1)*abs(CosAlpha);

			std::cout<<"current hypo Linearity Index Ld:"<<Ld<<std::endl;
			if( Ld>LdMax ){
				bCovConverged = false;
				break;
			}
			else{
				vmValidMus.push_back(SoundSourceStateGlobalXYZ);				
			}
		}
	}
		
	Eigen::MatrixXd mMean = Eigen::MatrixXd::Constant(3,1,0.0);
	double fDistFromMean = 0;
	bool bMeanConverged = true;
	if(bCovConverged){
		for(std::vector<Eigen::MatrixXd>::const_iterator it4=vmValidMus.begin(), itEnd4=vmValidMus.end(); it4!=itEnd4; it4++){
			mMean(0,0) += (*it4)(0,0);
			mMean(1,0) += (*it4)(1,0);
			mMean(2,0) += (*it4)(2,0);
		}
		std::cout<<"mMean = "<<mMean<<std::endl;

		mMean(0,0) /= double(vmValidMus.size());
		mMean(1,0) /= double(vmValidMus.size());
		mMean(2,0) /= double(vmValidMus.size());

		for(std::vector<Eigen::MatrixXd>::const_iterator it5=vmValidMus.begin(), itEnd5=vmValidMus.end(); it5!=itEnd5; it5++){
			fDistFromMean = sqrt( pow((*it5)(0,0)-mMean(0,0),2)+pow((*it5)(1,0)-mMean(1,0),2)+pow((*it5)(2,0)-mMean(2,0),2) );
			if(fDistFromMean>fMeanStdThr){
				bMeanConverged=false;
				break;
			}
		}
		if(bMeanConverged){
			mnState = 2;
			mmConvergedState = mMean;
		}
	}
	
	return 1;
}

}
