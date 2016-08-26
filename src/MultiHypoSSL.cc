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
	//std::cout<<"Here? (out3)"<<std::endl;
	for(std::vector<SoundSource>::iterator it=mvpSSLHypos.begin(), itEnd=mvpSSLHypos.end(); it!=itEnd; it++){
		//std::cout<<"Here? (in3)"<<std::endl;
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
	//std::cout<<"Here? (out2)"<<std::endl;
	if(nValidNum==0){
		for(std::vector<SoundSource>::iterator it2=mvpSSLHypos.begin(), itEnd2=mvpSSLHypos.end(); it2!=itEnd2; it2++){
			//std::cout<<"Here? (in2)"<<std::endl;
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


	
	// check convergence of the Multihyposis
	//double fSigmaThr = 0.06, fMeanStdThr = 0.05;
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

	//std::cout<<"Here? (out)"<<std::endl;
	for(std::vector<SoundSource>::iterator it3=mvpSSLHypos.begin(), itEnd3=mvpSSLHypos.end(); it3!=itEnd3; it3++){
		if(it3->mbValid){
			//std::cout<<"Here? (in)"<<std::endl;
			SoundSourceState = it3->mfmState;

			//SoundSourceStateLocalBearing << cos(SoundSourceState(4,0))*cos(SoundSourceState(3,0)),cos(SoundSourceState(4,0))*sin(SoundSourceState(3,0)),sin(SoundSourceState(4,0));
			//SoundSourceStateGlobalXYZ = (1.0/SoundSourceState(5,0))*SoundSourceStateLocalBearing + SoundSourceState.block(0,0,3,1);
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

			//F_IdpToEuc = jacobian_idp_to_euc(it3->mfmState);
			//SoundSourceCovEuc = F_IdpToEuc*(it3->mfmCov)*F_IdpToEuc.transpose();

			//std::cout<<"MHC here?"<<std::endl;
			HXyzW = SoundSourceState.block(0,0,3,1) - SoundSourceStateGlobalXYZ;
			//std::cout<<"MHC here1?"<<std::endl;
			SigmaRho = sqrt(it3->mfmCov(7,7));
			m = (SoundSourceStateGlobalXYZ - SoundSourceState.block(0,0,3,1))/(SoundSourceStateGlobalXYZ - SoundSourceState.block(0,0,3,1)).norm();
			
			//std::cout<<"MHC here2?"<<std::endl;
			SigmaD = SigmaRho/pow(SoundSourceState(7,0),2);
			//std::cout<< (m.transpose()*HXyzW/HXyzW.norm())<<std::endl;
			CosAlpha = (m.transpose()*HXyzW/HXyzW.norm())(0,0);
			//std::cout<<"MHC here3?"<<std::endl;
			D1 = HXyzW.norm();
			//std::cout<<"MHC here4?"<<std::endl;

			Ld = ((4.0*SigmaD)/D1)*abs(CosAlpha);

			std::cout<<"current hypo Linearity Index Ld:"<<Ld<<std::endl;
			if( Ld>LdMax ){
				bCovConverged = false;
				break;
			}
			else{
				//std::cout<<"SoundSourceStateGlobalXYZ = "<<SoundSourceStateGlobalXYZ<<std::endl;
				vmValidMus.push_back(SoundSourceStateGlobalXYZ);				
				//std::cout<<"vmValidMus.size()"<<vmValidMus.size()<<std::endl;// since vmValidMus is vector of Eigen::Matrix<double,3,1>, SoundSourceStateGlobalXYZ also needs to be Eigen::Matrix<double,3,1>. If SoundSourceStateGlobalXYZ is set to be Eigen::MatrixXd, even if at the end is 3x1 matrix, vmValidMus.push_back will not push back it successfully and vmValidMus.size()=0. 2nd, if std container is used together with Eigen Matrix, and Eigen Matrix is fixed size of 16 bit, it will be more complicated. Google it for details.
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
		//std::cout<<"vmValidMus.size() = "<<vmValidMus.size()<<std::endl;
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
			//std::cout<<"Current Sound Source Converged"<<std::endl;
			//std::cout<<"mmConvergedState = "<<mmConvergedState<<std::endl;
		}
		
	}

	//std::cout<<"nValidNum = "<<nValidNum<<std::endl;
	
	return 1;
}

}
