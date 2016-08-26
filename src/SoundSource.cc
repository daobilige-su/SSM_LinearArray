#include "SoundSource.h"

namespace ORB_SLAM2
{

SoundSource::SoundSource(const std::string &strSettingPath) : mfmState(Eigen::MatrixXd::Constant(8,1,0.0)) , mfmCov(Eigen::MatrixXd::Constant(8,8,0.0)) , mbValid(0), mfSumChiSquare(0.0), mnIniNum(0), mnIniNumMin(5) {
	msSettingPath = strSettingPath;
	cv::FileStorage fSettings(msSettingPath, cv::FileStorage::READ);

	fSettings["SSL.nMultiHypoNum"] >> mnHypoNum;
	fSettings["SSL.dIdpDistMin"] >> mfDistMin;
}

bool SoundSource::initialize(Eigen::MatrixXd mCurrentCameraFrame, double circum_angle, double axis_angle, double axis_angle_std){

	mfmState(0,0) = mCurrentCameraFrame(0,3);
	mfmState(1,0) = mCurrentCameraFrame(1,3);
	mfmState(2,0) = mCurrentCameraFrame(2,3);

	//Eigen::MatrixXd lmLocalXYZHom(4,1);
	//lmLocalXYZHom << cos(axis_angle)*cos(circum_angle), sin(axis_angle), cos(axis_angle)*sin(circum_angle), 1;
	
	//Eigen::MatrixXd lmGlobalXYZHom = mCurrentCameraFrame*lmLocalXYZHom;

	//double lmIniAzim = atan2(lmGlobalXYZHom(1,0)-mfmState(1,0),lmGlobalXYZHom(0,0)-mfmState(0,0));
	//double lmIniElev = atan2(lmGlobalXYZHom(2,0)-mfmState(2,0),sqrt( pow(lmGlobalXYZHom(0,0)-mfmState(0,0),2) + pow(lmGlobalXYZHom(1,0)-mfmState(1,0),2) ));
	
	Eigen::MatrixXd myAxisUnitPointHom(4,1);
	myAxisUnitPointHom << 0.0, 1.0, 0.0, 1.0;

	Eigen::MatrixXd mCurrentCameraFrameOrientation = mCurrentCameraFrame;
	mCurrentCameraFrameOrientation(0,3)=0.0;
	mCurrentCameraFrameOrientation(1,3)=0.0;
	mCurrentCameraFrameOrientation(2,3)=0.0;

	Eigen::MatrixXd yAxisDirectPointHom = mCurrentCameraFrameOrientation*myAxisUnitPointHom;

	double yAxisAzim = atan2(yAxisDirectPointHom(1,0),yAxisDirectPointHom(0,0));
	double yAxisElev = atan2( yAxisDirectPointHom(2,0), sqrt( pow(yAxisDirectPointHom(0,0),2) + pow(yAxisDirectPointHom(1,0),2) ) );

	//double mfDistMin = 0.1;
	mfmState(3,0) = yAxisAzim;
	mfmState(4,0) = yAxisElev;
	mfmState(5,0) = axis_angle;
	mfmState(6,0) = circum_angle;
	mfmState(7,0) = 1.0/(3.0*mfDistMin);

	// std
	//int hypo_num = 10;
	double circum_angle_std = ((360.0/double(mnHypoNum))*0.5)*(M_PI/180.0);
	//Eigen::MatrixXd sigmaAzimElevIdp(3,3);
	//sigmaAzimElevIdp << pow(axis_angle_std,2),0,0,
	//					0, pow(circum_angle_std,2), 0,
	//					0, 0, pow( (1.0/(3.0*mfDistMin)) ,2);

	//Eigen::MatrixXd F = jacobian_linear_to_azim_elev(mCurrentCameraFrame, axis_angle, circum_angle);
	
	Eigen::MatrixXd lmHypoSigma = Eigen::MatrixXd::Constant(8,8,0.0);
	lmHypoSigma(5,5) = pow(axis_angle_std,2);
	lmHypoSigma(6,6) = pow(circum_angle_std,2);
	lmHypoSigma(7,7) = pow( (1.0/(3.0*mfDistMin)) ,2);

	mfmCov = lmHypoSigma;
	mbValid = 1;

	return 1;
}

bool SoundSource::EKFTrack(Eigen::MatrixXd mCurrentCameraFrame, double axis_angle, double axis_angle_std){
	++mnIniNum;
	
	// hypo prunning
	ChiSquareHypoPrunning(mCurrentCameraFrame, axis_angle, axis_angle_std);

	double Z = axis_angle;

	//Eigen::MatrixXd temp(3,1);
	//temp << cos(mfmState(4,0))*cos(mfmState(3,0)),cos(mfmState(4,0))*sin(mfmState(3,0)),sin(mfmState(4,0));
	//Eigen::MatrixXd lmGlobalXYZ = (1.0/mfmState(5,0))*temp + mfmState.block(0,0,3,1);

	//Eigen::MatrixXd lmGlobalXYZHom(4,1);
	//lmGlobalXYZHom.block(0,0,3,1) << lmGlobalXYZ(0,0),lmGlobalXYZ(1,0),lmGlobalXYZ(2,0);
	//lmGlobalXYZHom(3,0) = 1;

	//std::cout<<"E here4?"<<std::endl;
	//Eigen::MatrixXd mCurrentCameraFrameInv(4,4);
	//Eigen::MatrixXd temp2 = mCurrentCameraFrame.block(0,0,3,3).transpose();
	//Eigen::MatrixXd temp3 = (-1.0)*mCurrentCameraFrame.block(0,0,3,3).transpose()*mCurrentCameraFrame.block(0,3,3,1);
	//mCurrentCameraFrameInv << temp2(0,0), temp2(0,1), temp2(0,2), temp3(0,0),
	//						  temp2(1,0), temp2(1,1), temp2(1,2), temp3(1,0),
	//						  temp2(2,0), temp2(2,1), temp2(2,2), temp3(2,0),
	//						  0,0,0,1;

	//std::cout<<"E here5?"<<std::endl;
	//Eigen::MatrixXd l_local_hom = mCurrentCameraFrameInv*lmGlobalXYZHom;

	//double expectedZ = atan2(l_local_hom(1,0),sqrt( pow(l_local_hom(0,0),2) + pow(l_local_hom(2,0),2) ) );

	Eigen::MatrixXd temp(3,1);
	temp << mfmState(3,0)-(M_PI/2.0), 0.0, mfmState(4,0);
	Eigen::MatrixXd y_axis_coord_M = Converter::YPRToMatrix4d(temp);
	y_axis_coord_M.block(0,3,3,1) << mfmState(0,0),mfmState(1,0),mfmState(2,0);

	Eigen::MatrixXd temp2(3,1);
	temp2 << cos(mfmState(5,0))*cos(mfmState(6,0)), sin(mfmState(5,0)), cos(mfmState(5,0))*sin(mfmState(6,0));
	temp2 = (1.0/mfmState(7,0))*temp2;
	Eigen::MatrixXd temp3(4,1);
	temp3 << temp2(0,0), temp2(1,0), temp2(2,0), 1; 
	Eigen::MatrixXd lm_global_xyz_hom = y_axis_coord_M*temp3;

	Eigen::MatrixXd mCurrentCameraFrameInv(4,4);
	Eigen::MatrixXd temp4 = mCurrentCameraFrame.block(0,0,3,3).transpose();
	Eigen::MatrixXd temp5 = (-1.0)*mCurrentCameraFrame.block(0,0,3,3).transpose()*mCurrentCameraFrame.block(0,3,3,1);
	mCurrentCameraFrameInv << temp4(0,0), temp4(0,1), temp4(0,2), temp5(0,0),
							  temp4(1,0), temp4(1,1), temp4(1,2), temp5(1,0),
							  temp4(2,0), temp4(2,1), temp4(2,2), temp5(2,0),
							  0,0,0,1;

	Eigen::MatrixXd l_local_hom = mCurrentCameraFrameInv*lm_global_xyz_hom;
	double expectedZ = atan2(l_local_hom(1,0),sqrt( pow(l_local_hom(0,0),2) + pow(l_local_hom(2,0),2) ) );

	Eigen::MatrixXd H = jacobian_ekf_update_3d_linear_hypo(mCurrentCameraFrame,mfmState);

	//std::cout<<"E here6?"<<std::endl;
	Eigen::MatrixXd Q(1,1);
	Q << pow(axis_angle_std,2);

	//Eigen::MatrixXd temp4 = mfmCov * (H.transpose());
	//Eigen::MatrixXd temp5 = (H*mfmCov*(H.transpose())+Q);
	//Eigen::MatrixXd K = temp4 / temp5;
	Eigen::MatrixXd K = mfmCov * (H.transpose()) * (H*mfmCov*(H.transpose())+Q).inverse();
	
	double Z_diff = Z - expectedZ;		
	if(Z_diff>M_PI)
		Z_diff -= 2*M_PI;
	if(Z_diff<(-1.0)*M_PI)
		Z_diff += 2*M_PI;

	//std::cout<<"E here7?"<<std::endl;
	mfmState += K*Z_diff;
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(8,8);
	mfmCov = (I-K*H)*mfmCov;

	//std::cout<<"E here8?"<<std::endl;
	return 1;
}

bool SoundSource::ChiSquareHypoPrunning(Eigen::MatrixXd mCurrentCameraFrame, double axis_angle, double axis_angle_std){
	//std::cout<<"P here1?"<<std::endl;
	double HypoRelaxingFactor = 2.0;

	double Z = axis_angle;
	
	//Eigen::MatrixXd temp(3,1);
	//temp << cos(mfmState(4,0))*cos(mfmState(3,0)),cos(mfmState(4,0))*sin(mfmState(3,0)),sin(mfmState(4,0));
	//Eigen::MatrixXd lmGlobalXYZ = (1.0/mfmState(5,0))*temp + mfmState.block(0,0,3,1);

	//Eigen::MatrixXd lmGlobalXYZHom(4,1);
	//lmGlobalXYZHom.block(0,0,3,1) << lmGlobalXYZ(0,0),lmGlobalXYZ(1,0),lmGlobalXYZ(2,0);
	//lmGlobalXYZHom(3,0) = 1;

	//Eigen::MatrixXd mCurrentCameraFrameInv(4,4);
	//Eigen::MatrixXd temp2 = mCurrentCameraFrame.block(0,0,3,3).transpose();
	//Eigen::MatrixXd temp3 = (-1.0)*mCurrentCameraFrame.block(0,0,3,3).transpose()*mCurrentCameraFrame.block(0,3,3,1);
	//mCurrentCameraFrameInv << temp2(0,0), temp2(0,1), temp2(0,2), temp3(0,0),
	//						  temp2(1,0), temp2(1,1), temp2(1,2), temp3(1,0),
	//						  temp2(2,0), temp2(2,1), temp2(2,2), temp3(2,0),
	//						  0,0,0,1;

	//Eigen::MatrixXd l_local_hom = mCurrentCameraFrameInv*lmGlobalXYZHom;

	//double expectedZ = atan2(l_local_hom(1,0),sqrt( pow(l_local_hom(0,0),2) + pow(l_local_hom(2,0),2) ) );

	Eigen::MatrixXd temp(3,1);
	temp << mfmState(3,0)-(M_PI/2.0), 0.0, mfmState(4,0);
	Eigen::MatrixXd y_axis_coord_M = Converter::YPRToMatrix4d(temp);
	y_axis_coord_M.block(0,3,3,1) << mfmState(0,0),mfmState(1,0),mfmState(2,0);

	Eigen::MatrixXd temp2(3,1);
	temp2 << cos(mfmState(5,0))*cos(mfmState(6,0)), sin(mfmState(5,0)), cos(mfmState(5,0))*sin(mfmState(6,0));
	temp2 = (1.0/mfmState(7,0))*temp2;
	Eigen::MatrixXd temp3(4,1);
	temp3 << temp2(0,0), temp2(1,0), temp2(2,0), 1; 
	Eigen::MatrixXd lm_global_xyz_hom = y_axis_coord_M*temp3;
	
	Eigen::MatrixXd mCurrentCameraFrameInv(4,4);
	Eigen::MatrixXd temp4 = mCurrentCameraFrame.block(0,0,3,3).transpose();
	Eigen::MatrixXd temp5 = (-1.0)*mCurrentCameraFrame.block(0,0,3,3).transpose()*mCurrentCameraFrame.block(0,3,3,1);
	mCurrentCameraFrameInv << temp4(0,0), temp4(0,1), temp4(0,2), temp5(0,0),
							  temp4(1,0), temp4(1,1), temp4(1,2), temp5(1,0),
							  temp4(2,0), temp4(2,1), temp4(2,2), temp5(2,0),
							  0,0,0,1;

	Eigen::MatrixXd l_local_hom = mCurrentCameraFrameInv*lm_global_xyz_hom;
	double expectedZ = atan2(l_local_hom(1,0),sqrt( pow(l_local_hom(0,0),2) + pow(l_local_hom(2,0),2) ) );

	
	//Eigen::MatrixXd F = jacobian_ekf_update_3d_linear_hypo(mCurrentCameraFrame,mfmState);
	Eigen::MatrixXd F = jacobian_ekf_update_3d_linear_hypo(mCurrentCameraFrame,mfmState);

	Eigen::MatrixXd Z_sigma = F*mfmCov*(F.transpose());

	double Z_diff = Z - expectedZ;		
	if(Z_diff>M_PI)
		Z_diff -= 2*M_PI;
	if(Z_diff<(-1.0)*M_PI)
		Z_diff += 2*M_PI;

	Eigen::MatrixXd temp6(1,1);
	temp6 << 1.0;
	Eigen::MatrixXd dist_square = Z_diff*(temp6*Z_sigma.inverse())*Z_diff;
	double dist_square_double = dist_square(0,0);
	

	mfSumChiSquare += dist_square_double;

	double chi2invValue = 6.6349;

	if(mnIniNum>mnIniNumMin){
		if(mfSumChiSquare>(chi2invValue*mnIniNum*HypoRelaxingFactor)){
			mbValid = 0;
		}
	}

	return 1;
}

}
