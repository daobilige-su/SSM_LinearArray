/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{

MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

	mViewerDenseMappingDistMax = fSettings["Viewer.DenseMappingDistMax"];
	mbMultiHypoEKFCovPlot = bool(int(fSettings["Viewer.bMultiHypoEKFCovPlot"]));
	mbOptSSCovPlot = bool(int(fSettings["Viewer.bOptSSCovPlot"]));
	
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

// TODO NEW: set DOA_handler
void MapDrawer::SetDOAHandler(DOA_handler *pDOAHandler)
{
    mpDOAHandler=pDOAHandler;
}

// TODO NEW: Draw Multi Hypo EKFs
void MapDrawer::DrawMultiHypoSSL()
{

	// params
	//bool bPlotCov = true;
	
    std::vector<MultiHypoSSL> vpMultiHypoSSL = mpDOAHandler->mvpMultiHypoSSL;
	std::vector<SoundSource> vpSoundSource;
	Eigen::MatrixXd SoundSourceState(8,1);	
	Eigen::MatrixXd SoundSourceStateGlobalXYZ(3,1);
	//Eigen::MatrixXd SoundSourceStateLocalBearing(3,1);

	Eigen::MatrixXd SoundSourceCov(8,8);
	Eigen::MatrixXd F_IdpToEuc(3,8);
	Eigen::MatrixXd SoundSourceCovEuc(3,3);
	Eigen::MatrixXd x(2,1);
	Eigen::MatrixXd C(2,2);
	double sxx=0, syy=0, sxy=0, a=0, b=0, angle=0, swap=0; 
	Eigen::MatrixXd ivec(1,36);
	for(int n=0;n<36;n++)
		ivec(0,n) = 10.0*n*(M_PI/180.0);
	Eigen::MatrixXd p(2,36);
	Eigen::MatrixXd R(2,2);
	Eigen::MatrixXd T(2,36);

	for(std::vector<MultiHypoSSL>::iterator vit=vpMultiHypoSSL.begin(), vend=vpMultiHypoSSL.end(); vit!=vend; vit++){
    	if(vit->mnState==1){
			//std::vector<SoundSource> mvpSSLHypos;
			vpSoundSource.clear();
			vpSoundSource = vit->mvpSSLHypos;
			
			for(std::vector<SoundSource>::iterator vits=vpSoundSource.begin(), vends=vpSoundSource.end(); vits!=vends; vits++){
			//std::vector<SoundSource>::iterator vits=vpSoundSource.begin();
				if(vits->mbValid){
					SoundSourceState = vits->mfmState;
					SoundSourceCov = vits->mfmCov;

					// draw the center
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

					glPointSize(mPointSize*5.0);
					glBegin(GL_POINTS);
					glColor3f(1.0,1.0,0.0);
					glVertex3f(SoundSourceStateGlobalXYZ(0,0),SoundSourceStateGlobalXYZ(1,0),SoundSourceStateGlobalXYZ(2,0));
					glEnd();
				
					// draw the covariance ellipse
					if(mbMultiHypoEKFCovPlot){
						F_IdpToEuc = jacobian_idp_to_euc(SoundSourceState);
						SoundSourceCovEuc = F_IdpToEuc*SoundSourceCov*F_IdpToEuc.transpose();

						//on xy plane
						x << SoundSourceStateGlobalXYZ(0,0), SoundSourceStateGlobalXYZ(1,0);
						C << SoundSourceCovEuc(0,0), SoundSourceCovEuc(0,1), SoundSourceCovEuc(1,0), SoundSourceCovEuc(1,1);
						sxx = C(0,0);
						syy = C(1,1);
						sxy = C(0,1);
				
						if((0.5*(sxx+syy+sqrt(pow((sxx-syy),2) + 4*pow(sxy,2)))>=0) && (0.5*(sxx+syy-sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))))>=0){

							a = sqrt(0.5*(sxx+syy+sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))));
							b = sqrt(0.5*(sxx+syy-sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))));					
							// 3 sigma region
							a *= sqrt(11.6183);
							b *= sqrt(11.6183);
							if(sxx<syy){
								swap = a;
								a=b;
								b=swap;
							}
					
							if(sxx!=syy){
								angle = 0.5*atan(2.0*sxy/(sxx-syy));
							}
							else{
								if(sxy==0)
									angle = 0;
								if(sxy>0)
									angle = M_PI/4.0;
								if(sxy<0)
									angle = -1.0*M_PI/4.0;
							}
					
							for(int n=0;n<36;n++){
								p(0,n) = a*cos(ivec(0,n));
								p(1,n) = b*sin(ivec(0,n));
							}

							R << cos(angle), -1.0*sin(angle), sin(angle), cos(angle);
							T = x*Eigen::MatrixXd::Constant(1,36,1.0);
							p = R*p+T;

							glLineWidth(mCameraLineWidth);
							glColor3f(1.0f,1.0f,0.0f);
							glBegin(GL_LINES);
					
							for(int n=0;n<35;n++){
								glVertex3f(p(0,n),p(1,n),SoundSourceStateGlobalXYZ(2,0));
								glVertex3f(p(0,n+1),p(1,n+1),SoundSourceStateGlobalXYZ(2,0));
							}
							glVertex3f(p(0,35),p(1,35),SoundSourceStateGlobalXYZ(2,0));
							glVertex3f(p(0,0),p(1,0),SoundSourceStateGlobalXYZ(2,0));
					
							glEnd();
						}
						else{
							std::cout<<"[SSL_Linear: User-Level] Error in MapDrawer::DrawMultiHypoSSL(): Cov matrix is not positive definate."<<std::endl;
						}

						//on xz plane
						x << SoundSourceStateGlobalXYZ(0,0), SoundSourceStateGlobalXYZ(2,0);
						C << SoundSourceCovEuc(0,0), SoundSourceCovEuc(0,2), SoundSourceCovEuc(2,0), SoundSourceCovEuc(2,2);
						sxx = C(0,0);
						syy = C(1,1);
						sxy = C(0,1);
						if((0.5*(sxx+syy+sqrt(pow((sxx-syy),2) + 4*pow(sxy,2)))>=0) && (0.5*(sxx+syy-sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))))>=0){

							a = sqrt(0.5*(sxx+syy+sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))));
							b = sqrt(0.5*(sxx+syy-sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))));					
							// 3 sigma region
							a *= sqrt(11.6183);
							b *= sqrt(11.6183);
							if(sxx<syy){
								swap = a;
								a=b;
								b=swap;
							}
					
							if(sxx!=syy){
								angle = 0.5*atan(2.0*sxy/(sxx-syy));
							}
							else{
								if(sxy==0)
									angle = 0;
								if(sxy>0)
									angle = M_PI/4.0;
								if(sxy<0)
									angle = -1.0*M_PI/4.0;
							}
					
							for(int n=0;n<36;n++){
								p(0,n) = a*cos(ivec(0,n));
								p(1,n) = b*sin(ivec(0,n));
							}

							R << cos(angle), -1.0*sin(angle), sin(angle), cos(angle);
							T = x*Eigen::MatrixXd::Constant(1,36,1.0);
							p = R*p+T;

							glLineWidth(mCameraLineWidth);
							glColor3f(1.0f,1.0f,0.0f);
							glBegin(GL_LINES);
					
							for(int n=0;n<35;n++){
								glVertex3f(p(0,n),SoundSourceStateGlobalXYZ(1,0),p(1,n));
								glVertex3f(p(0,n+1),SoundSourceStateGlobalXYZ(1,0),p(1,n+1));
							}
							glVertex3f(p(0,35),SoundSourceStateGlobalXYZ(1,0),p(1,35));
							glVertex3f(p(0,0),SoundSourceStateGlobalXYZ(1,0),p(1,0));
					
							glEnd();
						}
						else{
							std::cout<<"[SSL_Linear: User-Level] Error in MapDrawer::DrawMultiHypoSSL(): Cov matrix is not positive definate."<<std::endl;
						}

						//on yz plane
						x << SoundSourceStateGlobalXYZ(1,0), SoundSourceStateGlobalXYZ(2,0);
						C << SoundSourceCovEuc(1,1), SoundSourceCovEuc(1,2), SoundSourceCovEuc(2,1), SoundSourceCovEuc(2,2);
						sxx = C(0,0);
						syy = C(1,1);
						sxy = C(0,1);
						if((0.5*(sxx+syy+sqrt(pow((sxx-syy),2) + 4*pow(sxy,2)))>=0) && (0.5*(sxx+syy-sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))))>=0){

							a = sqrt(0.5*(sxx+syy+sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))));
							b = sqrt(0.5*(sxx+syy-sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))));					
							// 3 sigma region
							a *= sqrt(11.6183);
							b *= sqrt(11.6183);
							if(sxx<syy){
								swap = a;
								a=b;
								b=swap;
							}
					
							if(sxx!=syy){
								angle = 0.5*atan(2.0*sxy/(sxx-syy));
							}
							else{
								if(sxy==0)
									angle = 0;
								if(sxy>0)
									angle = M_PI/4.0;
								if(sxy<0)
									angle = -1.0*M_PI/4.0;
							}
					
							for(int n=0;n<36;n++){
								p(0,n) = a*cos(ivec(0,n));
								p(1,n) = b*sin(ivec(0,n));
							}

							R << cos(angle), -1.0*sin(angle), sin(angle), cos(angle);
							T = x*Eigen::MatrixXd::Constant(1,36,1.0);
							p = R*p+T;

							glLineWidth(mCameraLineWidth);
							glColor3f(1.0f,1.0f,0.0f);
							glBegin(GL_LINES);
					
							for(int n=0;n<35;n++){
								glVertex3f(SoundSourceStateGlobalXYZ(0,0),p(0,n),p(1,n));
								glVertex3f(SoundSourceStateGlobalXYZ(0,0),p(0,n+1),p(1,n+1));
							}
							glVertex3f(SoundSourceStateGlobalXYZ(0,0),p(0,35),p(1,35));
							glVertex3f(SoundSourceStateGlobalXYZ(0,0),p(0,0),p(1,0));
					
							glEnd();
						}
						else{
							std::cout<<"[SSL_Linear: User-Level] Error in MapDrawer::DrawMultiHypoSSL(): Cov matrix is not positive definate."<<std::endl;
						}
					}// if vits->mbValid
				}// draw Cov
				
			}
		}

		if(vit->mnState==2){
			SoundSourceStateGlobalXYZ = vit->mmConvergedState;

			glPointSize(mPointSize*10.0);
			glBegin(GL_POINTS);
			glColor3f(1.0,1.0,0.0);
			glVertex3f(SoundSourceStateGlobalXYZ(0,0),SoundSourceStateGlobalXYZ(1,0),SoundSourceStateGlobalXYZ(2,0));
			glEnd();

			// draw the covariance ellipse
			if(mbOptSSCovPlot){
				SoundSourceCovEuc = vit->mmConvergedCov;

				//on xy plane
				x << SoundSourceStateGlobalXYZ(0,0), SoundSourceStateGlobalXYZ(1,0);
				C << SoundSourceCovEuc(0,0), SoundSourceCovEuc(0,1), SoundSourceCovEuc(1,0), SoundSourceCovEuc(1,1);
				sxx = C(0,0);
				syy = C(1,1);
				sxy = C(0,1);
		
				if((0.5*(sxx+syy+sqrt(pow((sxx-syy),2) + 4*pow(sxy,2)))>=0) && (0.5*(sxx+syy-sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))))>=0){

					a = sqrt(0.5*(sxx+syy+sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))));
					b = sqrt(0.5*(sxx+syy-sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))));					
					// 3 sigma region
					a *= sqrt(11.6183);
					b *= sqrt(11.6183);
					if(sxx<syy){
						swap = a;
						a=b;
						b=swap;
					}
			
					if(sxx!=syy){
						angle = 0.5*atan(2.0*sxy/(sxx-syy));
					}
					else{
						if(sxy==0)
							angle = 0;
						if(sxy>0)
							angle = M_PI/4.0;
						if(sxy<0)
							angle = -1.0*M_PI/4.0;
					}
			
					for(int n=0;n<36;n++){
						p(0,n) = a*cos(ivec(0,n));
						p(1,n) = b*sin(ivec(0,n));
					}

					R << cos(angle), -1.0*sin(angle), sin(angle), cos(angle);
					T = x*Eigen::MatrixXd::Constant(1,36,1.0);
					p = R*p+T;

					glLineWidth(mCameraLineWidth);
					glColor3f(1.0f,1.0f,0.0f);
					glBegin(GL_LINES);
			
					for(int n=0;n<35;n++){
						glVertex3f(p(0,n),p(1,n),SoundSourceStateGlobalXYZ(2,0));
						glVertex3f(p(0,n+1),p(1,n+1),SoundSourceStateGlobalXYZ(2,0));
					}
					glVertex3f(p(0,35),p(1,35),SoundSourceStateGlobalXYZ(2,0));
					glVertex3f(p(0,0),p(1,0),SoundSourceStateGlobalXYZ(2,0));
			
					glEnd();
				}
				else{
					std::cout<<"[SSL_Linear: User-Level] Error in MapDrawer::DrawMultiHypoSSL(): Cov matrix is not positive definate."<<std::endl;
				}

				//on xz plane
				x << SoundSourceStateGlobalXYZ(0,0), SoundSourceStateGlobalXYZ(2,0);
				C << SoundSourceCovEuc(0,0), SoundSourceCovEuc(0,2), SoundSourceCovEuc(2,0), SoundSourceCovEuc(2,2);
				sxx = C(0,0);
				syy = C(1,1);
				sxy = C(0,1);
				if((0.5*(sxx+syy+sqrt(pow((sxx-syy),2) + 4*pow(sxy,2)))>=0) && (0.5*(sxx+syy-sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))))>=0){

					a = sqrt(0.5*(sxx+syy+sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))));
					b = sqrt(0.5*(sxx+syy-sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))));					
					// 3 sigma region
					a *= sqrt(11.6183);
					b *= sqrt(11.6183);
					if(sxx<syy){
						swap = a;
						a=b;
						b=swap;
					}
			
					if(sxx!=syy){
						angle = 0.5*atan(2.0*sxy/(sxx-syy));
					}
					else{
						if(sxy==0)
							angle = 0;
						if(sxy>0)
							angle = M_PI/4.0;
						if(sxy<0)
							angle = -1.0*M_PI/4.0;
					}
			
					for(int n=0;n<36;n++){
						p(0,n) = a*cos(ivec(0,n));
						p(1,n) = b*sin(ivec(0,n));
					}

					R << cos(angle), -1.0*sin(angle), sin(angle), cos(angle);
					T = x*Eigen::MatrixXd::Constant(1,36,1.0);
					p = R*p+T;

					glLineWidth(mCameraLineWidth);
					glColor3f(1.0f,1.0f,0.0f);
					glBegin(GL_LINES);
			
					for(int n=0;n<35;n++){
						glVertex3f(p(0,n),SoundSourceStateGlobalXYZ(1,0),p(1,n));
						glVertex3f(p(0,n+1),SoundSourceStateGlobalXYZ(1,0),p(1,n+1));
					}
					glVertex3f(p(0,35),SoundSourceStateGlobalXYZ(1,0),p(1,35));
					glVertex3f(p(0,0),SoundSourceStateGlobalXYZ(1,0),p(1,0));
			
					glEnd();
				}
				else{
					std::cout<<"[SSL_Linear: User-Level] Error in MapDrawer::DrawMultiHypoSSL(): Cov matrix is not positive definate."<<std::endl;
				}

				//on yz plane
				x << SoundSourceStateGlobalXYZ(1,0), SoundSourceStateGlobalXYZ(2,0);
				C << SoundSourceCovEuc(1,1), SoundSourceCovEuc(1,2), SoundSourceCovEuc(2,1), SoundSourceCovEuc(2,2);
				sxx = C(0,0);
				syy = C(1,1);
				sxy = C(0,1);
				if((0.5*(sxx+syy+sqrt(pow((sxx-syy),2) + 4*pow(sxy,2)))>=0) && (0.5*(sxx+syy-sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))))>=0){

					a = sqrt(0.5*(sxx+syy+sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))));
					b = sqrt(0.5*(sxx+syy-sqrt(pow((sxx-syy),2) + 4*pow(sxy,2))));					
					// 3 sigma region
					a *= sqrt(11.6183);
					b *= sqrt(11.6183);
					if(sxx<syy){
						swap = a;
						a=b;
						b=swap;
					}
			
					if(sxx!=syy){
						angle = 0.5*atan(2.0*sxy/(sxx-syy));
					}
					else{
						if(sxy==0)
							angle = 0;
						if(sxy>0)
							angle = M_PI/4.0;
						if(sxy<0)
							angle = -1.0*M_PI/4.0;
					}
			
					for(int n=0;n<36;n++){
						p(0,n) = a*cos(ivec(0,n));
						p(1,n) = b*sin(ivec(0,n));
					}

					R << cos(angle), -1.0*sin(angle), sin(angle), cos(angle);
					T = x*Eigen::MatrixXd::Constant(1,36,1.0);
					p = R*p+T;

					glLineWidth(mCameraLineWidth);
					glColor3f(1.0f,1.0f,0.0f);
					glBegin(GL_LINES);
			
					for(int n=0;n<35;n++){
						glVertex3f(SoundSourceStateGlobalXYZ(0,0),p(0,n),p(1,n));
						glVertex3f(SoundSourceStateGlobalXYZ(0,0),p(0,n+1),p(1,n+1));
					}
					glVertex3f(SoundSourceStateGlobalXYZ(0,0),p(0,35),p(1,35));
					glVertex3f(SoundSourceStateGlobalXYZ(0,0),p(0,0),p(1,0));
			
					glEnd();
				}
				else{
					std::cout<<"[SSL_Linear: User-Level] Error in MapDrawer::DrawMultiHypoSSL(): Cov matrix is not positive definate."<<std::endl;
				}
			}//draw Cov
		}// if draw Cov
	}// if state == 2

	

    //for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    //{
    //    if((*sit)->isBad())
    //        continue;
    //    cv::Mat pos = (*sit)->GetWorldPos();
    //    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    //}

    //glEnd();
}

void MapDrawer::DrawPointCloud(){

    //const float &w = mKeyFrameSize;
    //const float h = w*0.75;
    //const float z = w*0.6;

	bool bDrawPointCloud = true;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

	//std::cout<<"DPC: here?"<<std::endl;
    if(bDrawPointCloud)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();
            glMultMatrixf(Twc.ptr<GLfloat>(0));

			glPointSize(mPointSize*1.0);
			glBegin(GL_POINTS);

			//cv::Mat mPCImDepth;
			//static float mPCinvfx;
    		//static float mPCinvfy;
			//static float mPCcx;
    		//static float mPCcy;

			cv::Mat imRGB;
			pKF->mPCImRGBRaw.copyTo(imRGB);
			cvtColor(imRGB,imRGB,CV_BayerGB2RGB);

			cv::Mat imDepth;
			pKF->mPCImDepth.copyTo(imDepth);

			static float invfx = pKF->mPCinvfx;
    		static float invfy = pKF->mPCinvfy;
			static float cx = pKF->mPCcx;
    		static float cy = pKF->mPCcy;

			for(int uIndex=0; uIndex<imDepth.cols; uIndex++){
				for(int vIndex=0; vIndex<imDepth.rows; vIndex++){
		
					const float v = float(vIndex);
					const float u = float(uIndex);

					const float z = imDepth.at<float>(int(v),int(u));

					const float x = (u-cx)*z*invfx;
					const float y = (v-cy)*z*invfy;

					if(z>0 && sqrt(pow(x,2)+pow(y,2)+pow(z,2))<mViewerDenseMappingDistMax){
						// Undistort corners
						//mat=mat.reshape(2);
						//cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
						//mat=mat.reshape(1);
						cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

						Eigen::Matrix<double,6,1> pPoint;

						pPoint << x3Dc.at<float>(0,0), x3Dc.at<float>(1,0), x3Dc.at<float>(2,0), imRGB.at<cv::Vec3b>(int(v),int(u))[0], imRGB.at<cv::Vec3b>(int(v),int(u))[1], imRGB.at<cv::Vec3b>(int(v),int(u))[2];

						glColor3f(double(pPoint(3,0)/255.0),double(pPoint(4,0)/255.0),double(pPoint(5,0)/255.0));
						glVertex3f(pPoint(0,0),pPoint(1,0),pPoint(2,0));
					}
				}
			}
			/*
			for(int k=0; k<int(pKF->mpPointCloud.size()); k++){
				//std::cout<<"DPC: here1-0?"<<std::endl;
				Eigen::Matrix<double,6,1> pPoint = pKF->mpPointCloud.at(k);
				
				//std::cout<<"DPC: here1-1?"<<std::endl;
				//std::cout<<"pPoint: "<<pPoint(0)<<", "<<pPoint(1)<<", "<<pPoint(2)<<", "<<pPoint(3)<<std::endl;
				glColor3f(double(pPoint(3,0)/255.0),double(pPoint(4,0)/255.0),double(pPoint(5,0)/255.0));
				glVertex3f(pPoint(0,0),pPoint(1,0),pPoint(2,0));
				//std::cout<<"DPC: here1-2?"<<std::endl;
			}
			*/

			glEnd();

            glPopMatrix();
        }
    }
}


} //namespace ORB_SLAM
