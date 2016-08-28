/**
* This file is part of the SSM_LinearArray (Sound Sources Mapping
* using a Linear Microphone Array)
* developed by Daobilige Su <daobilige DOT su AT student DOT uts DOT edu DOT au>
*  
* This file is under the GPLv3 licence. 
*/
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <complex>
#include <ctime>

#define SOF 3200
#define AZIM_STEP 5 //azim angle range (-90:AZIM_STEP:90)
#define AZIM_NUM 180/AZIM_STEP + 1
#define ELEV_NUM 1
#define AZIM_ELEV_NUM 37
#define FFT_NUM 1024
#define FFT_NUM_2_POW 10 //2^10 is 1024

double mic_pos[3*4] = {0.0,-0.190,0,0.0,-0.04,0.0,0.0,0.000,0.0,0.0,0.037,0.0};
double fTauSeq = 0.00001;

#define FS 16000
#define LOW_FREQ 1000
#define HIGH_FREQ 2000

double cs=340;
bool bIncludeChannelOne = false;
void FFT(int dir,int m,double x[],double y[]);
void compute_srp_phat_fd();
double raw1[SOF];                  
double raw2[SOF];
double raw3[SOF];
double raw4[SOF];
double srp_phat_fd_p[AZIM_NUM][ELEV_NUM];

using namespace std;

class srp_phat_fd
{
public:
	srp_phat_fd()
	{
		msg_count=0;
		sub = n.subscribe("microphone_array_raw", 1, &srp_phat_fd::AudioSensor_Callback,this);
		srp_pub = n.advertise<std_msgs::Float32MultiArray>("srp_phat_fd_value", 1);
	}
	~srp_phat_fd() 
	{
	}
	void AudioSensor_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg_rece)
	{
		int azim_num = AZIM_NUM;
		int elev_num = ELEV_NUM;
		for(int n=0;n<SOF;n++)
		{
			raw1[n]=msg_rece->data[4*n];
			raw2[n]=msg_rece->data[4*n+1];
			raw3[n]=msg_rece->data[4*n+2];
			raw4[n]=msg_rece->data[4*n+3];
		}		
	  	++msg_count;
	  	compute_srp_phat_fd();
		double srp_phat_fd_p_to_pub[AZIM_NUM];
		for(int i=0;i<azim_num;i++)
		{
			for(int j=0;j<elev_num;j++)
			{
				srp_phat_fd_p_to_pub[i*elev_num+j]=srp_phat_fd_p[i][j];
			}
		}
		msg_srp_phat.data.assign(srp_phat_fd_p_to_pub,srp_phat_fd_p_to_pub+AZIM_NUM);
		srp_pub.publish(msg_srp_phat);
	}
	
private:
	ros::NodeHandle n;
	ros::Publisher srp_pub;
	ros::Subscriber sub;
	int msg_count;
	std_msgs::Float32MultiArray msg_srp_phat;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "srp_phat_fd_node");
	srp_phat_fd srp_phat_fd_obj;
	ros::spin();
	return 0;
}

void compute_srp_phat_fd()
{
	double angle_vector[3];
	double azim_value,elev_value,tau[4];
	complex<double> s,tmp;
	double p;

	for(int a_idx=0;a_idx<AZIM_NUM;a_idx++)
	{
		for(int e_idx=0;e_idx<ELEV_NUM;e_idx++)
		{
			srp_phat_fd_p[a_idx][e_idx]=0;
		}
	}
	for(int fft_frame_idx=0;fft_frame_idx<int((double(SOF)/FFT_NUM));fft_frame_idx++)
	{
		double raw1_fft_r[FFT_NUM];                  
		double raw2_fft_r[FFT_NUM];
		double raw3_fft_r[FFT_NUM];                  
		double raw4_fft_r[FFT_NUM];
		double raw1_fft_i[FFT_NUM];                  
		double raw2_fft_i[FFT_NUM];
		double raw3_fft_i[FFT_NUM];                  
		double raw4_fft_i[FFT_NUM];
		for(int n=0;n<FFT_NUM;n++)
		{
			raw1_fft_r[n] = raw1[FFT_NUM*fft_frame_idx+n];
			raw2_fft_r[n] = raw2[FFT_NUM*fft_frame_idx+n];
			raw3_fft_r[n] = raw3[FFT_NUM*fft_frame_idx+n];
			raw4_fft_r[n] = raw4[FFT_NUM*fft_frame_idx+n];
			raw1_fft_i[n] = 0;
			raw2_fft_i[n] = 0;
			raw3_fft_i[n] = 0;
			raw4_fft_i[n] = 0;
		}
		FFT(1,FFT_NUM_2_POW,raw1_fft_r,raw1_fft_i);
		FFT(1,FFT_NUM_2_POW,raw2_fft_r,raw2_fft_i);
		FFT(1,FFT_NUM_2_POW,raw3_fft_r,raw3_fft_i);
		FFT(1,FFT_NUM_2_POW,raw4_fft_r,raw4_fft_i);

		for(int n=0;n<FFT_NUM;n++)
		{
			raw1_fft_r[n] = raw1_fft_r[n]*FFT_NUM;
			raw2_fft_r[n] = raw2_fft_r[n]*FFT_NUM;
			raw3_fft_r[n] = raw3_fft_r[n]*FFT_NUM;
			raw4_fft_r[n] = raw4_fft_r[n]*FFT_NUM;
			raw1_fft_i[n] = raw1_fft_i[n]*FFT_NUM;
			raw2_fft_i[n] = raw2_fft_i[n]*FFT_NUM;
			raw3_fft_i[n] = raw3_fft_i[n]*FFT_NUM;
			raw4_fft_i[n] = raw4_fft_i[n]*FFT_NUM;
		}

		complex<double> X_1[FFT_NUM];
		complex<double> X_2[FFT_NUM];
		complex<double> X_3[FFT_NUM];
		complex<double> X_4[FFT_NUM];

		for(int k=0;k<FFT_NUM;k++)
		{
			X_1[k] = {raw1_fft_r[k] , raw1_fft_i[k]};
			X_2[k] = {raw2_fft_r[k] , raw2_fft_i[k]};
			X_3[k] = {raw3_fft_r[k] , raw3_fft_i[k]};
			X_4[k] = {raw4_fft_r[k] , raw4_fft_i[k]};

			X_1[k] = X_1[k]/double(abs(X_1[k]));
			X_2[k] = X_2[k]/double(abs(X_2[k]));
			X_3[k] = X_3[k]/double(abs(X_3[k]));
			X_4[k] = X_4[k]/double(abs(X_4[k]));
		}

		for(int a_idx=0;a_idx<AZIM_NUM;a_idx++)
		{
			for(int e_idx=0;e_idx<ELEV_NUM;e_idx++)
			{
				azim_value = (-90 + AZIM_STEP*a_idx)*(M_PI/180);
				elev_value = 0;

				angle_vector[0] = -cos(elev_value)*cos(azim_value);
				angle_vector[1] = -cos(elev_value)*sin(azim_value);
				angle_vector[2] = -sin(elev_value);

				tau[0] = (angle_vector[0]*mic_pos[3*0+0]+angle_vector[1]*mic_pos[3*0+1]+angle_vector[2]*mic_pos[3*0+2])/cs;
				tau[1] = (angle_vector[0]*mic_pos[3*1+0]+angle_vector[1]*mic_pos[3*1+1]+angle_vector[2]*mic_pos[3*1+2])/cs+1*fTauSeq;
				tau[2] = (angle_vector[0]*mic_pos[3*2+0]+angle_vector[1]*mic_pos[3*2+1]+angle_vector[2]*mic_pos[3*2+2])/cs+2*fTauSeq;
				tau[3] = (angle_vector[0]*mic_pos[3*3+0]+angle_vector[1]*mic_pos[3*3+1]+angle_vector[2]*mic_pos[3*3+2])/cs+3*fTauSeq;
				p=0;
				for(int k=int((double(LOW_FREQ)/FS)*FFT_NUM);k<int((double(HIGH_FREQ)/FS)*FFT_NUM);k++)
				{
					s={0,0};
					if(bIncludeChannelOne){
						tmp = {0,2*M_PI*k/(double(FFT_NUM)/FS)*tau[0]};
						s=s+X_1[k]*exp(tmp);
					}
					tmp = {0,2*M_PI*k/(double(FFT_NUM)/FS)*tau[1]};
					s=s+X_2[k]*exp(tmp);
					tmp = {0,2*M_PI*k/(double(FFT_NUM)/FS)*tau[2]};
					s=s+X_3[k]*exp(tmp);
					tmp = {0,2*M_PI*k/(double(FFT_NUM)/FS)*tau[3]};
					s=s+X_4[k]*exp(tmp);
					p=p+abs(s)*abs(s);
				}
				srp_phat_fd_p[a_idx][e_idx] = srp_phat_fd_p[a_idx][e_idx] + p;
			}
		}
	}
}

void FFT(int dir,int m,double x[],double y[])
{
   	long n,i,i1,j,k,i2,l,l1,l2;
   	double c1,c2,tx,ty,t1,t2,u1,u2,z;
   	n = 1;
   	for (i=0;i<m;i++) 
      		n *= 2;
   		i2 = n >> 1;
   		j = 0;
   	for (i=0;i<n-1;i++) {
      		if (i < j) {
         		tx = x[i];
         		ty = y[i];
         		x[i] = x[j];
         		y[i] = y[j];
         		x[j] = tx;
         		y[j] = ty;
      		}
      		k = i2;
      		while (k <= j) {
         		j -= k;
         		k >>= 1;
      		}
      		j += k;
   	}
   	c1 = -1.0; 
   	c2 = 0.0;
   	l2 = 1;
   	for (l=0;l<m;l++) {
      		l1 = l2;
      		l2 <<= 1;
      		u1 = 1.0; 
      		u2 = 0.0;
      		for (j=0;j<l1;j++) {
         		for (i=j;i<n;i+=l2) {
            			i1 = i + l1;
            			t1 = u1 * x[i1] - u2 * y[i1];
            			t2 = u1 * y[i1] + u2 * x[i1];
            			x[i1] = x[i] - t1; 
            			y[i1] = y[i] - t2;
            			x[i] += t1;
            			y[i] += t2;
         		}
         		z =  u1 * c1 - u2 * c2;
         		u2 = u1 * c2 + u2 * c1;
         		u1 = z;
      		}
      		c2 = sqrt((1.0 - c1) / 2.0);
      		if (dir == 1) 
         		c2 = -c2;
      		c1 = sqrt((1.0 + c1) / 2.0);
   	}
   	if (dir == 1) {
      		for (i=0;i<n;i++) {
         		x[i] /= n;
         		y[i] /= n;
      		}
   	}
}




