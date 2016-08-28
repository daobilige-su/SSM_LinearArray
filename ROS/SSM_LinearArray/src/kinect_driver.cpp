/**
* This file is part of the SSM_LinearArray (Sound Sources Mapping
* using a Linear Microphone Array)
* developed by Daobilige Su <daobilige DOT su AT student DOT uts DOT edu DOT au>
*  
* This file is under the GPLv3 licence. 
*/

#include "libfreenect.h"
#include "libfreenect_audio.h"
#include <stdio.h>
#include <signal.h>

#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#define SOF 3200

static freenect_context* f_ctx;
static freenect_device* f_dev;
int die = 0;
ros::Publisher kinect_mic_raw_pub;
std_msgs::Float32MultiArray msg_kinect_mic_raw;
int sample_remains = 0;
double sample_to_pub[4][SOF];
int max_loop_per_read = 100;

void in_callback(freenect_device* dev, int num_samples,
                 int32_t* mic1, int32_t* mic2,
                 int32_t* mic3, int32_t* mic4,
                 int16_t* cancelled, void *unknown) 
{
	int fill_num_total = 0;
	int loop_num = 0;
	while(loop_num<max_loop_per_read)
	{
		loop_num = loop_num+1;
		
		if((sample_remains+num_samples-fill_num_total)<SOF)
		{
			for(int i=0;i<(num_samples-fill_num_total);i++)
			{
				sample_to_pub[0][sample_remains+i] = double(mic1[fill_num_total+i]);
				sample_to_pub[1][sample_remains+i] = double(mic2[fill_num_total+i]);
				sample_to_pub[2][sample_remains+i] = double(mic3[fill_num_total+i]);
				sample_to_pub[3][sample_remains+i] = double(mic4[fill_num_total+i]);
			}
			sample_remains = sample_remains+num_samples-fill_num_total;
			break;
		}
		else
		{
			int fill_num = SOF - sample_remains;
			for(int i=0;i<fill_num;i++)
			{
				sample_to_pub[0][sample_remains+i] = double(mic1[fill_num_total+i]);
				sample_to_pub[1][sample_remains+i] = double(mic2[fill_num_total+i]);
				sample_to_pub[2][sample_remains+i] = double(mic3[fill_num_total+i]);
				sample_to_pub[3][sample_remains+i] = double(mic4[fill_num_total+i]);
			}

			//publish data
			double mic_raw_in_one_chunk[4*SOF];
			for(int i=0;i<SOF;i++)
			{

				mic_raw_in_one_chunk[4*i+0] = sample_to_pub[0][i];
				mic_raw_in_one_chunk[4*i+1] = sample_to_pub[1][i];
				mic_raw_in_one_chunk[4*i+2] = sample_to_pub[2][i];
				mic_raw_in_one_chunk[4*i+3] = sample_to_pub[3][i];
			}
			msg_kinect_mic_raw.data.assign(mic_raw_in_one_chunk,mic_raw_in_one_chunk+4*SOF);
			kinect_mic_raw_pub.publish(msg_kinect_mic_raw);
			sample_remains = 0;
			fill_num_total = fill_num_total+fill_num;
		}
	}
}

void cleanup(int sig) {
	printf("Caught SIGINT, cleaning up\n");
	die = 1;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "kinect_driver");

	ros::NodeHandle n;

	kinect_mic_raw_pub = n.advertise<std_msgs::Float32MultiArray>("microphone_array_raw", 5);

	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}
	freenect_set_log_level(f_ctx, FREENECT_LOG_SPEW);
	freenect_select_subdevices(f_ctx, FREENECT_DEVICE_AUDIO);

	int nr_devices = freenect_num_devices (f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);
	if (nr_devices < 1) {
		freenect_shutdown(f_ctx);
		return 1;
	}

	int user_device_number = 0;
	if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
		printf("Could not open device\n");
		freenect_shutdown(f_ctx);
		return 1;
	}

	freenect_set_audio_in_callback(f_dev, in_callback);
	freenect_start_audio(f_dev);
	signal(SIGINT, cleanup);

	while(!die && freenect_process_events(f_ctx) >= 0) {
	}

	freenect_shutdown(f_ctx);
	return 0;
}

