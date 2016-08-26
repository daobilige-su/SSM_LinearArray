#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
#sudo apt-get install python-pyaudio
import pyaudio
from rospy.numpy_msg import numpy_msg
import numpy as np
import time
import signal
import os
import sys

CHUNK = 3200
FORMAT = pyaudio.paInt16
CHANNELS = 4
RATE = 16000

DEV_IDX = 5
# import pyaudio
# obj = pyaudio.PyAudio()
# obj.get_device_info_by_index(0)...

#RECORD_SECONDS = 5
#WAVE_OUTPUT_FILENAME = "output_2.wav"
#b = np.random.randn(1,10)
p = pyaudio.PyAudio()
pub_mic_array = rospy.Publisher("/microphone_array_raw", numpy_msg(Int32MultiArray),queue_size=1)


def callback(in_data, frame_count, time_info, status):
    global np,pub_mic_array
    #print('called')
    numpydata = np.fromstring(in_data, dtype=np.int16)
    #print numpydata
    print('sending...')
    

    #mat_sess.putvalue('waves',numpydata)
    #mat_sess.run('doa_func(waves);')
    #mat_sess.run('save(\'~/work_space/sony_cam_mics/tmp.mat\',\'a\',\'waves\')')
    #mat_sess.getvalue('a')
    #mat_sess.getvalue('b')
    #time.sleep(5)
    #data = wf.readframes(frame_count)
    numpydata_msg = Int32MultiArray()
    numpydata_msg.data = numpydata
    pub_mic_array.publish(numpydata_msg)
    
    return (in_data, pyaudio.paContinue)
    #return (in_data, pyaudio.paComplete)

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK,
                input_device_index=DEV_IDX,
		stream_callback=callback)

def signal_handler(signal, frame):
    print('---stopping---')
    stream.close()
    p.terminate()
    sys.exit()

signal.signal(signal.SIGINT, signal_handler)


def talker():
    rospy.init_node('microphone_array_driver', anonymous=True)

    print("---recording---")

    stream.start_stream()
    while stream.is_active():
	time.sleep(0.1)

    stream.close()
    p.terminate()
    
    #sys.exit()
    #print('here?')
    #os._exit

    
    #pub_mic_array = rospy.Publisher("/microphone_array_raw", numpy_msg(Float32MultiArray),queue_size=10)
    


if __name__ == '__main__':
    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
