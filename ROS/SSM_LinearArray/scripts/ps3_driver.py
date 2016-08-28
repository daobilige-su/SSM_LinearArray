#!/usr/bin/env python
#
# This file is part of the SSM_LinearArray (Sound Sources Mapping
# using a Linear Microphone Array)
# developed by Daobilige Su <daobilige DOT su AT student DOT uts DOT edu DOT au>
#  
# This file is under the GPLv3 licence. 
#
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
p = pyaudio.PyAudio()
pub_mic_array = rospy.Publisher("/microphone_array_raw", numpy_msg(Int32MultiArray),queue_size=1)

def callback(in_data, frame_count, time_info, status):
    global np,pub_mic_array
    numpydata = np.fromstring(in_data, dtype=np.int16)
    print('sending...')
    numpydata_msg = Int32MultiArray()
    numpydata_msg.data = numpydata
    pub_mic_array.publish(numpydata_msg)
    return (in_data, pyaudio.paContinue)

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
    
if __name__ == '__main__':
    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

