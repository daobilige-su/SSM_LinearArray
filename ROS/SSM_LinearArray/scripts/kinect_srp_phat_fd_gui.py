#!/usr/bin/env python
#
# This file is part of the SSM_LinearArray (Sound Sources Mapping
# using a Linear Microphone Array)
# developed by Daobilige Su <daobilige DOT su AT student DOT uts DOT edu DOT au>
#  
# This file is under the GPLv3 licence. 
#

from __future__ import division

import rospy
from std_msgs.msg import Float32MultiArray
from rospy.numpy_msg import numpy_msg
import numpy as np
from matplotlib import pyplot
import time

azim_num = 180/5+1;
elev_num = 1;

def callback(srp_msg):
    srp_p = np.array(srp_msg.data)
    srp_p = np.reshape(srp_p,(azim_num,1))
    pyplot.figure("srp_phat_p")
    pyplot.clf()
    x_idx = np.array([n for n in range(-90,95,5)])
    y_idx = np.array([0])
    pyplot.plot(x_idx,srp_p)
    pyplot.draw()
    pyplot.pause(0.1)

def listener():
    rospy.init_node('srp_phat_fd_gui', anonymous=True)
    rospy.Subscriber('/srp_phat_fd_value', numpy_msg(Float32MultiArray), callback,queue_size = 1)
    rospy.spin()

if __name__ == '__main__':
    listener()
