#!/usr/bin/env python
# license removed for brevity

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
    
    print srp_p.shape[0]
    srp_p = np.reshape(srp_p,(azim_num,1))
    print srp_p
    
    pyplot.figure("srp_phat_p")
    pyplot.clf()
    #pyplot.plot(pointcloud[:,0],pointcloud[:,1],linestyle='None',marker='x')
    #pyplot.xlim(-180,180)
    #pyplot.ylim(-90,90)
    #pyplot.imshow(srp_p)
    x_idx = np.array([n for n in range(-90,95,5)])
    y_idx = np.array([0])
    #print x_idx,y_idx
    #pyplot.pcolor(x_idx,y_idx,srp_p.T)#,vmin=1300,vmax=1600)
    
    pyplot.plot(x_idx,srp_p)
    pyplot.draw()
    pyplot.pause(0.1)

    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('srp_phat_fd_gui', anonymous=True)

    rospy.Subscriber('/srp_phat_fd_value', numpy_msg(Float32MultiArray), callback,queue_size = 1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
