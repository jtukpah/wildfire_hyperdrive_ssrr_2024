#!/usr/bin/env python2

import rospy
import cv2 as cv
import os
import numpy as np
import sys
import logging
from hsi_driver.msg import DataCube

os.environ['PATH'] += os.pathsep + r'/home/river/Downloads/HSI Suite/bin'

try:
    os.add_dll_directory(r'/opt/imec/hsi-mosaic/python_apis')
except:
    pass

sys.path.append('/opt/imec/hsi-mosaic/python_apis')

import hsi_common as HSI_COMMON
import hsi_camera as HSI_CAMERA

class CombineDataCube(object):
    def __init__(self):
        
        #cube vars
        self.ximea_cube = []
        self.imec_cube = []
        
        # subscriber
        rospy.Subscriber(f'cube_pub/ximea', DataCube, self.callback_ximea)
        rospy.Subscriber(f'cube_pub/imec', DataCube, self.callback_imec)

        # publisher
        # self.pub = rospy.Publisher('com_cube_pub', DataCube, queue_size=10)

    #callback for subscriber
    def callback_ximea(self, msg):
        #recontructs data cube
        self.ximea_cube = np.reshape(msg.data, (msg.width, msg.height, msg.lam))
        
    
    def callback_imec(self, msg):
        #reconstructs data cube
        self.imec_cube = np.reshape(msg.data, (msg.width, msg.height, msg.lam))
        
        #trims to fit ximea data cube
        self.imec_cube = cv.resize(self.imec_cube, (426,340))
        self.imec_cube2 = self.imec_cube[62:-61,9:-8,:]

    def start(self):
        while not rospy.is_shutdown():
            if self.ximea_cube != [] and self.imec_cube !=[]:
                rospy.loginfo(f'XIMEA SHAPE: {self.ximea_cube.shape}')
                rospy.loginfo(f'IMEC SHAPE: {self.imec_cube2.shape}')

                # #combines ximea and imec data cube into one (adds by lambda axis)
                # combine_cube = np.dstack((self.ximea_cube, self.imec_cube2))
                # np.save('/home/river/combine_cube.npy',combine_cube)
                # ros_cube = DataCube()
                # ros_cube.data = combine_cube.flatten()
                # ros_cube.width = combine_cube.shape[0]
                # ros_cube.height = combine_cube.shape[1]
                # ros_cube.lam = combine_cube.shape[2]
                # self.pub.publish(ros_cube)
                # rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('CombineDataCube', anonymous=True)
    my_node = CombineDataCube()
    my_node.start()