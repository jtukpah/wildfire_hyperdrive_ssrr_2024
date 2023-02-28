#!/usr/bin/env python3

import os
import sys
import cv2 as cv
import rospy
import typing
import rospkg
import ros_numpy
import numpy as np
import scipy.io as sio
from std_msgs.msg import String, Header
from sensor_msgs.msg import CompressedImage, Image

class ImageCorrecter(object):
    def __init__(self):
        # Setup callback for data
        self.new_camera_matrix = np.array([])
        self.load_distortion_parameters()
        self.pub = rospy.Publisher(f'/camera/image_raw/rect', Image, queue_size=10)
        # Subscribe to the raw data image
        self.sub = rospy.Subscriber(f'/camera/image_raw', Image, self.image_callback)

    def load_distortion_parameters(self) -> None:
        '''
        Load camera calibration coefficients from file
        '''
        # Get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        self.mat_data = sio.loadmat(os.path.join(rospack.get_path('hyper_drive'),'config','distortion','alvium_params.mat'))
        self.camera_matrix = self.mat_data['intrinsicMatrix']
        self.dist_coeffs = self.mat_data['distortionCoefficients']
        
    def image_callback(self, msg: Image) -> None:
        '''
        Callback function for hypercube data
        '''
        frame = ros_numpy.numpify(msg)
        if self.new_camera_matrix.size == 0:
            self.new_camera_matrix, _ = cv.getOptimalNewCameraMatrix(
                self.camera_matrix, self.dist_coeffs, frame.shape, 1, frame.shape
            )
        frame = cv.undistort(frame,self.camera_matrix,self.dist_coeffs,None,self.new_camera_matrix)
        ros_image = ros_numpy.msgify(Image, frame, encoding="8UC1")
        self.pub.publish(ros_image)
        
    def shutdown(self):
        '''
        Custom shutdown behavior
        '''
        return

if __name__ == '__main__':
    rospy.init_node('ImageCorrecter', anonymous=True)
    try:
        my_node = ImageCorrecter()
        rospy.spin()
    except rospy.ROSInterruptException:
        my_node.shutdown()
