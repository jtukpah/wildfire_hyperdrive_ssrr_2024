#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import ros_numpy
import cv2
import os
import numpy as np
import sys
from imec_driver.srv import adjust_param


os.environ['PATH'] += os.pathsep + r'/home/river/Downloads/HSI Suite/bin'

sys.path.append('/home/river/Downloads/HSI Suite/python_apis')

import hsi_common as HSI_COMMON
import hsi_camera as HSI_CAMERA

class Nodo(object):
    def __init__(self):

         #getting user input exposure
        self.param_server = rospy.Service('/adjust_param', adjust_param, self.handle_adjust_param)

        # node cycle rate
        self.loop_rate = rospy.Rate(1)

        # subscriber
        rospy.Subscriber('img_pub', Image, self.callback)

        # publisher
        self.pub = rospy.Publisher('img_pub', Image, queue_size=10)
        
        #getting user input exposure
        self.model = rospy.get_param('hsi_processor')['camera_model']

        # look for connected cameras
        if self.model['camera_model'] == 'ximea':
            self.dev_list = HSI_CAMERA.EnumerateConnectedDevices(manufacturer=HSI_CAMERA.Manufacturer.EM_XIMEA)
        elif self.model['camera_model'] == 'imec':
            self.dev_list = HSI_CAMERA.EnumerateConnectedDevices(manufacturer=HSI_CAMERA.Manufacturer.EM_IMEC)
        rospy.loginfo('return number of devices = {}'.format(len(self.dev_list)))

        # and connect to the first one
        rospy.loginfo('looking for device:: {}'.format(self.dev_list[0]))
        self.device = HSI_CAMERA.OpenDevice(self.dev_list[0])

        #initialize camera
        rospy.loginfo('Initializing Camera...')
        HSI_CAMERA.Initialize(self.device)

        # Get/Set Camera Runtime Parameters (example)
        self.r_params = HSI_CAMERA.GetRuntimeParameters(self.device)

        if 1:
            # Explicit Allocate based on Output-Data-Format
            dfmt = HSI_CAMERA.GetOutputFrameDataFormat (self.device)
            self.frame = HSI_COMMON.AllocateFrame(dfmt)
            rospy.loginfo("Camera Output Data Format: {}".format(dfmt))
        else:
            # Implicit Allocate for Camera
            self.frame = HSI_CAMERA.AllocateFrameForCamera(self.device)
            rospy.loginfo("Camera Output Data Format: {}".format(self.frame.format))

    def callback(self, msg):
        self.image = ros_numpy.numpify(msg)
        self.image = self.image.astype(np.uint8)

        #apply color
        #COLOR_BAYER_BG2BGR
        # rgb_img = cv2.cvtColor(self.image, cv2.COLOR_BAYER_GB2BGR_EA)
        
        cv2.imshow("my image", self.image)
        cv2.waitKey(1)

    #handle function for adjust_param srv
    def handle_adjust_param(self, req):
        result = req.user_in

        if result >= 65 or result < 0:
            rospy.loginfo("exposure time out of range, enter number between 0.02 and 989.34")
            return None
        else:
            HSI_CAMERA.Pause(self.device)
            self.r_params['exposure_time_ms'] = result
            HSI_CAMERA.SetRuntimeParameters(self.device, self.r_params)       

        HSI_CAMERA.Start(self.device)
        return result

    def start(self):
        # the device is ready: we can start it now ...
        HSI_CAMERA.Start(self.device)
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            # Send a Software Trigger to the camera and grab the Frame
            HSI_CAMERA.Trigger(self.device)
            HSI_CAMERA.AcquireFrame(self.device, frame=self.frame)

            tmp = HSI_COMMON.FrameAsArray(self.frame) # internally convert frame to numpy array
            ros_image = ros_numpy.msgify(Image, tmp, encoding="32FC1")
            self.pub.publish(ros_image)

        # And Cleanup ...
        HSI_CAMERA.Pause(self.device)
        HSI_CAMERA.Stop(self.device)
        HSI_CAMERA.CloseDevice(self.device)
        HSI_COMMON.DeallocateFrame(self.frame)

if __name__ == '__main__':
    rospy.init_node('Nodo', anonymous=True)
    my_node = Nodo()
    my_node.start()
 
