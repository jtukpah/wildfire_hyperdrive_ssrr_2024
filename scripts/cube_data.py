#!/usr/bin/env python3

import rospy
import ros_numpy
import cv2
import os
import numpy as np
import sys
import logging
from imec_driver.msg import Num
from imec_driver.srv import adjust_param

os.environ['PATH'] += os.pathsep + r'/home/river/Downloads/HSI Suite/bin'

try:
    os.add_dll_directory(r'/opt/imec/hsi-mosaic/python_apis')
except:
    pass

sys.path.append('/opt/imec/hsi-mosaic/python_apis')

import hsi_common as HSI_COMMON
import hsi_camera as HSI_CAMERA

class DataCube(object):
    def __init__(self):
        
        #getting user input exposure
        self.param_server = rospy.Service('/adjust_param', adjust_param, self.handle_adjust_param)
        self.model = rospy.get_param('cube_processor')
    
        # node cycle rate
        self.loop_rate = rospy.Rate(1)

        # subscriber
        rospy.Subscriber('cube_pub', Num, self.callback)

        # publisher
        self.pub = rospy.Publisher('cube_pub', Num, queue_size=10)

        ########################################################################
        #CAMERA SETUP
        rospy.loginfo ("Starting Logger.")
        
        # look for connected cameras, choice between ximea and imec
        if self.model['camera_model'] == 'ximea':
            self.dev_list = HSI_CAMERA.EnumerateConnectedDevices(manufacturer=HSI_CAMERA.Manufacturer.EM_XIMEA)
        elif self.model['camera_model'] == 'imec':
            self.dev_list = HSI_CAMERA.EnumerateConnectedDevices(manufacturer=HSI_CAMERA.Manufacturer.EM_IMEC)
        rospy.loginfo('return number of devices = {}'.format(len(self.dev_list)))

        # and connect to the first one
        rospy.loginfo('looking for device:: {}'.format(self.dev_list[0]))
        self.device = HSI_CAMERA.OpenDevice(self.dev_list[0])
       
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
        
    #handle function for param srv    
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

    #callback for subscriber
    def callback(self, msg):
        cube = np.reshape(msg.data, (msg.width, msg.height, msg.lam))
        
        if self.model['camera_model'].lower() == 'ximea':
            cv2.imshow("my image", cube[:,:,5])
        elif self.model['camera_model'].lower() == 'imec':
            cv2.imshow("my_image", cube[:,:,3])
        cv2.waitKey(1)
        rospy.loginfo(f'Recieved data cube shape: {cube.shape}')

    def start(self):
        # the device is ready: we can start it now ...
        HSI_CAMERA.Start(self.device)
        while not rospy.is_shutdown():
            # Send a Software Trigger to the camera and grab the Frame
            HSI_CAMERA.Trigger(self.device)
            HSI_CAMERA.AcquireFrame(self.device, frame=self.frame)
            rospy.loginfo(self.frame)

            tmp = HSI_COMMON.FrameAsArray(self.frame) # internally convert frame to numpy array
            rospy.loginfo(self.frame.format)
            rospy.loginfo('Acquired Frame :: {}'.format(tmp.shape))

            ### TEMPORARY WORKAROUND TO MAKE A CUBE
            if self.model['camera_model'].lower() == 'ximea':
                cube = np.zeros((217,409,25))
                for x in range(1085):
                     for y in range(2045):
                         # Add pixel value to cube
                         use_x = x//5
                         use_y = y//5
                         band = (x%5)*(y%5)
                         cube[use_x][use_y][band] = tmp[x][y].copy()
            
            elif self.model['camera_model'].lower() == 'imec':
                    cube = np.zeros((170,213,9))
                    for x in range(510):
                        for y in range(639):
                            # Add pixel value to cube
                            use_x = x//3
                            use_y = y//3
                            band = (x%3)*(y%3)
                            cube[use_x][use_y][band] = tmp[x][y].copy()
            
            else:
                rospy.loginfo('Unknown camera model')

            rospy.loginfo(f'Published data cube shape: {cube.shape}')
            ros_cube = Num()
            ros_cube.data = cube.flatten()
            ros_cube.width = cube.shape[0]
            ros_cube.height = cube.shape[1]
            ros_cube.lam = cube.shape[2]
            self.pub.publish(ros_cube)
            # rospy.sleep(0.01)
        
        rospy.loginfo("Cleanup ...")
        HSI_COMMON.DeallocateFrame(self.frame)
        # HSI_COMMON.DeallocateCube(cube)
        # HSI_MOSAIC.DeallocateContext(context)
        # And Cleanup ...
        HSI_CAMERA.Pause(self.device)
        HSI_CAMERA.Stop(self.device)
        HSI_CAMERA.CloseDevice(self.device)

    rospy.loginfo ('MAIN DONE.')

if __name__ == '__main__':
    rospy.init_node('DataCube', anonymous=True)
    my_node = DataCube()
    my_node.start()