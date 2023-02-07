#!/usr/bin/env python3

import os
import sys
import cv2
import rospy
import typing
import rospkg
import traceback
import ros_numpy
import numpy as np
from bs4 import BeautifulSoup
from numba import jit, prange
import matplotlib.pyplot as plt
from imec_driver.msg import DataCube
from imec_driver.srv import adjust_param
from sensor_msgs.msg import Image

### Set parameters to find the HSI mosaic binaries
os.environ['PATH'] += os.pathsep + r'/opt/imec/hsi-mosaic/bin'
# Also add them to the path
sys.path.append('/opt/imec/hsi-mosaic/python_apis')
# Import IMEC specific libraries
import hsi_common as HSI_COMMON
import hsi_camera as HSI_CAMERA

class DataCubeGenerator(object):
    def __init__(self):
        # Setup callback for data
        self.model = rospy.get_param('~camera_model')
        self.param_server = rospy.Service(f'/{self.model}/adjust_param', adjust_param, self.handle_adjust_param)
        # Rate at which to generate composite data cubes
        # Look for connected cameras an choose appropriate model (assumes we only have 1 IMEC and 1 XIMEA)
        if self.model == 'ximea':
            self.integration_range = (0.021000, 999.995000)
            self.dev_list = HSI_CAMERA.EnumerateConnectedDevices(manufacturer=HSI_CAMERA.Manufacturer.EM_XIMEA)
        elif self.model == 'imec':
            self.integration_range = (0.010000, 90)
            self.dev_list = HSI_CAMERA.EnumerateConnectedDevices(manufacturer=HSI_CAMERA.Manufacturer.EM_IMEC)
        rospy.loginfo('Found number of devices = {}'.format(len(self.dev_list)))
       
        # Create publisher to send datacubes on
        # self.pub = rospy.Publisher(f'cube_pub/{self.model}', DataCube, queue_size=10)
        self.pub_raw = rospy.Publisher(f'raw_pub/{self.model}', Image, queue_size=10)
        # Load camera parameters
        self.parse_parameters()
        # Connect to the first available camera of the specified model type
        rospy.loginfo('looking for device:: {}'.format(self.dev_list[0]))
        self.device = HSI_CAMERA.OpenDevice(self.dev_list[0])
        rospy.loginfo('Initializing Camera...')
        HSI_CAMERA.Initialize(self.device)
        # Get/Set Camera Configuration Parameters (example)
        self.c_params = HSI_CAMERA.GetConfigurationParameters(self.device)
        print("C PARAMS>")
        print(self.c_params)
        self.r_params = HSI_CAMERA.GetRuntimeParameters(self.device)
        # self.r_params['trigger_mode'] = 1
        HSI_CAMERA.SetRuntimeParameters(self.device, self.r_params)
        print("R PARAMS>")
        print(self.r_params)        # Explicit Allocate based on Output-Data-Format
        dataformat = HSI_CAMERA.GetOutputFrameDataFormat (self.device)
        self.frame = HSI_COMMON.AllocateFrame(dataformat)
        rospy.loginfo("Camera Output Data Format: {}".format(dataformat))

    def parse_parameters(self) -> None:
        '''
        Load parameter for camera from manufacturer provided XML file
        for publication in datacube messages
        '''
        # Get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        param_path = os.path.join(rospack.get_path('imec_driver'),'config',f'{self.model}.xml')
        with open(param_path, 'r') as f:
            data = f.read()
            Bs_data = BeautifulSoup(data, "xml")
            self.central_wave = []
            self.fwhm = []
            self.QE = []
            for band in Bs_data.find_all("wavelength_nm"):
                self.central_wave.append(float(band.getText()))
            for band in Bs_data.find_all("fwhm_nm"):
                self.fwhm.append(float(band.getText()))
            for band in Bs_data.find_all("QE"):
                self.QE.append(float(band.getText()))
            # Get calibration coefficients
            coefficients = []
            for coefficient in Bs_data.find_all("coefficients"):
                coefficients.append(np.array([float(z) for z in coefficient['values'].split()]))
            self.coefficients = np.array(coefficients)
        
    def handle_adjust_param(self, req: adjust_param) -> adjust_param:
        '''
        Listen to user parameter requests
        '''
        try:
            # Pause the camera
            if self.integration_range[0] < req.integration_time < self.integration_range[1]:
                HSI_CAMERA.Pause(self.device)
                self.r_params.exposure_time_ms = req.integration_time
                print(f'Update Runtime Params: {HSI_CAMERA.SetRuntimeParameters(self.device, self.r_params)}')       
                HSI_CAMERA.Start(self.device)
                return True
            else:
                rospy.logerr('INVALID INTEGRATION TIME REQUESTED!')
                HSI_CAMERA.Start(self.device)
        except Exception as e:
            rospy.logerr(traceback.print_exc())
            rospy.logerr(str(e))
            rospy.logerr('Error setting user user parameter!')
            return False
    
    @staticmethod
    @jit(nopython=True)
    def demosaic_cube(data: np.ndarray, outpt_shape: np.array, use_in_width: np.int, use_in_height: np.int, mosaic_size: np.int) -> np.ndarray:
        '''
        Workaround method to demosaic image
        '''
        cube = np.zeros(outpt_shape, dtype=np.float32)
        # Pixel array lookup
        voxel_lookup = np.arange(0,mosaic_size**2,1).reshape((mosaic_size, mosaic_size))
        for x in range(use_in_width):
            for y in range(use_in_height):
                # Add pixel value to cube
                use_x = x//mosaic_size
                use_y = y//mosaic_size
                # NATHANIEL IS FIXING THIS CALCULATION
                band = voxel_lookup[(x%mosaic_size),(y%mosaic_size)]
                cube[use_x][use_y][band] = data[x][y]
        return cube
    
    @staticmethod
    @jit(nopython=True)
    def perform_corrective_factors(data: np.ndarray) -> np.ndarray:
        '''
        Using the factors provided in the XML file calculate the final bands

        This operation should be optimized with numpy for simplicity
        '''
        # TODO
        return data

    def publish_raw(self, raw: np.ndarray) -> None:
        ros_image = ros_numpy.msgify(Image, raw, encoding="32FC1")
        self.pub_raw.publish(ros_image)

    def publish_cube(self, cube: np.ndarray) -> None:
        '''
        Create a data cube message and publish to topic
        '''
        ros_cube = DataCube()
        ros_cube.data = cube.flatten()
        ros_cube.width, ros_cube.height, ros_cube.lam = tuple(cube.shape)
        ros_cube.qe = self.QE
        ros_cube.fwhm_nm = self.fwhm
        ros_cube.central_wavelengths = self.central_wave
        self.pub.publish(ros_cube)

    def run(self) -> None:
        '''
        Run central processing loop for camera
        '''
        HSI_CAMERA.Start(self.device)
        while not rospy.is_shutdown():
            # Send a Software Trigger to the camera and grab the Frame
            HSI_CAMERA.Trigger(self.device)
            HSI_CAMERA.AcquireFrame(self.device, frame=self.frame)
            tmp = HSI_COMMON.FrameAsArray(self.frame) # internally convert frame to numpy array
            # Publish the raw image
            self.publish_raw(tmp)
            rospy.sleep(0.01)
        # End of loop behavior        
        self.shutdown()

    def shutdown(self):
        '''
        Custom shutdown behavior
        '''
        rospy.loginfo(f"Cleaning up node for the {self.model.upper()} camera...")
        HSI_COMMON.DeallocateFrame(self.frame)
        HSI_CAMERA.Pause(self.device)
        HSI_CAMERA.Stop(self.device)
        HSI_CAMERA.CloseDevice(self.device)

if __name__ == '__main__':
    rospy.init_node('DataCubeProcessor', anonymous=True)
    try:
        my_node = DataCubeGenerator()
        my_node.run()
    except rospy.ROSInterruptException:
        my_node.shutdown()
