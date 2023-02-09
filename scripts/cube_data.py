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
from pathlib import Path
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
import hsi_mosaic as HSI_MOSAIC

class DataCubeGenerator(object):
    def __init__(self):
        # Setup callback for data
        self.ros_pack = rospkg.RosPack()
        self.model = rospy.get_param('~camera_model')
        self.param_server = rospy.Service(f'adjust_param', adjust_param, self.handle_adjust_param)
        # Rate at which to generate composite data cubes
        # Look for connected cameras an choose appropriate model (assumes we only have 1 IMEC and 1 XIMEA)
        if self.model == 'ximea':
            self.integration_range = (0.021000, 999.995000)
            self.dev_list = HSI_CAMERA.EnumerateConnectedDevices(manufacturer=HSI_CAMERA.Manufacturer.EM_XIMEA)
            self.roi = [HSI_COMMON.RegionOfInterest(x=0, y=0, width=2045, height=1085)]

        elif self.model == 'imec':
            self.integration_range = (0.010000, 90)
            self.dev_list = HSI_CAMERA.EnumerateConnectedDevices(manufacturer=HSI_CAMERA.Manufacturer.EM_IMEC)
            self.roi = [HSI_COMMON.RegionOfInterest(x=1, y=1, width=639, height=510)]

        rospy.loginfo('Found number of devices = {}'.format(len(self.dev_list)))
       
        # Create publisher to send datacubes on
        # self.pub = rospy.Publisher(f'cube_pub/{self.model}', DataCube, queue_size=10)
        self.pub_raw = rospy.Publisher(f'raw_data', Image, queue_size=10)
        self.pub_cube = rospy.Publisher(f'cube_data', DataCube, queue_size=10)
        # Load camera parameters
        self.parse_parameters()
        # Connect to the first available camera of the specified model type
        rospy.loginfo('looking for device:: {}'.format(self.dev_list[0]))
        self.device = HSI_CAMERA.OpenDevice(self.dev_list[0])
        HSI_CAMERA.SetRegionOfInterestArray(self.device, self.roi)
        print(f"Region-of-Intereset Set to: {self.roi}")
        rospy.loginfo('Initializing Camera...')
        HSI_CAMERA.Initialize(self.device)
        # Get/Set Camera Configuration Parameters (example)
        self.c_params = HSI_CAMERA.GetConfigurationParameters(self.device)
        print("C PARAMS>")
        print(self.c_params)
        self.r_params = HSI_CAMERA.GetRuntimeParameters(self.device)
        self.r_params.frame_rate_hz = 60
        # self.r_params['trigger_mode'] = 1
        if self.model == 'imec':
            # self.r_params['exposure_time_ms'] = 60
            self.r_params['flip_vertical'] = False

        HSI_CAMERA.SetRuntimeParameters(self.device, self.r_params)
        print("R PARAMS>")
        print(self.r_params)        # Explicit Allocate based on Output-Data-Format
        dataformat = HSI_CAMERA.GetOutputFrameDataFormat (self.device)
        self.frame = HSI_COMMON.AllocateFrame(dataformat)
        rospy.loginfo("Camera Output Data Format: {}".format(dataformat))
        self.setup_context()

    def setup_context(self) -> None:
        '''
        Load HSI Context files and prepare to run demosaicing pipeline
        '''
        dn_context = os.path.join(self.ros_pack.get_path('imec_driver'),'config',self.model, 'context')
        #####################################################################
        version = HSI_MOSAIC.GetAPIVersion()
        print(f'VERSION :: {version}')
        print(Path(dn_context).absolute())
        assert Path(dn_context).exists()
        self.context = HSI_MOSAIC.LoadContext(dn_context)
        print(f'Context = {self.context}')
        status = HSI_MOSAIC.ContextGetStatus(self.context)
        print(status)
        self.pipeline = HSI_MOSAIC.Create(self.context)
        print(self.pipeline)

        self.params = HSI_MOSAIC.GetConfigurationParameters(self.pipeline)
        self.params.spatial_median_filter_enable = False
        print(f'Configuration Params : {self.params}')
        HSI_MOSAIC.SetConfigurationParameters(self.pipeline, self.params)
        print('Initializing Pipeline')
        HSI_MOSAIC.Initialize(self.pipeline)

        outputdataformat = HSI_MOSAIC.GetOutputDataFormat(self.pipeline)
        print(f'Output Data Format = {outputdataformat}')

        self.cube = HSI_COMMON.AllocateCube(outputdataformat)

        print('Starting Pipeline')
        HSI_MOSAIC.Start(self.pipeline)


    def parse_parameters(self) -> None:
        '''
        Load parameter for camera from manufacturer provided XML file
        for publication in datacube messages
        '''
        # Get an instance of RosPack with the default search paths
        param_path = os.path.join(self.ros_pack.get_path('imec_driver'),'config',f'{self.model}.xml')
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
    

    def publish_raw(self, raw: np.ndarray) -> None:
        ros_image = ros_numpy.msgify(Image, raw, encoding="32FC1")
        self.pub_raw.publish(ros_image)

    def publish_cube(self, cube: np.ndarray):
        '''
        Publish hyperspectral datacubes
        '''
        # Mark that we've received a new cube
        ros_cube = DataCube()
        ros_cube.data = cube.flatten()
        ros_cube.width, ros_cube.height, ros_cube.lam = tuple(cube.shape)
        ros_cube.qe = self.QE
        ros_cube.fwhm_nm = self.fwhm
        ros_cube.central_wavelengths = self.central_wave
        self.pub_cube.publish(ros_cube)


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
            HSI_MOSAIC.PushFrame(self.pipeline, self.frame)
            HSI_MOSAIC.GetCube(self.pipeline, self.cube, timeout_ms=1000)
            py_cube = HSI_COMMON.CubeAsArray(self.cube, BSQ=False)
            self.publish_cube(py_cube)
            rospy.sleep(0.001)
        # End of loop behavior        
        self.shutdown()

    def shutdown(self):
        '''
        Custom shutdown behavior
        '''
        rospy.loginfo(f"Cleaning up node for the {self.model.upper()} camera...")
        # HSI_COMMON.DeallocateFrame(self.frame)
        HSI_CAMERA.Pause(self.device)
        HSI_CAMERA.Stop(self.device)
        HSI_CAMERA.CloseDevice(self.device)
        HSI_MOSAIC.Pause(self.pipeline)
        HSI_MOSAIC.Stop(self.pipeline)
        HSI_COMMON.DeallocateCube(self.cube)
        HSI_COMMON.DeallocateFrame(self.frame)
        HSI_COMMON.DeallocateCube(self.cube)
        HSI_MOSAIC.DeallocateContext(self.context)


if __name__ == '__main__':
    rospy.init_node('DataCubeProcessor', anonymous=True)
    try:
        my_node = DataCubeGenerator()
        my_node.run()
    except rospy.ROSInterruptException:
        my_node.shutdown()
