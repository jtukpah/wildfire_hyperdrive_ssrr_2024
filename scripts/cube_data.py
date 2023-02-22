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
from hsi_driver.msg import DataCube
from hsi_driver.srv import adjust_param
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import logging

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
        # Setup logging to roslog location
        log_dir = rospkg.get_log_dir()
        rospy.loginfo(f"Logging camera info to {log_dir}")
        HSI_COMMON.InitializeLogger(log_dir, HSI_COMMON.LoggerVerbosity.LV_INFO)
        rospy.loginfo("Loading Context ...")

        
        # Setup callback for data
        self.ros_pack = rospkg.RosPack()
        self.model = rospy.get_param('~camera_model')
        self.frame_rate = rospy.get_param('~frame_rate', 60)
        self.integration_time = rospy.get_param('~integration_time', 10)
        self.param_server = rospy.Service('adjust_param', adjust_param, self.handle_adjust_param)
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
        self.pub_raw = rospy.Publisher(f'raw_data', Image, queue_size=10)
        self.pub_cube = rospy.Publisher(f'cube_data', DataCube, queue_size=10)
        # Load camera parameters
        self.parse_parameters()
        # Connect to the first available camera of the specified model type
        rospy.loginfo('looking for device:: {}'.format(self.dev_list[0]))
        self.device = HSI_CAMERA.OpenDevice(self.dev_list[0])
        HSI_CAMERA.SetRegionOfInterestArray(self.device, self.roi)
        rospy.loginfo(f"Region-of-Intereset Set to: {self.roi}")
        rospy.loginfo('Initializing Camera...')
        HSI_CAMERA.Initialize(self.device)
        # Get/Set Camera Configuration Parameters (example)
        self.c_params = HSI_CAMERA.GetConfigurationParameters(self.device)
        rospy.loginfo("C PARAMS>")
        rospy.loginfo(self.c_params)
        self.r_params = HSI_CAMERA.GetRuntimeParameters(self.device)
        rospy.loginfo(self.r_params)
        # Set the frame rate and exposure time before the camera starts
        # These values can be adjusted later through a callback
        self.r_params.frame_rate_hz = self.frame_rate
        self.r_params.exposure_time_ms = self.integration_time
        # Flip the frame of the IMEC camera to match housing pattern
        if self.model == 'imec':
            self.r_params['flip_vertical'] = False
        # Set these parameters on the device
        HSI_CAMERA.SetRuntimeParameters(self.device, self.r_params)
        rospy.loginfo("R PARAMS>")
        rospy.loginfo(self.r_params)
        # Explicit Allocate based on Output-Data-Format
        dataformat = HSI_CAMERA.GetOutputFrameDataFormat (self.device)
        self.frame = HSI_COMMON.AllocateFrame(dataformat)
        rospy.loginfo("Camera Output Data Format: {}".format(dataformat))
        self.setup_context()

    def setup_context(self) -> None:
        '''
        Load HSI Context files and prepare to run demosaicing pipeline
        '''
        dn_context = os.path.join(self.ros_pack.get_path('hsi_driver'),'config',self.model, 'context')
        #####################################################################
        version = HSI_MOSAIC.GetAPIVersion()
        rospy.loginfo(f'VERSION :: {version}')
        rospy.loginfo(Path(dn_context).absolute())
        assert Path(dn_context).exists()
        self.context = HSI_MOSAIC.LoadContext(dn_context)
        rospy.loginfo(f'Context = {self.context}')
        status = HSI_MOSAIC.ContextGetStatus(self.context)
        rospy.loginfo(status)
        self.pipeline = HSI_MOSAIC.Create(self.context)
        rospy.loginfo(self.pipeline)

        self.params = HSI_MOSAIC.GetConfigurationParameters(self.pipeline)
        self.params.spatial_median_filter_enable = False
        rospy.loginfo(f'Configuration Params : {self.params}')
        HSI_MOSAIC.SetConfigurationParameters(self.pipeline, self.params)
        rospy.loginfo('Initializing Pipeline')
        HSI_MOSAIC.Initialize(self.pipeline)

        outputdataformat = HSI_MOSAIC.GetOutputDataFormat(self.pipeline)
        rospy.loginfo(f'Output Data Format = {outputdataformat}')

        self.cube = HSI_COMMON.AllocateCube(outputdataformat)

        rospy.loginfo('Starting Pipeline')
        HSI_MOSAIC.Start(self.pipeline)


    def parse_parameters(self) -> None:
        '''
        Load parameter for camera from manufacturer provided XML file
        for publication in datacube messages
        '''
        # Get an instance of RosPack with the default search paths
        param_path = os.path.join(self.ros_pack.get_path('hsi_driver'),'config',f'{self.model}.xml')
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
        frame_time = 1/(req.frame_rate) * 1000
        rospy.loginfo(f'Incoming message: {adjust_param}')
        try:
            # Pause the camera
            if (self.integration_range[0] < req.integration_time < self.integration_range[1]) & (req.integration_time < frame_time):
                HSI_CAMERA.Pause(self.device)
                self.r_params.exposure_time_ms = req.integration_time
                self.r_params.frame_rate_hz = req.frame_rate
                rospy.loginfo(self.r_params)
                rospy.loginfo(f'Update Runtime Params: {HSI_CAMERA.SetRuntimeParameters(self.device, self.r_params)}')       
                HSI_CAMERA.Start(self.device)
                rospy.sleep(1)
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
        '''
        Publish the raw image pulled from the camera photodetector
        '''
        ros_image = ros_numpy.msgify(Image, raw, encoding="32FC1")
        self.pub_raw.publish(ros_image)

    def publish_cube(self, cube: np.ndarray):
        '''
        Publish hyperspectral datacubes
        '''
        # Mark that we've received a new cube
        ros_cube = DataCube()
        # Create header
        h = Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        ros_cube.header = h
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
