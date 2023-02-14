#!/usr/bin/env python3

import os
os.environ['PATH'] += os.pathsep + 'opt/imec/hsi-mosaic/bin'
import sys
import cv2 as cv
import rospy
import typing
import rospkg
import logging
import ros_numpy
import traceback
import numpy as np
from pathlib import Path
from bs4 import BeautifulSoup
from numba import jit, prange
from hsi_driver.msg import DataCube
from hsi_driver.srv import adjust_param
from sensor_msgs.msg import Image
import hsi_common as HSI_COMMON
import hsi_mosaic as HSI_MOSAIC
import hsi_camera as HSI_CAMERA


class CubeDemosaicer(object):
    def __init__(self):
        # Setup callback for data
        self.model = rospy.get_param('~camera_model')
        self.parse_parameters()
        # Create publisher to send datacubes on
        self.pub = rospy.Publisher(f'cube_pub/{self.model}', DataCube, queue_size=10)
        # Subscribe to the raw data image
        self.sub = rospy.Subscriber(f'raw_pub/{self.model}', Image, self.cube_callback)

    def setup_context(self) -> None:
        '''
        Load HSI Context files and prepare to run demosaicing pipeline
        '''
        dn_context = '/home/river/catkin_ws/src/hsi_driver/config/imec/context'
        #####################################################################

        formatter = logging.Formatter(fmt='%(asctime)s %(levelname)-6s :: %(message)s',
                                    datefmt='%Y-%m-%d %H:%M:%S')
        logger = logging.getLogger()
        for h in logger.handlers: h.setFormatter(formatter)
        logger.setLevel(logging.DEBUG)

        logging.info ('START MAIN.')

        version = HSI_MOSAIC.GetAPIVersion()
        logging.info (f'VERSION :: {version}')

        logging.info ("Starting Logger.")
        HSI_COMMON.InitializeLogger('logs', HSI_COMMON.LoggerVerbosity.LV_DEBUG)

        logging.info ("Loading Context ...")
        logging.info(Path(dn_context).absolute())
        assert Path(dn_context).exists()
        self.context = HSI_MOSAIC.LoadContext(dn_context)
        logging.info (f'Context = {self.context}')
        status = HSI_MOSAIC.ContextGetStatus(self.context)
        logging.info(status)


        #####################################################################
        ### Creating the first pipeline
        #####################################################################
        self.pipeline = HSI_MOSAIC.Create(self.context)
        logging.info(self.pipeline)

        self.params = HSI_MOSAIC.GetConfigurationParameters(self.pipeline)
        # params.spatial_resampling_width = 2045
        # params.spatial_resampling_height = 1085
        self.params.spatial_median_filter_enable = False
        logging.info(f'Configuration Params : {self.params}')
        HSI_MOSAIC.SetConfigurationParameters(self.pipeline, self.params)
        logging.info('Initializing Pipeline')
        HSI_MOSAIC.Initialize(self.pipeline)

        outputdataformat = HSI_MOSAIC.GetOutputDataFormat(self.pipeline)
        logging.info(f'Output Data Format = {outputdataformat}')

        self.cube = HSI_COMMON.AllocateCube(outputdataformat)

        logging.info('Starting Pipeline')
        HSI_MOSAIC.Start(self.pipeline)

    def parse_parameters(self) -> None:
        '''
        Load parameter for camera from manufacturer provided XML file
        for publication in datacube messages
        '''
        # Get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        param_path = os.path.join(rospack.get_path('hsi_driver'),'config',f'{self.model}.xml')
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

    def cube_callback(self, msg: Image):
        '''
        Callback function for hypercube data
        '''
        # Mark that we've received a new cube
        cube = ros_numpy.numpify(msg)
        HSI_MOSAIC.PushFrame(self.pipeline, self.frame)
        HSI_MOSAIC.GetCube(self.pipeline, cube, timeout_ms=1000)
        py_cube = HSI_COMMON.CubeAsArray(cube)
        ros_cube = DataCube()
        ros_cube.data = noiseless_channel.flatten()
        ros_cube.width, ros_cube.height, ros_cube.lam = tuple(noiseless_channel.shape)
        ros_cube.qe = self.QE
        ros_cube.fwhm_nm = self.fwhm
        ros_cube.central_wavelengths = self.central_wave

        self.pub.publish(ros_cube)

    def shutdown(self):
        '''
        Custom shutdown behavior
        '''
        HSI_MOSAIC.Pause(self.pipeline)
        HSI_MOSAIC.Stop(self.pipeline)
        HSI_COMMON.DeallocateCube(self.cube)
        logging.info("Cleanup ...")
        HSI_COMMON.DeallocateFrame(self.frame)
        HSI_COMMON.DeallocateCube(self.cube)
        HSI_MOSAIC.DeallocateContext(self.context)

if __name__ == '__main__':
    rospy.init_node('CubeProcessor', anonymous=True)
    try:
        my_node = CubeDemosaicer()
        rospy.spin()
    except rospy.ROSInterruptException:
        my_node.shutdown()
