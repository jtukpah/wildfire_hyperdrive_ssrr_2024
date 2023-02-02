#!/usr/bin/env python3

import os
import sys
import cv2 as cv
import rospy
import typing
import rospkg
import ros_numpy
import traceback
import numpy as np
from bs4 import BeautifulSoup
from numba import jit, prange
from imec_driver.msg import DataCube
from imec_driver.srv import adjust_param
from sensor_msgs.msg import Image
import gzip

class CubeDemosaicer(object):
    def __init__(self):
        # Setup callback for data
        self.model = rospy.get_param('cube_processor')['camera_model']
        self.parse_parameters()
        # Create publisher to send datacubes on
        self.pub = rospy.Publisher(f'cube_pub/{self.model}', DataCube, queue_size=10)
        # Subscribe to the raw data image
        self.sub = rospy.Subscriber(f'raw_pub/{self.model}', Image, self.cube_callback)

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

    def cube_callback(self, msg: Image):
        '''
        Callback function for hypercube data
        '''
        # Mark that we've received a new cube
        cube = ros_numpy.numpify(msg)
        cube = cube.astype(np.int16)
        if self.model == 'ximea':
            #cube = self.demosaic_cube(cube, (217,409,25), 1085, 2045, 5)
            print(cube.shape)
            cube = self.im2cube_sinc(cube, 5, cube.shape[0], cube.shape[1])
        elif self.model == 'imec':
            #cube = self.demosaic_cube(cube, (170,213,9), 510, 639, 3)
            cube = self.im2cube_sinc(cube, 3, cube.shape[0], cube.shape[1])
        else:
            rospy.loginfo('Unknown camera model')
        self.publish_cube(cube)

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

    # @staticmethod
    # @jit(nopython=True)
    # def demosaic_cube(data: np.ndarray, outpt_shape: np.array, use_in_width: np.int, use_in_height: np.int, mosaic_size: np.int) -> np.ndarray:
        # '''
        # Workaround method to demosaic image
        # '''
        # cube = np.zeros(outpt_shape, dtype=np.float32)
        # # Pixel array lookup
        # voxel_lookup = np.arange(0,mosaic_size**2,1).reshape((mosaic_size, mosaic_size))
        # for x in range(use_in_width):
        #     for y in range(use_in_height):
        #         # Add pixel value to cube
        #         use_x = x//mosaic_size
        #         use_y = y//mosaic_size
        #         band = voxel_lookup[(x%mosaic_size),(y%mosaic_size)]
        #         cube[use_x][use_y][band] = data[x][y]
        # return cube

    @staticmethod
    @jit(nopython=True)
    def demosaic_cube(im, radius):
        """
        Box filter with running average O(1)
        :param im: input image
        :param radius: radius of box kernel
        :return: box-filtered image
        """
        (rows, cols) = im.shape[:2]
        z = np.zeros_like(im)

        tile = [1] * im.ndim
        tile[0] = radius
        t = np.cumsum(im, 0)
        z[0:radius + 1, :, ...] = t[radius:2 * radius + 1, :, ...]
        z[radius + 1:rows - radius, :, ...] = t[2 * radius + 1:rows, :, ...] - t[0:rows - 2 * radius - 1, :, ...]
        z[rows - radius:rows, :, ...] = np.tile(t[rows - 1:rows, :, ...], tile) - t[rows - 2 * radius - 1:rows - radius - 1, :, ...]

        tile = [1] * im.ndim
        tile[1] = radius
        t = np.cumsum(z, 1)
        z[:, 0:radius + 1, ...] = t[:, radius:2 * radius + 1, ...]
        z[:, radius + 1:cols - radius, ...] = t[:, 2 * radius + 1: cols, ...] - t[:, 0: cols - 2 * radius - 1, ...]
        z[:, cols - radius: cols, ...] = np.tile(t[:, cols - 1:cols, ...], tile) - t[:, cols - 2 * radius - 1: cols - radius - 1, ...]
        return z
    
    def filter_guided_gray(self,im,guide,radius,smooth):
        """
        Guided filter for grayscale images
        :param im: input image
        :param guide: guidance image
        :param radius: window radius parameter
        :param smooth: regularization or smooth parameter
        :return: guided-filtered image
        """

        (rows, cols) = guide.shape
        N = self.demosaic_cube(np.ones([rows, cols]), radius)

        meanI = self.demosaic_cube(guide, radius) / N
        meanP = self.demosaic_cube(im, radius) / N
        corrI = self.demosaic_cube(guide * guide, radius) / N
        corrIp = self.demosaic_cube(guide * im, radius) / N
        varI = corrI - meanI * meanI
        covIp = corrIp - meanI * meanP

        a = covIp / (varI + smooth)
        b = meanP - a * meanI

        meanA = self.demosaic_cube(a, radius) / N
        meanB = self.demosaic_cube(b, radius) / N

        z = meanA * guide + meanB

        return z

    def im2cube_sinc_guided(self, im_raw, pattern, height, width):
        """
        Interpolated demosaicking with guided filter
        :param im_raw: raw input image
        :param pattern: integer dimension of the mosaic pattern (e.g. 4 or 5)
        :param height: height of valid image region
        :param width: width of valid image region
        :return:
        """
        cube = self.im2cube_sinc(im_raw, pattern, height, width)
        im_guide = cube[:, :, 0]
        cube = self.filter_guided_gray(cube, im_guide, 3, 0.01)

        return cube

    def im2cube_sinc(self, im_raw, pattern, height, width):
        """
        Interpolated demosaicking (Lanczos windowed sinc)
        :param im_raw: raw input image
        :param pattern: integer dimension of the mosaic pattern (e.g. 4 or 5)
        :param height: height of valid image region
        :param width: width of valid image region
        :return:
        """
        cube = np.zeros([height, width, pattern ** 2])

        band = 0
        for i in range(0, pattern):
            for j in range(0, pattern):
                im_sub = im_raw[i:(height + 1):pattern, j:(width + 1):pattern]
                im_sub = cv.resize(im_sub, (width, height), interpolation = cv.INTER_LANCZOS4)

                offset = [i / pattern, j / pattern]
                im_sub = self.filter_shift(im_sub, offset)

                cube[:, :, band] = im_sub[:height, :width]
                band += 1

        im_guide = cube[:, :, 0]

        return cube

    def filter_shift(sefl,im,offset):
        """
        Shift image with offset
        :param im: input image
        :param offset: translation shift
        :return: shifted image
        """

        rows, cols = im.shape
        M = np.float32([[1, 0, offset[1]], [0, 1, offset[0]]])
        z = cv.warpAffine(im, M, (cols, rows), cv.INTER_LANCZOS4)

        return z

    @staticmethod
    @jit(nopython=True)
    def perform_corrective_factors(data: np.ndarray) -> np.ndarray:
        '''
        Using the factors provided in the XML file calculate the final bands

        This operation should be optimized with numpy for simplicity
        '''
        # TODO
        return data

    def publish_cube(self, cube: np.ndarray) -> None:
        '''
        Create a data cube message and publish to topic
        '''
        print(f'CUBE SHAPE: {cube.shape}')
        cube = cube.astype(np.int16)
        print(cube.size * cube.itemsize)
        print(cube.dtype)
        print(cube.min())
        print(cube.max())
        ros_cube = DataCube()
        #flat_data = cube.flatten()
        #ros_cube.data = gzip.compress(flat_data)
        ros_cube.data = cube.flatten()
        ros_cube.width, ros_cube.height, ros_cube.lam = tuple(cube.shape)
        ros_cube.qe = self.QE
        ros_cube.fwhm_nm = self.fwhm
        ros_cube.central_wavelengths = self.central_wave

        self.pub.publish(ros_cube)

    def shutdown(self):
        '''
        Custom shutdown behavior
        '''
        return 

if __name__ == '__main__':
    rospy.init_node('CubeProcessor', anonymous=True)
    try:
        my_node = CubeDemosaicer()
        rospy.spin()
    except rospy.ROSInterruptException:
        my_node.shutdown()
