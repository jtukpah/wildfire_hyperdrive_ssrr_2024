#!/usr/bin/env python3

import rospy
from spectrometer_drivers.msg import Spectra
from hyper_drive.msg import MultipleDataCubes
from hyper_drive.msg import DataCube
import os
import numpy as np

class BagReader(object):
    def __init__(self):
        self.home_dir = '/home/river/catkin_ws/Bag_Files/Numpy_Files/' + rospy.get_param('~experiment_name', '')
        os.mkdir(self.home_dir)
        self.sub_spectra = rospy.Subscriber(f'/combined_spectra', Spectra, self.spectra_callback)
        self.sub_datacubes = rospy.Subscriber(f'/syncronous_cubes', MultipleDataCubes, self.datacubes_callback)

    def spectra_callback(self, msg):
        self.spectra_data = msg.data

    def datacubes_callback(self, msg):
        dir_name = '/' + str(rospy.Time.now())
        os.mkdir(self.home_dir + dir_name)
        np.save(self.home_dir + dir_name + '/ximea', msg.cubes[0].data)
        np.save(self.home_dir + dir_name + '/imec', msg.cubes[1].data)
        np.save(self.home_dir + dir_name + '/spectra', self.spectra_data)

if __name__ == '__main__':
    rospy.init_node('BagRead', anonymous=True)
    try:
        my_node = BagReader()
        rospy.spin()
    except rospy.ROSInterruptException:
        my_node.shutdown()
