#!/usr/bin/env python3

import rospy
import ros_numpy
from spectrometer_drivers.msg import Spectra
from hyper_drive.msg import MultipleDataCubes
from hyper_drive.msg import DataCube
import os
import numpy as np
import cv2 
class BagReader(object):
    def __init__(self):
        #self.home_dir = '/home/river/catkin_ws/Bag_Files/Numpy_Files/' + rospy.get_param('~experiment_name', '')
        self.home_dir = '/home/river/catkin_ws/src/hyper_drive/Images/Outdoor_Images/outdoor_03_10_23_rgb'
        #os.mkdir(self.home_dir)
        #self.sub_spectra = rospy.Subscriber(f'/combined_spectra', Spectra, self.spectra_callback)
        self.sub_datacubes = rospy.Subscriber(f'/syncronous_cubes', MultipleDataCubes, self.datacubes_callback)

    def spectra_callback(self, msg):
        self.spectra_data = msg.data
        #os.mkdir(self.home_dir + '/dark_spectra')
        np.save('/home/river/catkin_ws/src/hyper_drive/Bag_Files/Numpy_Files/dark_spectra', self.spectra_data)

    def datacubes_callback(self, msg):
        #dir_name = str(rospy.Time.now())
        #os.mkdir(self.home_dir + dir_name)
        
        ximea_cube = np.reshape(msg.cubes[0].data, (msg.cubes[0].width, msg.cubes[0].height, msg.cubes[0].lam))
        imec_cube = np.reshape(msg.cubes[1].data, (msg.cubes[1].width, msg.cubes[1].height, msg.cubes[1].lam))
        #rgb_cube = np.reshape(msg.cubes[2].data, (msg.cubes[2].width, msg.cubes[2].height, msg.cubes[2].lam))

        # cv2.imwrite(self.home_dir + dir_name + '/ximea.png', ximea_cube[:,:,0])
        # cv2.imwrite(self.home_dir + dir_name + '/imec.png', imec_cube[:,:,0])
        cv2.imwrite(self.home_dir + '/' + str(rospy.Time.now()) + '.png', cv2.cvtColor(ros_numpy.numpify(msg.im), cv2.COLOR_BGR2RGB))

        # np.save(self.home_dir + dir_name + '/ximea', msg.cubes[0].data)
        # np.save(self.home_dir + dir_name + '/imec', msg.cubes[1].data)
        # np.save(self.home_dir + dir_name + '/spectra', self.spectra_data)

if __name__ == '__main__':
    rospy.init_node('BagRead', anonymous=True)
    try:
        my_node = BagReader()
        rospy.spin()
    except rospy.ROSInterruptException:
        my_node.shutdown()