#!/usr/bin/env python3

import rospy
from hyper_drive.msg import MultipleDataCubes
import os
import cv2
import ros_numpy
import numpy as np
from spectrometer_drivers.msg import Spectra

class ImageReader(object):
    def __init__(self):

        self.sub_datacubes = rospy.Subscriber(f'/synchronous_cubes', MultipleDataCubes, self.datacubes_callback)
        self.sub_spectra = rospy.Subscriber(f'/combined_spectra', Spectra, self.spectra_callback)

        self.img_num = 0
        self.bag_file_name = 'olin_collection_05_16_23_parcel_b_v2'

    def spectra_callback(self, msg):

        self.spectra_data = msg.data

    def datacubes_callback(self, msg):

        image = ros_numpy.numpify(msg.im)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        cv2.imwrite(f'/home/river/catkin_ws/src/hyper_drive/ImageData/{self.bag_file_name}/RGB/Image_{self.img_num}.jpg', image)

        np.save(f'/home/river/catkin_ws/src/hyper_drive/ImageData/{self.bag_file_name}/ximea/Cube_{self.img_num}', msg.cubes[0].data)
        np.save(f'/home/river/catkin_ws/src/hyper_drive/ImageData/{self.bag_file_name}/imec/Cube_{self.img_num}', msg.cubes[1].data)
        np.save(f'/home/river/catkin_ws/src/hyper_drive/ImageData/{self.bag_file_name}/spectra/spectra_{self.img_num}', self.spectra_data)
        
        self.img_num += 1

if __name__ == '__main__':
    rospy.init_node('ImageRead', anonymous=True)

    try:
        my_node = ImageReader()
        rospy.spin()

    except rospy.ROSInterruptException:
        my_node.shutdown()