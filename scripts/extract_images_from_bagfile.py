#!/usr/bin/env python

import matplotlib.image
import numpy as np
import os
import ros_numpy
import rospy
import sys

from hyper_drive.msg import DataCube, MultipleDataCubes


# the index of the frame being processed
frame_number = 0

# saves the given 2d array as a JPG to the given directory
# np.array str -> None
def save_array_as_image(array, image_filepath):
    matplotlib.image.imsave(image_filepath, array)

# Reshape each of the datacubes and for each wavelength slice in each datacube save that slice as an image
# DataCube[] -> None
def process_datacubes(datacubes, img_dir, num_messages):
    for dc_index in range(len(datacubes)):
        dc = datacubes[dc_index]
        cube = np.reshape(dc.data, (dc.width, dc.height, dc.lam))
        for i in range(dc.lam):
            lam_img = cube[:, :, i]
            num_messages_digits = len(str(num_messages))
            frame_number_str = str(frame_number).zfill(num_messages_digits)
            image_filename = f'frame{frame_number_str}_hyperspectral_{dc_index}.jpg'
            image_filepath = os.path.join(img_dir, image_filename)
            save_array_as_image(lam_img, image_filepath)

# Save the RGB image to the given directory
# sensor_msgs/Image -> None
def process_rgb_img(rgb_msg, img_dir, num_messages):
    rgb_img = ros_numpy.numpify(rgb_msg)
    num_messages_digits = len(str(num_messages))
    frame_number_str = str(frame_number).zfill(num_messages_digits)
    image_filename = f'frame{frame_number_str}_rgb.jpg'
    image_filepath = os.path.join(img_dir, image_filename)
    save_array_as_image(rgb_img, image_filepath)


# saves each slice of the datacube (and the one rgb image) as a jpg
# MultipleDataCubes str Int -> None
def save_images_from_bagfile(message, img_dir, num_messages):
    datacubes = message.cubes
    rgb_img = message.im

    process_datacubes(datacubes, img_dir, num_messages)
    process_rgb_img(rgb_img, img_dir, num_messages)

    # this is the only place we update the frame number global variable (after we have finished processing the frame)
    global frame_number
    frame_number += 1


# Create a subscriber to the given topic which saves messages published to the topic
# as images in the given directory
# str str Int -> None
def listener(topic_name, img_dir, num_messages):
    rospy.init_node('listener', anonymous=False)

    callback = lambda message: save_images_from_bagfile(message, img_dir, num_messages)
    rospy.Subscriber(topic_name, MultipleDataCubes, callback)

    rospy.spin()

if __name__ == '__main__':
    script_name = sys.argv[0]
    topic_name = sys.argv[1]
    img_dir = sys.argv[2]
    num_messages = sys.argv[3]
    listener(topic_name, img_dir, int(num_messages))
