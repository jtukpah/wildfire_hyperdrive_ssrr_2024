#!/usr/bin/env python3
import os
import glob
import cv2
from PIL import Image
from numpy import asarray

base_path = '/home/river/Downloads/calibration_0003-20230207T233516Z-001/calibration_0003/cam1/'
save_path = '/home/river/Downloads/calibration_0003-20230207T233516Z-001/calibration_0003_resize/cam1'
for f in os.listdir(base_path):
    img = cv2.imread(os.path.join(base_path, f))
    print(img.shape)
    cv2.imwrite(os.path.join(save_path,f), img)
# # environ['PATH'] += "/home/river/catkin_ws/src/hsi_driver/scripts/calibration_0002/cam2"
# path1 = '/home/river/catkin_ws/src/hsi_driver/scripts/calibration_0002/cam2/'
# listing = os.listdir(path1)

# img = Image.open(path1 + '7.png')
# img1 = img.resize((426,340))
# num_array = asarray(img1)
# img2 = num_array[62:-61,9:-8]
# img3 = Image.fromarray(img2)
# img3.save("/home/river/catkin_ws/src/hsi_driver/scripts/calibration_0002/cam2/7.png")