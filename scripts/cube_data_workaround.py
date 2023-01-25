#!/usr/bin/env python3

import os
# NOTE: put this as very first action.
#       DLL-loading Could be impacted by loading other libs first ...
os.environ['PATH'] += os.pathsep + r'/opt/imec/hsi-mosaic/bin'

# only needed for python 3.8 (and above?)
try:
    os.add_dll_directory(r'/opt/imec/hsi-mosaic/python_apis')
except:
    pass
import rospy
import sys
import logging
import numpy as np

from pathlib import Path

sys.path.append('/opt/imec/hsi-mosaic/python_apis')

import hsi_common as HSI_COMMON
import hsi_camera as HSI_CAMERA

if __name__ == '__main__':
    formatter = logging.Formatter(fmt='%(asctime)s %(levelname)-6s :: %(message)s',
                                  datefmt='%Y-%m-%d %H:%M:%S')
    rospy.loginfo ('START MAIN.')

    rospy.loginfo ("Starting Logger.")
    HSI_COMMON.InitializeLogger('logs', HSI_COMMON.LoggerVerbosity.LV_DEBUG)

    # Set up the internal HSI rospy.log infrastructure
    HSI_COMMON.InitializeLogger('.', HSI_COMMON.LoggerVerbosity.LV_INFO)

    # look for connected cameras
    dev_list = HSI_CAMERA.EnumerateConnectedDevices(manufacturer=HSI_CAMERA.Manufacturer.EM_ALL)
    rospy.loginfo('return number of devices = {}'.format(len(dev_list)))

    # and connect to the first one
    rospy.loginfo('looking for device:: {}'.format(dev_list[0]))
    device = HSI_CAMERA.OpenDevice(dev_list[0])

    rospy.loginfo('Initializing Camera...')
    HSI_CAMERA.Initialize(device)

    # Get/Set Camera Runtime Parameters (example)
    r_params = HSI_CAMERA.GetRuntimeParameters(device)
    rospy.loginfo("Camera Runtime Parameters: {}".format(r_params))
    HSI_CAMERA.SetRuntimeParameters(device, r_params)

    # Implicit Allocate for Camera
    frame = HSI_CAMERA.AllocateFrameForCamera(device)
    rospy.loginfo(f"Camera Output Data Format: {frame.format}")


    # the device is ready: we can start it now ...
    HSI_CAMERA.Start(device)
    # Send a Software Trigger to the camera and grab the Frame
    HSI_CAMERA.Trigger(device)
    HSI_CAMERA.AcquireFrame(device, frame=frame)
    rospy.loginfo(frame)

    tmp = HSI_COMMON.FrameAsArray(frame) # internally convert frame to numpy array
    rospy.loginfo(frame.format)
    rospy.loginfo('Acquired Frame :: {}'.format(tmp.shape))

    print(type(tmp.shape))
    print(tmp.shape)

    ### TEMPORARY WORKAROUND TO MAKE A CUBE
    cube = np.zeros((217,409,25))
    for x in range(1085):
        for y in range(2045):
            # Add pixel value to cube
            use_x = x//5
            use_y = y//5
            band = (x%5)*(y%5)
            cube[use_x][use_y][band] = tmp[x][y].copy()
    print(f'Data cube shape: {cube.shape}')
    print(f'Data type {type(cube)}')

    rospy.loginfo("Cleanup ...")
    HSI_COMMON.DeallocateFrame(frame)
    HSI_CAMERA.Pause(device)
    HSI_CAMERA.Stop(device)
    HSI_CAMERA.CloseDevice(device)

    rospy.loginfo ('MAIN DONE.')
