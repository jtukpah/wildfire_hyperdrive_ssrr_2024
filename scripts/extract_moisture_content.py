#!/usr/bin/env python3

import numpy as np
import rospy
from hyper_drive.msg import DataCube, MultipleDataCubes
import matplotlib.pyplot as plt
import matplotlib

font = {'family' : 'normal',
        'size'   : 16}

matplotlib.rc('font', **font)

vnir_white = np.load('/home/river/catkin_ws/src/hyper_drive/scripts/white_reference/vnir.npy').reshape((215,407,24))
swir_white = np.load('/home/river/catkin_ws/src/hyper_drive/scripts/white_reference/swir.npy').reshape((168, 211, 9))
vnir_lambda = [662,678,691,702,719,730,743,757,770,779,794,805,817,831,842,853,867,878,887,901,909,919,928,932]
swir_lambda = [1125,1140,1162,1192,1210,1298,1382,1465,1654]

def callback_hyper_spectral(msg: MultipleDataCubes):
    vnir: DataCube = msg.cubes[0]
    swir: DataCube = msg.cubes[1]
    vnir_data = np.array(vnir.data)
    swir_data = np.array(swir.data)
    vnir_data = vnir_data.reshape(vnir.width, vnir.height, len(vnir_data)//(vnir.width*vnir.height))
    swir_data = swir_data.reshape(swir.width, swir.height, len(swir_data)//(swir.width*swir.height))
    print(vnir_data[30,30,:].shape)
    vnir_reflectance = np.divide(vnir_data, vnir_white, where=vnir_white!=0)
    swir_reflectance = np.divide(swir_data, swir_white, where=swir_white!=0)
    plt.plot(vnir_lambda, vnir_reflectance[190,200,:])
    plt.title("Reflectance of selected leaf (VNIR)")
    plt.xlabel("Wavelength (nm)")
    plt.ylabel("Normalized Reflectance")
    plt.tight_layout()
    plt.savefig('vnir_r.png')
    plt.close()
    plt.plot(swir_lambda, swir_reflectance[160,100,:])
    plt.title("Reflectance of selected leaf (VNIR)")
    plt.xlabel("Wavelength (nm)")
    plt.ylabel("Normalized Reflectance")
    # plt.show()
    plt.savefig('swir_r.png')
    plt.tight_layout()
    plt.close()
    swir_channel_subtract = vnir_reflectance[:,:, 23] - vnir_reflectance[:,:,6]
    swir_channel_add = vnir_reflectance[:,:, 23] + vnir_reflectance[:,:,6]
    moisture = np.divide(swir_channel_subtract, swir_channel_add, where=swir_channel_add!=0)
    plt.title("Moisture Content")
    plt.imshow(moisture, cmap='viridis')
    plt.colorbar()
    # ax.set_yticklabels([])
    # ax.set_xticklabels([])
    plt.axis('off')
    plt.savefig("moisture.png")
    plt.close()
    while True:
        pass
    



if __name__ == '__main__':
    rospy.init_node('moisture_extractor')
    rospy.Subscriber('/synchronous_cubes', MultipleDataCubes, callback_hyper_spectral)
    rospy.spin()