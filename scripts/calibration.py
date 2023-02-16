#!/usr/bin/env python3

import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
import rospy
import ros_numpy

import PySimpleGUI as sg
from hsi_driver.msg import DataCube
from sensor_msgs.msg import Image

from os import mkdir, chdir, scandir, environ
import copy


# Authored by Gary Lvov
class Calibration():
    def __init__(self):
        self.frame1 = []
        self.frame2 = []
        self.frame3 = []

        self.ximea_sub = rospy.Subscriber('/ximea/cube_data', DataCube, self.callback_ximea)
        self.imec_sub = rospy.Subscriber('/imec/cube_data', DataCube, self.callback_imec)
        self.alvium_sub = rospy.Subscriber('/camera/image_raw', Image, self.callback_alvium)

        self.create_folders()
        self.create_gui()
        
    def callback_ximea(self, msg):
        #recontructs data cube
        ximea_cube = np.reshape(msg.data, (msg.width, msg.height, msg.lam))
        self.frame1 = ximea_cube[:, :, 0]

    def callback_imec(self, msg):
        #reconstructs data cube
        imec_cube = np.reshape(msg.data, (msg.width, msg.height, msg.lam))
        self.frame2 = imec_cube[:, :, 0]

    def callback_alvium(self, msg):
        self.frame3 = ros_numpy.numpify(msg)
        print(self.frame3.shape)

    def create_folders(self):
        try:
            mkdir("calibration_000")
        except FileExistsError:
            print("Calibrations exist in this directory, adding newest calibration in new folder")

        files = [x.path for x in scandir(".") if x.is_dir()]

        files = list(filter(lambda x: x[:13] == "./calibration",  files[::-1]))
        highest = max(files[-3:])
        highest = int(highest[-3:])
        next = highest + 1
        self.next_name = "calibration_000" + str(next)
        mkdir(self.next_name)
        chdir(self.next_name)
        mkdir("cam1")
        mkdir("cam2")
        mkdir("cam3")
        chdir("../")

    def update_config(self):
        chdir("../") # Now in parent folder
        chdir("/scripts/matlab/")
        chdir("../../") # change back to parent folder

    def create_gui(self):
        img_scale = .5

        sg.theme("DarkPurple")

        layout = [[sg.Text("Gary and Ben's Sick Stereo Calibration Helper Tool", size=(40, 1), justification='center', font='Helvetica 20')],
              [sg.Image(filename='', key='cam1'), sg.Image(filename='', key='cam2'), sg.Image(filename='', key='cam3')],
              [sg.Button('Save Image', size=(10, 1), font='Helvetica 14'),]]

        window = sg.Window("Gary and Ben's Camera Calibration Helper Tool",
                        layout, location=(800, 400))

        
        
        saved_count = 0
        
        while True:
            event, values = window.read(timeout=20)

            if event == 'Save Image':
                cv.imwrite(self.next_name + "/cam1/" + str(saved_count) + ".png", frame1)
                cv.imwrite(self.next_name + "/cam2/" + str(saved_count) + ".png", self.frame2)
                cv.imwrite(self.next_name + "/cam2/" + str(saved_count) + ".png", self.frame3)
            
                saved_count += 1
            
            elif event == sg.WIN_CLOSED:
                break

            imgbytes1 = cv.imencode('.png', self.frame1)[1].tobytes() 
            imgbytes2 = cv.imencode('.png', self.frame2)[1].tobytes()
            imgbytes3 = cv.imencode('.png', self.frame3)[1].tobytes()
            window['cam1'].update(data=imgbytes1)
            window['cam2'].update(data=imgbytes2)
            window['cam3'].update(data=imgbytes3)

if __name__ == '__main__':
    rospy.init_node('Calibration_Imgs', anonymous=True)
    try:
        my_node = Calibration()
        rospy.spin()
    except rospy.ROSInterruptException:
        my_node.shutdown()

