#!/usr/bin/env python3
import cv2 as cv
import numpy as np

import PySimpleGUI as sg

from os import mkdir, chdir, scandir, environ
import copy

environ['PATH'] += "/home/river/catkin_ws/src/imec_driver/scripts/"
from get_calibration_images import CalibrationImages

# Authored by Gary Lvov
class Calibration():
    def __init__(self):
        self.camera_driver = CalibrationImages()
        self.create_folders()
        self.create_gui()
        
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
        chdir("../")

    def update_config(self):
        chdir("../") # Now in parent folder
        chdir("/scripts/matlab/")
        chdir("../../") # change back to parent folder

    def create_gui(self):
        img_scale = .5

        sg.theme("DarkPurple")

        layout = [[sg.Text("Gary and Ben's Sick Stereo Calibration Helper Tool", size=(40, 1), justification='center', font='Helvetica 20')],
              [sg.Image(filename='', key='cam1'), sg.Image(filename='', key='cam2')],
              [sg.Button('Save Image', size=(10, 1), font='Helvetica 14'),]]

        window = sg.Window("Gary and Ben's Camera Calibration Helper Tool",
                        layout, location=(800, 400))

        
        
        saved_count = 0
        
        while True:
            frame1, frame2 = self.camera_driver.grab_images()
            event, values = window.read(timeout=20)

            if event == 'Save Image':
                cv.imwrite(self.next_name + "/cam1/" + str(saved_count) + ".png", frame1)
                cv.imwrite(self.next_name + "/cam2/" + str(saved_count) + ".png", frame2)
                saved_count += 1
            
            elif event == sg.WIN_CLOSED:
                break

            imgbytes1 = cv.imencode('.png', frame1)[1].tobytes() 
            imgbytes2 = cv.imencode('.png', frame2)[1].tobytes()
            window['cam1'].update(data=imgbytes1)
            window['cam2'].update(data=imgbytes2)

if __name__ == '__main__':
    
    calib = Calibration()

