#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
import matplotlib.pyplot as plt
from imec_driver.msg import DataCube
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from sensor_msgs.msg import *
from std_msgs.msg import *
from cv_bridge import CvBridge

class HSI_HIST(QtWidgets.QWidget):
    
    def __init__(self, parent=None):
        super(HSI_HIST, self).__init__(parent)

        self.br = CvBridge()
        # Current wavelength to display
        self.lam = 0
        # Flag to set if new information is available
        self.new_cube = False
        # Placeholder for hypercube data
        self.cube = np.array([])
        # Placeholder ofr single channel grayscale image
        self.img = np.array([])
        
        self.cam_model = rospy.get_param('~camera_model')

        #get cam model
        self.model_sub = rospy.Subscriber('/gui/vis_cam', String, self.callback_model)

        #get slider num
        self.slider_num = rospy.Subscriber('/gui/bond_num', Int8, self.callback_slider)

        #get cube data
        self.data_sub = rospy.Subscriber('/{}/cube_data'.format(self.cam_model), DataCube, self.callback)

        #publish image
        self.pub_img = rospy.Publisher('/gui/current_im', Image, queue_size=10)

        #publish histogram
        self.pub_hist = rospy.Publisher('/gui/current_hist', Image, queue_size=10)

        # Create timer to regularly update the GUI
        timer = QTimer(self)
        timer.timeout.connect(self.update_gui)
        timer.start(1)
        
    def callback_model(self, msg):
        self.cam_model = msg.data

    def callback_slider(self, msg):
        self.lam = msg.data

    def callback(self, msg):
        '''
        Callback function for hypercube data
        '''
        # Mark that we've received a new cube
        self.new_cube = True
        self.cube = np.reshape(msg.data, (msg.width, msg.height, msg.lam))

    def rescale_image(self, arr):
        '''
        Rescale image to 8-bit pixels for display with PyQT
        '''
        return ((arr) * (1/((arr.max())) * 255)).astype('uint8')

    def update_histogram(self):
        '''
        Redraw the histogram on the main panel
        '''
        # configure and draw the histogram figure
        if self.img.size > 0:
            histogram, bin_edges = np.histogram(self.cube[:, :, self.lam], bins=50)
            fig = plt.figure()
            canvas = FigureCanvas(fig)
            ax = fig.gca()
            ax.set_title("Grayscale Histogram")
            ax.set_xlabel("grayscale value")
            ax.set_ylabel("pixel count")
            ax.plot(bin_edges[0:-1], histogram, color="yellow")
            ax.fill_between(bin_edges[0:-1], histogram, color="purple")
            canvas.draw()
            image = np.frombuffer(canvas.tostring_rgb(), dtype='uint8').reshape(fig.canvas.get_width_height()[::-1] + (3,))
            self.pub_hist.publish(self.br.cv2_to_imgmsg(image))
            plt.close()

    def update_gui(self):
        '''
        Callback function to update the GUI elements periodically
        '''
        self.update_image()
        self.update_histogram()

    def update_image(self, override = False):
        '''
        Update the value of the single channel image
        '''
        if self.cube.size == 0:
            #print('NO CUBE!')
            return
        if self.new_cube == False and override == False:
            return
        slice_image = self.cube[:, :, self.lam]
        self.img = slice_image
        img = self.rescale_image(slice_image)
        self.img = img
        self.pub_img.publish(self.br.cv2_to_imgmsg(self.img))
        self.new_cube = False
        
def timer_callback(event):
    '''
    Allow ROS to check for new messages occassionally
    '''
    rospy.spin() # start the ros spin here

if __name__ == "__main__":
    rospy.init_node('HSIProcessor', anonymous=True)
    app = QtWidgets.QApplication([])
    w = HSI_HIST()
    w.raise_()
    # Start the timer for ROS to regularly check for new messages
    timer = rospy.Timer(rospy.Duration(0.01), timer_callback) # start the ros spin after 1 s
    app.exec_()

    try:
        my_node = HSI_HIST()
        my_node.run()
    except rospy.ROSInterruptException:
        my_node.shutdown()