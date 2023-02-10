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

class Widget(QtWidgets.QWidget):

    def __init__(self, parent=None):
        super(Widget, self).__init__(parent)
        # Current wavelength to display
        self.lam = 0
        # Flag to set if new information is available
        self.new_cube = False
        # Placeholder for hypercube data
        self.cube = np.array([])
        # Placeholder ofr single channel grayscale image
        self.img = np.array([])
        # Set the initial size of the window
        self.resize(1000, 1000)
        self.layout = QtWidgets.QVBoxLayout(self)
        self.scene = QtWidgets.QGraphicsScene(self)
        self.view = QtWidgets.QGraphicsView(self.scene)
        self.layout.addWidget(self.view)
        # Visualization window for single channel image
        self.image = QtWidgets.QGraphicsPixmapItem()
        # Placeholder for histogram plot
        self.histo_image = QtWidgets.QGraphicsPixmapItem()
        self.scene.addItem(self.image)
        self.view.centerOn(self.image)
        self.scene.addItem(self.histo_image)
        self.histo_image.moveBy(450, -100)


        self.drop_box = QtWidgets.QComboBox(self)
        self.drop_box.addItem('[Choose option]')
        self.drop_box.addItem('ximea')
        self.drop_box.addItem('imec')
        self.drop_box.addItem('combined')
        self.drop_box.adjustSize()

        self.drop_box.currentTextChanged.connect(self.handle_drop_box)
        self.cam_model = self.drop_box.currentText()

        # Create and prepare the slider
        self.get_slider()
        self.label = QtWidgets.QLabel('0', self)
        self.label.setAlignment(QtCore.Qt.AlignCenter |
                                QtCore.Qt.AlignVCenter)
        self.label.setMinimumWidth(80)
        self.layout.addWidget(self.slider)
        self.layout.addWidget(self.label)
        self.slider.valueChanged.connect(self.slidermove)

        # Create timer to regularly update the GUI
        timer = QTimer(self)
        timer.timeout.connect(self.update_gui)
        timer.start(1)
        
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

    def convert_nparray_to_QPixmap(self, img, grayscale = False):
        '''
        Take a Numpy representation of an image and convert it to a QPixmap
        '''
        if grayscale:
            w,h = img.shape
            img =  cv.cvtColor(img,cv.COLOR_GRAY2RGB)
        else:
            #print(img.shape)
            w,h,ch = img.shape
        # Convert resulting image to pixmap
        qimg = QImage(img.data, h, w, 3*h, QImage.Format_RGB888) 
        qpixmap = QPixmap(qimg)

        return qpixmap


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
            self.histo_image.setPixmap(self.convert_nparray_to_QPixmap(image, grayscale=False))
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
        try:
            if self.cube.size == 0:
                #print('NO CUBE!')
                return
            if self.new_cube == False and override == False:
                return
            slice_image = self.cube[:, :, self.lam]
            self.img = slice_image
            img = self.rescale_image(slice_image)
            self.img = img
            self.image.setPixmap(self.convert_nparray_to_QPixmap(self.img, grayscale=True))
            self.new_cube = False
        except IndexError:
            print("Error: No image at index", self.lam)

    def slidermove(self, val):
        '''
        Callback function for moments when the slider moves
        '''
        self.label.setText(str(val))
        self.lam = val
        self.update_image(override=True)

    def handle_drop_box(self, val):
        '''
        Decide which topic panel should subscribe to and render
        '''
        self.cam_model = val
        try:
            self.data_sub.unregister()
        except:
            pass
        self.data_sub = rospy.Subscriber('/{}/cube_data'.format(self.cam_model), DataCube, self.callback)
        # Max is the last index of the image list
        if self.cam_model == 'ximea':
            self.slider.setMaximum(23)
        elif self.cam_model == 'imec':
            self.slider.setMaximum(8)

    def get_slider(self):
        '''
        Create a new slider as a class member variable
        '''
        self.slider = QtWidgets.QSlider(self)
        self.slider.setOrientation(QtCore.Qt.Horizontal)
        self.slider.setMinimum(0)
     
    
def timer_callback(event):
    '''
    Allow ROS to check for new messages occassionally
    '''
    rospy.spin() # start the ros spin here

if __name__ == "__main__":
    rospy.init_node('GUI', anonymous=True)
    # Create the main window
    app = QtWidgets.QApplication([])
    w = Widget()
    w.show()
    w.raise_()
    # Start the timer for ROS to regularly check for new messages
    timer = rospy.Timer(rospy.Duration(0.01), timer_callback) # start the ros spin after 1 s
    app.exec_()

    try:
        my_node = Widget()
        my_node.run()
    except rospy.ROSInterruptException:
        my_node.shutdown()