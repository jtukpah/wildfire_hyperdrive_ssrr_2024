#!/usr/bin/env python

from PyQt5 import QtCore, QtGui
from PyQt5 import QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
import rospy
from imec_driver.msg import DataCube
import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv

class Widget(QtWidgets.QWidget):

    def __init__(self, parent=None):
        super(Widget, self).__init__(parent)

        self.lam = 0
        self.new_cube = False
        self.cube = np.array([])
               
        self.resize(640,480)
        self.layout = QtWidgets.QVBoxLayout(self)

        self.scene = QtWidgets.QGraphicsScene(self)
        self.view = QtWidgets.QGraphicsView(self.scene)
        self.layout.addWidget(self.view)

        self.image = QtWidgets.QGraphicsPixmapItem()
        self.scene.addItem(self.image)
        self.view.centerOn(self.image)

        self.drop_box = QtWidgets.QComboBox(self)
        self.drop_box.addItem('[Choose option]')
        self.drop_box.addItem('ximea')
        self.drop_box.addItem('imec')
        self.drop_box.adjustSize()

        self.drop_box.currentTextChanged.connect(self.handle_drop_box)
        self.cam_model = self.drop_box.currentText()

        self.get_slider()

        #slider label
        self.label = QtWidgets.QLabel('0', self)
        self.label.setAlignment(QtCore.Qt.AlignCenter |
                                QtCore.Qt.AlignVCenter)
        self.label.setMinimumWidth(80)

        self.layout.addWidget(self.slider)
        self.layout.addWidget(self.label)

        self.slider.valueChanged.connect(self.slidermove)#dropdown box
        
        timer = QTimer(self)
        timer.timeout.connect(self.update_image)
        timer.start(10)

    def callback(self, msg):
        self.new_cube = True
        self.cube = np.reshape(msg.data, (msg.width, msg.height, msg.lam))
        self.cube = self.cube.astype(np.int)

    def rescale_image(self, arr):
        return ((arr) * (1/((arr.max())) * 255)).astype('uint8')

    def convert_nparray_to_QPixmap(self,img):
        w,h = img.shape
        # Convert resulting image to pixmap
        img =  cv.cvtColor(img,cv.COLOR_GRAY2RGB)
        qimg = QImage(img.data, h, w, 3*h, QImage.Format_RGB888) 
        qpixmap = QPixmap(qimg)

        return qpixmap

    def update_image(self, override=False):
        try:
            if self.cube.size == 0:
                #print('NO CUBE!')
                return
            if self.new_cube == False and override == False:
                return
            slice_image = self.cube[:, :, self.lam]
            img = self.rescale_image(slice_image)
            self.image.setPixmap(self.convert_nparray_to_QPixmap(img))
            self.new_cube = False
        except IndexError:
            print("Error: No image at index", self.lam)

    def slidermove(self, val):
        self.label.setText(str(val))
        self.lam = val
        self.update_image(override=True)

    def handle_drop_box(self, val):
        self.cam_model = val
        try:
            self.data_sub.unregister()
        except:
            pass
        self.data_sub = rospy.Subscriber('/cube_pub/{}'.format(self.cam_model), DataCube, self.callback)
        # max is the last index of the image list
        if self.cam_model == 'ximea':
            self.slider.setMaximum(24)
        elif self.cam_model == 'imec':
            self.slider.setMaximum(8)

    def get_slider(self):
        self.slider = QtWidgets.QSlider(self)
        self.slider.setOrientation(QtCore.Qt.Horizontal)
        self.slider.setMinimum(0)
     
    
def timer_callback(event):
    global timer
    rospy.spin() # start the ros spin here
    timer.shutdown()

if __name__ == "__main__":
    rospy.init_node('GUI', anonymous=True)

    app = QtWidgets.QApplication([])
    w = Widget()
    w.show()
    w.raise_()

    # start ros spin
    timer = rospy.Timer(rospy.Duration(0.1), timer_callback) # start the ros spin after 1 s
    app.exec_()