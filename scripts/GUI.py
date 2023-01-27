#!/usr/bin/env python

from PyQt5 import QtCore, QtGui
from PyQt5 import QtWidgets

class Widget(QtWidgets.QWidget):

    def __init__(self, parent=None):
        super(Widget, self).__init__(parent)
        self.resize(640,480)
        self.layout = QtWidgets.QVBoxLayout(self)

        self.scene = QtWidgets.QGraphicsScene(self)
        self.view = QtWidgets.QGraphicsView(self.scene)
        self.layout.addWidget(self.view)

        self.image = QtWidgets.QGraphicsPixmapItem()
        self.scene.addItem(self.image)
        self.view.centerOn(self.image)

        self._images = [
            QtGui.QPixmap('/home/river/Pictures/Screenshot from 2023-01-17 17-29-56.png'),
            QtGui.QPixmap('/home/river/Pictures/Screenshot from 2023-01-18 12-45-07.png'),
            QtGui.QPixmap('/home/river/Pictures/Screenshot from 2023-01-18 13-43-04.png'),
            QtGui.QPixmap('/home/river/Pictures/Screenshot from 2023-01-19 09-14-33.png'),
            QtGui.QPixmap('/home/river/Pictures/Screenshot from 2023-01-20 12-28-59.png')
        ]

        self.slider = QtWidgets.QSlider(self)
        self.slider.setOrientation(QtCore.Qt.Horizontal)
        self.slider.setMinimum(0)
        # max is the last index of the image list
        self.slider.setMaximum(len(self._images)-1)
        self.layout.addWidget(self.slider)

        # set it to the first image, if you want.
        self.slidermove(0)

        self.slider.valueChanged.connect(self.slidermove)

    def slidermove(self, val):
        print("Slider moved to:", val)
        try:
            self.image.setPixmap(self._images[val])
        except IndexError:
            print("Error: No image at index", val)

if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    w = Widget()
    w.show()
    w.raise_()
    app.exec_()