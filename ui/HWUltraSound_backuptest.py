# import initExample ## Add path to library (just for examples; you do not need this)
import sys
import threading
import time
import numpy as np
from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import *
from pyqtgraph.Qt import QtGui, QtCore
from PyQt5.QtCore import QTimer

from software.server import ImageRecvThread
import pyqtgraph as pg
import matplotlib.pyplot as plt
import threading
# from laser_data import LaserData
from ui.LaserData import LaserData


class UltraSoundView(QWidget):
    def __init__(self, parent=None):
        super(UltraSoundView, self).__init__(parent)
        with open('C:/Users/Administrator/Desktop/breast.png','rb') as f:
            img = f.read()
        image = QtGui.QImage.fromData(img)
        self.pixmap = QtGui.QPixmap.fromImage(image)

    def setImage(self, data):
        image = QtGui.QImage.fromData(data)
        self.pixmap = QtGui.QPixmap.fromImage(image)

    def paintEvent(self, event):
        # print("paintevent")
        painter = QtGui.QPainter()
        painter.begin(self)
        brush = QtGui.QBrush(self.pixmap)
        painter.setBrush(brush)
        painter.drawRect(0, 0, 1000, 800)
        painter.end()
        time.sleep(0.05)
        self.update()


class LiveDataView(QWidget):
    def __init__(self):
        super(LiveDataView, self).__init__()
        vLayout = QVBoxLayout(self)
        self.breastDataView = BreastDataView()
        vLayout.addWidget(self.breastDataView)
        vLayout.setStretchFactor(self.breastDataView, 1)


class BreastDataView(QGraphicsView):
    def __init__(self, parent=None):
        super(BreastDataView, self).__init__(parent)
        self.vector = []
        plotWidgetOne = pg.PlotWidget()
        layout = QGridLayout(self)
        self.setLayout(layout)
        layout.addWidget(plotWidgetOne, 0, 1, 3, 1)
        self.plotDataItem = pg.PlotDataItem([1, 2])
        self.plotDataItem.setData(self.vector)
        plotWidgetOne.addItem(self.plotDataItem)
        self.a = 10
        self.fun_timer()

    def setData(self, data):
        self.vector.append(data)
        print("setdata")
        # self.update()
        # self.a = self.vector
        # self.plotDataItem.setData(self.vector)
        # plotWidgetOne.addItem(self.plotDataItem)
        # print("setData")
        # print(self.vector)

    def fun_timer(self):
        # self.a = self.a + 10
        # self.vector.append(self.a)

        # print(self.__hash__(), self.vector)
        self.plotDataItem.setData(self.vector)

        timers = threading.Timer(0.1, self.fun_timer)
        timers.start()


# class BreastDataViewTest(object):
#     def __init__(self):
#         print("breastdataviewtest")
#         self.breastDataView = BreastDataView()


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setupUI()

    def setupUI(self):
        self.setObjectName("MainWindow")
        self.setGeometry(100, 100, 1000, 800)
        self.tabWidget = QTabWidget(self)
        self.tabWidget.setGeometry(0, 0, 1000, 800)
        self.ultraSoundGraphic = UltraSoundView()
        self.tabWidget.addTab(self.ultraSoundGraphic, "实时渲染")
        self.liveDataView = LiveDataView()
        self.tabWidget.addTab(self.liveDataView, "呼吸数据曲线")


if __name__ == '__main__':

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    laserData = LaserData(window.liveDataView.breastDataView)
    laserData.start()

    timer = threading.Timer(0.1, window.liveDataView.breastDataView.fun_timer)
    timer.start()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        app.exec_()
