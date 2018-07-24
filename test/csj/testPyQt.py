# import initExample ## Add path to library (just for examples; you do not need this)
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import sys
from PyQt5.QtWidgets import *
from test.csj.MainWindow import Ui_MainWindow
import threading
import time


class thread_breath(threading.Thread):
    def __init__(self):
        super().__init__()
        self.i = 0

    def run(self):  # 以下就可以编写线程需要的功能
        while True:
            # print(data1)
            time.sleep(1)
            self.i = self.i + 1
            print(self.i)
            print(len(data1))
            p11.setData(data1)
            p12.setData(data2)
            p21.setData(data1)


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(MainWindow=self)
        self.show()


app = QApplication(sys.argv)
mainWin = MainWindow()
win = pg.GraphicsWindow(parent=mainWin.graphicsView, size=(1121, 491))
win.setWindowTitle('pyqtgraph example: crosshair')
label = pg.LabelItem(justify='right')
win.addItem(label)
p1 = win.addPlot(row=1, col=0)
p2 = win.addPlot(row=2, col=0)

region = pg.LinearRegionItem()
region.setZValue(10)
# Add the LinearRegionItem to the ViewBox, but tell the ViewBox to exclude this
# item when doing auto-range calculations.
p2.addItem(region, ignoreBounds=True)

# pg.dbg()
p1.setAutoVisible(y=True)


# create numpy arrays
# make the numbers large to show that the xrange shows data from 10000 to all the way 0
data1 = 10000 + 15000 * pg.gaussianFilter(np.random.random(size=100), 10) + 3000 * np.random.random(size=100)
data2 = 15000 + 15000 * pg.gaussianFilter(np.random.random(size=100), 10) + 3000 * np.random.random(size=100)
# data1 = np.array([])
# data2 = np.array([])

# print(data1)

p11 = p1.plot(data1, pen="r")
p21 = p1.plot(data2, pen="g")
p12 = p2.plot(data1, pen="w")


def update():
    region.setZValue(10)
    minX, maxX = region.getRegion()
    p1.setXRange(minX, maxX, padding=0)


region.sigRegionChanged.connect(update)


def updateRegion(window, viewRange):
    rgn = viewRange[0]
    region.setRegion(rgn)


p1.sigRangeChanged.connect(updateRegion)
region.setRegion([10, 20])
# cross hair
vLine = pg.InfiniteLine(angle=90, movable=False)
hLine = pg.InfiniteLine(angle=0, movable=False)
p1.addItem(vLine, ignoreBounds=True)
p1.addItem(hLine, ignoreBounds=True)


vb = p1.vb


def mouseMoved(evt):
    pos = evt[0]  # using signal proxy turns original arguments into a tuple
    if p1.sceneBoundingRect().contains(pos):
        mousePoint = vb.mapSceneToView(pos)
        index = int(mousePoint.x())
        if index > 0 and index < len(data1):
            label.setText("<span style='font-size: 12pt'>x=%0.1f,   <span style='color: red'>y1=%0.1f</span>,   <span style='color: green'>y2=%0.1f</span>" % (mousePoint.x(), data1[index], data2[index]))
        vLine.setPos(mousePoint.x())
        hLine.setPos(mousePoint.y())


proxy = pg.SignalProxy(p1.scene().sigMouseMoved, rateLimit=60, slot=mouseMoved)
# p1.scene().sigMouseMoved.connect(mouseMoved)


# Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        # thread_breath().start()
        app.exec_()

