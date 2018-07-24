import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from test.csj.MainWindow import Ui_MainWindow
# import pyqtgraph as pg
# import numpy as np


class Main(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        print(self.Button1.text())
        self.Button1.clicked.connect(self.on_click)

    def on_click(self):
        print("asd")


app = QApplication(sys.argv)
# The QWidget widget is the base class of all user interface objects in PyQt5.
# We provide the default constructor for QWidget. The default constructor has no parent.
# A widget with no parent is called a window.
root = Main()
root.show()

# root.resize(320, 240)  # The resize() method resizes the widget.
# root.setWindowTitle("Hello, World!")  # Here we set the title for our window.
# root.show()  # The show() method displays the widget on the screen.

sys.exit(app.exec_())
