# -*- coding: utf-8 -*-
"""
Created on Thu Jul 19 15:35:15 2018

@author: Administrator
"""


# import subprocess
# import time
# import threading
#
#
# class LaserData(threading.Thread):
#     def __init__(self, laserData=None):
#         super().__init__()
#         self.laserData = laserData
#         self.l_recv += 100
#
#     def run(self):
#         while True:
#
#             self.laserData = self.l_recv
#
#             time.sleep(0.1)
#             print(self.laserData)
#
#
# if __name__ == '__main__':
#     LaserData().start()

import subprocess
import time
import threading


class LaserData(threading.Thread):
    def __init__(self, laserData=None):
        super().__init__()
        self.laserData = laserData
        self.a = 10
        print(self.a)

    def run(self):
        while True:
            self.a += 10
            print("laserdata", self.a)
            self.laserData.setData(self.a)
            time.sleep(1)



if __name__ == '__main__':
    LaserData().start()








