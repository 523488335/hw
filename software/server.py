import subprocess
import time
import threading


class ImageRecvThread(threading.Thread):
    def __init__(self, window=None):
        super().__init__()
        self.window = window

    def run(self):
        # server = socket.socket()
        #         # server.bind(("192.168.2.111", 12122))
        #         # server.listen(5)
        #         # c, addr = server.accept()
        #         # s = b''
        #         # end = b'EndImage'
        #         # starTime = round(time.time() * 1000)
        #         # while True:
        #         #     a = c.recv(1024)
        #         #     if end in a:
        #         #         # 返回s + a[:a.find(end)]
        #         #         data = s + a[:a.find(end)]
        #         #         if self.window is None:
        #         #             file = open('D:/picture/' + str(round(time.time() * 1000)) + '.jpg', "wb")
        #         #             file.write(data)
        #         #         else:
        #         #             self.window.setImage(data)
        #         #             # time.sleep(1)
        #         #             print("sleep")
        #         #
        #         #         s = a[a.find(end) + len(end):]
        #         #
        #         #         endTime = round(time.time() * 1000)
        #         #         print(str(endTime - starTime) + "ms")
        #         #         starTime = endTime
        #         #         continue
        #         #     s = s + a
        while True:
            start = time.time() * 1000
            # p0 = subprocess.Popen('adb shell screenrecord --bit-rate 2000000  --time-limit 10 /sdcard/test.mp4',
            #                       shell=True, stdout=subprocess.PIPE)
            p1 = subprocess.Popen('adb shell screencap -p', shell=True, stdout=subprocess.PIPE)
            p2 = subprocess.Popen("sed 's/\r$//'", shell=True, stdin=p1.stdout, stdout=subprocess.PIPE)
            if self.window is None:
                file = open('D:/picture/' + str(round(time.time() * 1000)) + '.jpg', "wb")
                file.write(p2.stdout.read())
            else:

                self.window.setImage(p2.stdout.read())
                # time.sleep(1)
                # print("sleep")
            end = time.time() * 1000
            # print(str(end - start) + "ms")


if __name__ == '__main__':
    ImageRecvThread().start()



