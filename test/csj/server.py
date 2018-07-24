import socket
import time
import threading


class ImageRecvThread(threading.Thread):
    def __init__(self):
        super().__init__()

    def run(self):
        server = socket.socket()
        server.bind(("192.168.0.106", 12122))
        server.listen(5)
        c, addr = server.accept()
        s = b''
        end = b'EndImage'
        starTime = round(time.time() * 1000)
        while True:
            a = c.recv(1024)
            if end in a:
                # 返回s + a[:a.find(end)]
                file = open('F:/' + str(round(time.time() * 1000)) + '.jpg', "wb")
                file.write(s + a[:a.find(end)])
                s = a[a.find(end) + len(end):]
                endTime = round(time.time() * 1000)
                print(str(endTime - starTime) + "ms")
                starTime = endTime
                continue
            s = s + a


if __name__ == '__main__':
    ImageRecvThread().start()
    # while True:
    #     pass



