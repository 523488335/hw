import threading
from hardware.main_business_camera import main_business
from hardware.function_lib import *
from realtime_show import button, realtimeShow
from hardware.LCD_show import LCD
from hardware import globalvar as gl
from hardware.demo_main import demo_main_business

def dispatch(c=None, request=None):
    ser = serial_open('/dev/ttyAMA0', 115200)  # 打开串口与单片机通信 串口打开函数，波特率 115200

    while True:
        Flag_start = gl.get_value('Flag_start')
        Flag_pause = gl.get_value('Flag_pause')
        Flag_reset = gl.get_value('Flag_reset')
        Flag_init = gl.get_value('Flag_init')
        Flag_demo = gl.get_value('Flag_demo')
        gl.set_value('picture', 4)
        if Flag_demo == True:
            print('demo')
            demo_main_business()
            gl.set_value('Flag_demo', False)
        if Flag_start == True :  # or request['mode'] == "MainStream":
            thread_main = main_stream_thread(c, request)
            thread_realtime = thread_realtime_show(c, request)
            thread_main.start()
            thread_realtime.start()
            gl.set_value('Flag_start', False)
        elif Flag_pause == True:
            gl.set_value('Flag_pause', False)
            while True:
                if Flag_init == True:
                    gl.set_value('Flag_init', False)
                    break
        elif Flag_reset == True:
            ser.write(bytes('SZF00000!', 'utf-8'))
            gl.set_value('Flag_start', False)
        #elif Flag_demo == True:
            #print('demo')
            #thread_demo = thread_breath(c, request)
            #thread_breath.start()
            #gl.set_value('Flag_demo', False)




class thread_breath(threading.Thread):
    def __init__(self, conection, request):
        super().__init__()
        self.c = conection
        self.rqt = request

    def run(self):  # 以下就可以编写线程需要的功能
        demo_main_business()
        


class main_stream_thread(threading.Thread):
    def __init__(self, conection, request):
        super().__init__()
        self.c = conection
        self.rqt = request

    def run(self):  # 以下就可以编写线程需要的功能
        main_business()
        # sendMessage(self.c, self.rqt['id'], 1, "主流程结束")

class thread_realtime_show(threading.Thread):
    def __init__(self, conection, request):
        super().__init__()
        self.c = conection
        self.rqt = request

    def run(self):  # 以下就可以编写线程需要的功能
        # sendMessage(self.c, self.rqt['id'], 1, "TrueTime start")
        # time.sleep(2)
        # for i in range(10):
        #     sendMessage(self.c, self.rqt['id'], 1, "time:" + str(i))
        # sendMessage(self.c, self.rqt['id'], 3, "TrueTime end")
        # realtimeShow(self.c, self.rqt)
        realtimeShow(self.c, self.rqt)

class thread_button(threading.Thread):
    def __init__(self):
        super().__init__()
    def run(self):  # 以下就可以编写线程需要的功能
        button()

class thread_LCD(threading.Thread):
    def __init__(self):
        super().__init__()
    def run(self):  # 以下就可以编写线程需要的功能
        LCD()


logger_config()  # 配置日志文件
gl._init()
GPIO.setmode(GPIO.BCM)
button_t = thread_button()
button_t.start()
LCD_t = thread_LCD()
LCD_t.start()
gl.set_value('picture', 0)
dispatch()












#
# while True:
#     try:
#         gl._init()  # 初始化各全局变量
#         # 创建socket对象
#         c = socket.socket()
#         # 连接套接字（服务器）
#         c.connect(('localhost', 11111))
#         print("连接套接字（服务器）完成")
#         break
#     except BaseException as e:
#         print("服务器还未开启，等待中。。。。。")
#         time.sleep(2)
# while True:
#     rev_data = str(c.recv(1024), encoding='utf-8')
#     data = json.loads(rev_data)
#     print(data)
#     dispatch(c, data)





