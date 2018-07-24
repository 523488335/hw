import serial
import logging
import time
import numpy as np
import json
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

#  IO 管脚输入类
class Io(object):
    GPIO.setmode(GPIO.BCM)
    def __init__(self,pin):
        GPIO.setmode(GPIO.BCM)
        self.pin = pin
        GPIO.setup(self.pin, GPIO.IN)
        #  GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # 树莓派IO管脚读取函数
    def input(self):
        GPIO.setup(self.pin, GPIO.IN)
        return GPIO.input(self.pin)


# 日志配置文件
def log_config(log_name="vision-7", log_filename="V7_info.log" ):
    """
    #########################################################################################################
    # 设置log日志的格式 eg.  2018-06-25 15:32:20,231 test0620.py point [line:61] WARNING This is WARNING message #
    :return: 日志实例
    """
    # 获取logger实例，如果参数为空则返回 root logger
    logger = logging.getLogger(log_name)

    # 指定logger输出格式
    formatter = logging.Formatter(fmt='%(asctime)s %(filename)s %(funcName)s[line:%(lineno)d] %(levelname)s %(message)s')

    # 文件日志
    file_handler = logging.FileHandler(filename=log_filename, mode='w')
    file_handler.setFormatter(formatter)  # 可以通过setFormatter指定输出格式

    # 控制台日志
    console_handler = logging.StreamHandler() # sys.stdout
    console_handler.formatter = formatter  # 也可以直接给formatter赋值

    # 为logger添加的日志处理器
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    # 指定日志的最低输出级别，默认为WARN级别
    logger.setLevel(logging.INFO)
    return logger


def logger_config(log_filename="V7_info.log"):
    logging.basicConfig(level=logging.DEBUG,
                        format='%(asctime)s %(filename)s %(funcName)s[line:%(lineno)d] %(levelname)s %(message)s',
                        datefmt='%a, %d %b %Y %H:%M:%S',
                        filename=log_filename,
                        filemode='w')



def sendResponse(conection, response):
    s = json.dumps(response) + "\n"
    # u = s.encode('utf-8')
    u = bytes(s, 'utf-8')
    conection.send(u)

def sendMessage(conection, id, state, msg):
    if conection:
        response = {}
        response['id'] = id
        response['state'] = state
        response['msg'] = msg
        sendResponse(conection, response)


# 串口打开函数
def serial_open(device, Baud_rate = 9600):
    """
    :param device: USB串口号
    :return: 打开USB串口
    """
    try:
        return serial.Serial(device, Baud_rate)
    except (FileNotFoundError) as FileNotFound:
        logging.error(FileNotFound)
        print(FileNotFound)


# 数据保存
def save_file(filename, data, mode=0):
    """
    数据保存
    :param filename: 文件名
    :param data: 数据
    :param mode:  mode =0 以字符串模式保存    mode=1 以 numpy.ndarray 模式保存
    :return: None
    """
    if mode == 0:
        with open('data/'+filename, 'w') as file:
            for i in data:
                file.write(str(i) + '\n')
    elif mode == 1:
        np.savetxt('data/'+filename, data)
    return None


# 激光接收数据转换
def laser_receive(serial_number):
    """
    激光传感器返回值接收函数
    :param serial_no: 串口号
    :return: 整型数据
    """
    try:
        recv = int(str(serial_number.readline()).split('mm')[0].split('b\'')[1])
        # recv0 = str(serial_number.readline())
        # recv1 = recv0.split('mm')[0]
        # recv2 = recv1.split('b\'')[1]
        # recv2 = int(recv2)
    except (IndexError, ValueError) as e:
        print(e, ':invalid literal for int() with base 10')
        logging.warning('ValueError:invalid literal for int() with base 10')
        recv = 0
    return recv


# 机械臂复位函数
def Mechanical_arm_reset(ser):
    """
    机械臂复位函数 发送指令是机械臂复位  关闭灯光  关闭指示灯
    :return:
    """
    # ser.write(bytes('SZF00000!', 'utf-8'))
        
    ser.write(bytes('SZF00000!', 'utf-8'))
    while True:
        if ser.read() == b'2':
            print('SZF00000!')
            break
        
    time.sleep(0.1)
    ser.write(bytes('SaA00008!', 'utf-8'))


# 通信数据规范化
def data_format(point_info):
    """
    数据按照通信协议进行规范化处理
    :param point_info: 算法数据点的信息
    :return:  规范化后数据点
    """
    point_format = []
    for point in point_info:
        if point >= 0:
            point = int(point * 100 + 0.5)
            point = 'A' + str(abs(point)).zfill(5)
        else:
            point = int(point * 100 - 0.5)
            point = 'F' + str(abs(point)).zfill(5)
        point_format.append(point)

    return point_format



# ###############################################
# # 机械臂运动控制算法  参数没改的算法
# def ctrlbot(nx, ny, nz, px, py, pz):
#     flag = 0
#     flag_d = 0
#     flag_v = 0
#     variance_e = 1
#     variance = 1
#     d3_limit_min = 129
#     d3_limit_max = 179
#     d4_limit_min = 18
#     d4_limit_max = 65
#
#     theta2_e = 0
#     theta1 = 0
#     thetafi = 0
#     a5 = 50
#     d1 = 60
#     if px == 0 and py == 0:
#         ax = 0
#         ay = 1
#         az = 0
#     else:
#         rp = (px ** 2 + py ** 2) ** 0.5
#         ax = - py / rp
#         ay = px / rp
#         az = 0
#
#     ox = ay * nz - az * ny
#     oy = az * nx - ax * nz
#     oz = ax * ny - ay * nx
#
#     if px == 0:
#         theta1 = np.pi / 2
#     elif px > 0:
#         theta1 = np.arctan(py / px)
#     elif px < 0:
#         theta1 = np.arctan(py / px) + np.pi
#
#     if nz == 0:
#         thetafi = np.pi / 2
#     elif nz > 0:
#         thetafi = - np.arctan(oz / nz)
#     elif nz < 0:
#         thetafi = - np.arctan(oz / nz) + np.pi
#
#     for k in range(-90, 91):
#         theta2 = k / 180 * np.pi
#         theta5 = thetafi - theta2
#         d3 = - (np.cos(theta2) * (
#                 d1 - pz + a5 * np.cos(theta2) * np.cos(theta5) - a5 * np.sin(theta2) * np.sin(theta5))) - (
#                      np.sin(theta2) * (
#                      a5 * np.cos(theta1) * np.cos(theta2) * np.sin(theta5) - py - px + a5 * np.cos(
#                  theta1) * np.cos(theta5) * np.sin(theta2) + a5 * np.cos(theta2) * np.sin(theta1) * np.sin(
#                  theta5) + a5 * np.cos(theta5) * np.sin(theta1) * np.sin(theta2))) / (
#                      np.cos(theta1) + np.sin(theta1))
#         d4 = (np.sin(theta2) * (
#                 d1 - pz + a5 * np.cos(theta2) * np.cos(theta5) - a5 * np.sin(theta2) * np.sin(theta5))) - (
#                      np.cos(theta2) * (
#                      a5 * np.cos(theta1) * np.cos(theta2) * np.sin(theta5) - py - px + a5 * np.cos(
#                  theta1) * np.cos(theta5) * np.sin(theta2) + a5 * np.cos(theta2) * np.sin(theta1) * np.sin(
#                  theta5) + a5 * np.cos(theta5) * np.sin(theta1) * np.sin(theta2))) / (
#                      np.cos(theta1) + np.sin(theta1))
#
#         if d3 >= d3_limit_min and d3 <= d3_limit_max and d4 >= d4_limit_min and d4 <= d4_limit_max:
#             pxt = d4 * np.cos(theta1) * np.cos(theta2) + d3 * np.cos(theta1) * np.sin(theta2) + a5 * np.cos(
#                 theta1) * np.cos(theta2) * np.sin(theta5) + a5 * np.cos(theta1) * np.cos(theta5) * np.sin(theta2)
#             pyt = d4 * np.cos(theta2) * np.sin(theta1) + d3 * np.sin(theta1) * np.sin(theta2) + a5 * np.cos(
#                 theta2) * np.sin(theta1) * np.sin(theta5) + a5 * np.cos(theta5) * np.sin(theta1) * np.sin(theta2)
#             pzt = d1 + d3 * np.cos(theta2) - d4 * np.sin(theta2) + a5 * np.cos(theta2) * np.cos(theta5) - a5 * np.sin(
#                 theta2) * np.sin(theta5)
#             variance = (px - pxt) ** 2 + (py - pyt) ** 2 + (pz - pzt) ** 2
#             flag_d = 1
#             if variance <= variance_e:
#                 variance_e = variance
#                 theta2_e = theta2
#                 d3_e = d3
#                 d4_e = d4
#                 flag = 1
#                 flag_v = 1
#
#     theta5_e = thetafi - theta2_e
#
#     if flag == 0:
#         if flag_d == 0:
#             return ['solution not found:DBL']
#         elif flag_v == 0:
#             return ['solution not found:VAR']
#     elif flag == 1:
#         theta1_s = theta1 / np.pi * 180
#         if theta1_s < 0:
#             theta1_s = theta1_s + 360
#         theta2_s = theta2_e / np.pi * 180
#         theta5_s = theta5_e / np.pi * 180
#         return [theta1_s, theta2_s, d3_e - 129, d4_e - 45, theta5_s]
