import RPi.GPIO as GPIO
from hardware.function_lib import *
from hardware import globalvar as gl


#  悬臂高度调整
def cantilever_height_adjustment(ser):
    """
    悬臂高度调整  用红绿指示灯调整悬臂上下高度位置是否合适
    :return:
    """
    print('3333')
    gl.set_value('picture', 5)
    ser_top_plate = serial_open('/dev/lm')  # 打开上顶板激光
    Red_Led = 1  # 定义 1号管脚为红灯
    Green_Led = 12  # 定义12号管脚为绿灯
    distance_top_middle_plate = 102  # The distance between the top plate and the middle plate
    GPIO.setup(Green_Led, GPIO.OUT)  # 设置为输出管脚
    GPIO.setup(Red_Led, GPIO.OUT)
    # 设置红灯亮 绿灯灭
    GPIO.output(Red_Led, GPIO.HIGH)
    GPIO.output(Green_Led, GPIO.LOW)
    time.sleep(3)
    sendInstructions(ser, 'Sb500000!')

    # sendInstructions(ser, 'SbA00000!')  # 打开陀螺仪
    # while True:
    #     if ser.read() == b'#':
    #
    #         gyroscope_recv = gyroscope(ser)
    #         print(gyroscope_recv)
    #     else:
    #         gyroscope_recv = 361
    #
    #     recv_top_plate = laser_receive(ser_top_plate) - distance_top_middle_plate
    #     print('recv_top_plate', recv_top_plate)
    #     if 250 < recv_top_plate < 265 and (0 <= gyroscope_recv <= 2 or 358 <= gyroscope_recv <= 360):
    #         sendInstructions(ser, 'SbB0000!')  # 打开陀螺仪
    #         # 设置红灯灭 绿灯亮
    #         GPIO.output(Green_Led, GPIO.HIGH)
    #         GPIO.output(Red_Led, GPIO.HIGH)
    #         buzzer()
    #         break
    while True:
        #if ser.read() == b'#':
            #gyroscope_recv = gyroscope(ser)
            
        #else:
            #gyroscope_recv = 361

        recv_top_plate = 265  # laser_receive(ser_top_plate) - distance_top_middle_plate
        print('recv_top_plate', recv_top_plate)
        if 260 < recv_top_plate < 270 : #and (0 <= gyroscope_recv <= 2 or 358 <= gyroscope_recv <= 360):
            # sendInstructions(ser, 'SbB00000!')  # 陀螺仪
            # 设置红灯灭 绿灯亮

            GPIO.output(Green_Led, GPIO.HIGH)
            GPIO.output(Red_Led, GPIO.LOW)
            gl.set_value('picture', 7)
            break
    Flag = True
    sendInstructions(ser, 'Sb200000!')  # 语音6
    return Flag


def gyroscope(ser):
    try:
        recv = ser.readline(4)
        print('gyroscope',ser.readline(4))
        recv = str(recv)
        recv = recv.split('b\'')[1].split('!')[0]
        recv = int(recv)

        recv = recv * 360/255
    except ValueError as v:
        print(v)
        recv = 361
    return recv


# 得到激光所有的信息点
def get_points(ser):
    """
    得到激光传感器采集点的信息 并进行坐标转换,以双目摄像头中心的下方中位板为坐标原点
    :return:
    """
    print('Start collecting data points ...')
    ser_e0 = serial_open(device="/dev/l2")  # 打开0号机械臂侧壁激光 1    l2
    ser_z0 = serial_open(device="/dev/l1")  # 打开0号机械臂中位板激光0   l1
    ser_e1 = serial_open(device="/dev/l3")  # 打开1号机械臂侧壁激光3     l3
    ser_z1 = serial_open(device="/dev/l4")  # 打开1号机械臂中位板激光2   l4

    laser_e0 = []  # 0号机械臂侧壁激光的信息列表
    laser_z0 = []

    laser_e1 = []  # 1号机械臂侧壁激光的信息列表
    laser_z1 = []

    length_init = 50  # 长度阈值 50mm

    while True:
        recv_e0 = laser_receive(ser_e0)  # 接收激光0e得到信息
        recv_z0 = laser_receive(ser_z0)  # 接收激光0z得到信息

        recv_e1 = laser_receive(ser_e1)  # 接收激光0e得到信息
        recv_z1 = laser_receive(ser_z1)  # 接收激光0z得到信息

        if 200 < recv_z0 < 500 and recv_e0 < 250:
            laser_e0.append(recv_e0)
            laser_z0.append(recv_z0)
        if 200 < recv_z1 < 500 and recv_e1 < 250:
            laser_e1.append(recv_e1)
            laser_z1.append(recv_z1)

        # 打印激光距离信息 可注释
        if recv_e0 - length_init >= 5:  # 长度每隔5mm 进行数据显示
            length_init = recv_e0
            print('0 edge dis :', recv_e0, '0 depth dis:', recv_z0)
            print('1 edge dis :', recv_e1, '1 depth dis:', recv_z1)

        cnt = ser.inWaiting()  # 等待单片机发送指令
        if cnt == 0:
            continue
        elif ser.read() == b'T':  # 如果发送指令为'T' 则跳出
            print('These points have been obtained!')
            logging.info('These points have been obtained!')
            break

    # 保存信息用于乳房曲线拟合
    save_file('laser_e0.txt', laser_e0)  # 保存0侧壁激光的距离信息
    save_file('laser_z0.txt', laser_z0)  # 保存0中位板激光的距离信息
    save_file('laser_e1.txt', laser_e1)  # 保存1侧壁激光的距离信息
    save_file('laser_z1.txt', laser_z1)  # 保存1中位板激光的距离信息

    # 坐标转换
    def coordinate_transformation(laser_y, laser_z, mode=0, limit_center=284, limit_step_motor=70, limit_thick=8):
        """
        对所有的激光进行坐标转换  坐标原点为中位板中心处
        :param laser_y: 边缘处激光距离
        :param laser_z: 中位板处激光距离
        :param mode: 当模式为 1 时为 y 轴负方向
        :param limit_center: 中位板中心到边缘激光的距离
        :param limit_step_motor: 中位板激光到步进电机白板的距离
        :param limit_thick: 中位板激光到中位板的深度距离
        :return: 转换后的坐标信息   x正方向为从脚到头   y的正方向为从1到0    z的正方向为向下
        """
        if mode == 1:
            y = (np.array(laser_y) + limit_step_motor) - limit_center
        if mode == 0:
            y = limit_center - (np.array(laser_y) + limit_step_motor)
        x = np.array([0 for i in range(len(laser_y))])

        z = np.array(laser_z) + limit_thick

        return x, y, z

    x_0, y_0, z_0 = coordinate_transformation(laser_e0, laser_z0)  # 将0号进行坐标转换
    x_1, y_1, z_1 = coordinate_transformation(laser_e1, laser_z1, mode=1)  # 将1号进行坐标转换

    # 补偿激光高度
    z_0 = 442.3 / (0.186 * y_0 + 423.82) * z_0
    z_1 = 443.2 / (-0.104 * y_1 + 436.1) * z_1

    points_0 = np.vstack((x_0, y_0, z_0)).T
    points_1 = np.vstack((x_1, y_1, z_1)).T

    save_file('points_0.txt', points_0, mode=1)
    save_file('points_1.txt', points_1, mode=1)

    return points_0, points_1


def points_processing(points, nipple, nipple_bone, mode=0):
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]


    a2, a1, a0 = np.poly1d(np.polyfit(y, z, 2))
    b3, b2, b1, b0 = np.poly1d(np.polyfit(y, z, 3))
    k = list(map(lambda x: 2 * a2 * x + a1, y))
    nipple_index = np.argmin(np.array(list(map(abs, k))))

    nipple_x =  x[nipple_index]
    nipple_y = nipple[1]  # y[nipple_index]
    nipple_z = b3 * nipple_y ** 3 + b2 * nipple_y ** 2 + b1 * nipple_y + b0

    nipple_duan_y =nipple_bone[1]  # y[-1]
    nipple_duan_z =nipple_bone[2]  #  b3 * (nipple_duan_y ** 3) + b2 * (nipple_duan_y ** 2) + b1 * nipple_duan_y + b0

    k_bone, _ = np.poly1d(np.polyfit([nipple_y, nipple_duan_y], [nipple_z, nipple_duan_z], 1))

    r_e = abs(nipple_y * 3 / 5)
    if r_e >80:
        r_e = 50
    r_m = 35  # abs(nipple_y * 1 / 2)

    print('mode', mode, 'nipple_y, nipple_z', nipple_y, nipple_z, '/n')

    r_e_l = nipple_y * (1 + 3 / 5)
    r_m_l = nipple_y * (1 + 1 / 2)

    # 注释 r_0_e_d 为乳房边缘的depth   # r_0_e_k 为乳房边缘的斜率
    r_e_d = b3 * (r_e_l ** 3) + b2 * (r_e_l ** 2) + b1 * r_e_l + b0
    r_e_k = 3 * b3 * (r_e_l ** 2) + 2 * b2 * r_e_l + b1
    r_e_bone =nipple_bone[2]  #  r_e_d - 5  # b3 * (r_e ** 3) + b2 * (r_e ** 2) + b1 * r_e + b0

    r_m_d = b3 * (r_m_l ** 3) + b2 * (r_m_l ** 2) + b1 * r_m_l + b0
    r_m_bone = r_m_d - 5  # b3 * (r_m ** 3) + b2 * (r_m ** 2) + b1 * r_m + b0

    r_m_k = 3 * b3 * (r_m_l ** 2) + 2 * b2 * r_m_l + b1

    if r_e_d < 230 or r_e_d > 270:
        print('r_e_d',r_e_d)
        r_e_d = 260
    if r_m_d < 230 or r_m_d > 270:
        r_m_d = 250

    r_e_k = abs(r_e_k)
    r_m_k = abs(r_m_k)
    k_bone = abs(k_bone)
    if abs(r_e_k) > 0.5:
        r_e_k = 0.5
    if abs(r_m_k) > 0.5:
        r_m_k = 0.5  # 有问题
    if abs(k_bone) > 0.5:
        k_bone = 0.25

    print('mode', mode, 'k_bone, r_e_k, r_m_k', k_bone, r_e_k, r_m_k, '/n')

    def k_info(k, k_bone, mode, times=9):
        k_value = []
        k_d_value = k / times
        k_bone_d_value = k_bone / times
        a = []
        b = []
        c = []
        d = []
        for i in range(9):
            a.append(k_bone_d_value * i)
            b.append(k_bone - i * k_bone_d_value)
            c.append(i * k_d_value)
            d.append(k - i * k_d_value)
        if mode == 1:
            k_value.extend(a)
            k_value.extend(b)
            k_value.extend(c)
            k_value.extend(d)
        elif mode == 0:
            k_value.extend(c)
            k_value.extend(d)
            k_value.extend(a)
            k_value.extend(b)
        return k_value

    def dif_depth(depth, depth_bone, mode, times=18):
        d_value = (depth - depth_bone) / times
        d_depth = [depth_bone + d_value * i for i in range(times)]

        if mode == 0:
            breast_depth = d_depth[9:]
            breast_depth.extend(d_depth[::-1])
            breast_depth.extend(d_depth[:9])
            
        elif mode == 1:
            breast_depth = d_depth[:9][::-1]
            breast_depth.extend(d_depth)
            breast_depth.extend(d_depth[9:][::-1])

        return breast_depth

    print('r_e_d,r_m_d, r_e_bone', r_e_d, r_m_d, r_e_bone)
    k_e = k_info(r_e_k, k_bone, mode)
    k_m = k_info(r_m_k, k_bone, mode)
    depth_e = dif_depth(r_e_d + 5, r_e_bone, mode)
    depth_m = dif_depth(r_m_d + 10, r_e_bone, mode)

    def point_set(x, y, k, r, depth, times=36):
        # 将所有的点信息规范化
        pointSet = []
        for i in range(times):
            radian = i * 360 / times / 180 * np.pi
            mo = np.sqrt(((np.cos(radian) * k[i]) ** 2 + 1 + (np.sin(radian) * k[i]) ** 2))
            if i < 9:
                pointSet.append(
                    [-np.cos(radian) * k[i] / mo, -np.sin(radian) * k[i] / mo, 1 / mo, np.cos(radian) * r + x,
                     np.sin(radian) * r + y, depth[i] + 2 + np.cos(radian) * r * np.tan(10 / 180 * np.pi),
                     i * 360 / times])
            if 9 <= i < 27:
                pointSet.append(
                    [-np.cos(radian) * k[i] / mo, -np.sin(radian) * k[i] / mo, 1 / mo, np.cos(radian) * r + x,
                     np.sin(radian) * r + y, depth[i]-2, i * 360 / times])
            if i >= 27:
                pointSet.append(
                    [-np.cos(radian) * k[i] / mo, -np.sin(radian) * k[i] / mo, 1 / mo, np.cos(radian) * r + x,
                     np.sin(radian) * r + y, depth[i] + 2 + np.cos(radian) * r * np.tan(10 / 180 * np.pi),
                     i * 360 / times])

        return pointSet

    pointSet_e = point_set(nipple_x, nipple_y, k_e, r_e, depth_e)

    pointSet_m = point_set(nipple_x, nipple_y, k_m, r_m, depth_m)

    return pointSet_e, pointSet_m, nipple_y, r_e


def data_handle(pointSet_e, pointSet_m, nipple_y, mode=0):
    def coordinate_0(point, nipple, i):
        point_coo = []
        RT = (np.mat([[1, 0, 0, 0], [0, 1, 0, -1 * nipple], [0, 0, 1, 0], [0, 0, 0, 1]]))

        point.insert(3, 1)
        point.append(1)

        b = RT * np.mat(point[4:]).T

        point_coo.extend(point[:3])
        point_coo.extend(b.T.tolist()[0][:3])
        point_coo.extend([i * 10])
        return point_coo

    d = abs(nipple_y) - 20  # abs(nipple_left[1]) - 60

    # 坐标旋转之后再平移
    if mode == 0:
        nipple = abs(nipple_y)
    elif mode == 1:
        nipple = -1 * abs(nipple_y)

    pointSend_e = []
    pointSend_m = []
    for i in range(36):
        pointSend_e.append(coordinate_0(pointSet_e[i][:6], nipple, i))
        pointSend_m.append(coordinate_0(pointSet_m[i][:6], nipple, i))

    return pointSend_e, pointSend_m, d


def normal_uplift(pointSend_e, pointSend_m):
    pointSend_e_t = []
    pointSend_m_t = []

    for i in range(len(pointSend_e)):
        pointSend_e_t.append([pointSend_e[i][0], pointSend_e[i][1], pointSend_e[i][2],
                              pointSend_e[i][3] - pointSend_e[i][0] * 10,
                              pointSend_e[i][4] - pointSend_e[i][1] * 10,
                              pointSend_e[i][5] - pointSend_e[i][2] * 10])

        pointSend_m_t.append([pointSend_m[i][0], pointSend_m[i][1], pointSend_m[i][2],
                              pointSend_m[i][3] - pointSend_m[i][0] * 10,
                              pointSend_m[i][4] - pointSend_m[i][1] * 10,
                              pointSend_m[i][5] - pointSend_m[i][2] * 10])

    return pointSend_e_t, pointSend_m_t


def s_scan(probe_width, probe_depth, r, r_1_m_d):
    x_scan_num = int(np.ceil(2 * abs(r) / probe_width))
    y_scan_num = int(np.ceil(2 * abs(r) / probe_depth))
    pointSet_s_scan = []
    for i in range(y_scan_num):
        y = r - r / abs(r) * (probe_depth / 2 + i * probe_depth)
        for j in range(x_scan_num):
            pointSet_s_scan.append([((-1) ** i) * (abs(r) - probe_width / 2 - j * probe_width), y, r_1_m_d])

    return pointSet_s_scan, x_scan_num * y_scan_num


def edge_scan(probe_width, probe_depth, r, r_1_m_d):  # edge scan points (x,y,z)
    edge_distance = r / 2  # edge distance to nipple = r/2
    top_bottom_scan_num = int(np.ceil((abs(r) / 2) / probe_depth))
    # left_right_scan
    pointSet_edge_scan = []
    pointSet_edge_scan.append([0, r / 2, r_1_m_d])  # right side
    for i in range(top_bottom_scan_num):  # bottom
        pointSet_edge_scan.append(
            [-abs(r) / 2, (r / 4) - r / abs(r) * (probe_depth / 2) - r / abs(r) * i * probe_depth, r_1_m_d])
    pointSet_edge_scan.append([0, -r / 2, r_1_m_d])
    for i in range(top_bottom_scan_num):  # top
        pointSet_edge_scan.append(
            [abs(r) / 2, -((r / 4) - r / abs(r) * (probe_depth / 2) - r / abs(r) * i * probe_depth),
             r_1_m_d])  # left side

    return pointSet_edge_scan, (top_bottom_scan_num + 1) * 2


# 机械臂运动控制算法


def ctrlbot(nx, ny, nz, px, py, pz):
    flag = 0
    flag_d = 0
    flag_v = 0
    variance_e = 1
    variance = 1
    d3_limit_min = 129
    d3_limit_max = 179
    d4_limit_min = 25
    d4_limit_max = 67

    theta2_e = 0
    theta1 = 0
    thetafi = 0
    a5 = 67
    d1 = 50
    if px == 0 and py == 0:
        ax = 0
        ay = 1
        az = 0
    else:
        rp = (px ** 2 + py ** 2) ** 0.5
        ax = - py / rp
        ay = px / rp
        az = 0

    ox = ay * nz - az * ny
    oy = az * nx - ax * nz
    oz = ax * ny - ay * nx

    if px == 0:
        theta1 = np.pi / 2
    elif px > 0:
        theta1 = np.arctan(py / px)
    elif px < 0:
        theta1 = np.arctan(py / px) + np.pi

    if nz == 0:
        thetafi = np.pi / 2
    elif nz > 0:
        thetafi = - np.arctan(oz / nz)
    elif nz < 0:
        thetafi = - np.arctan(oz / nz) + np.pi

    for k in range(-90, 91):
        theta2 = k / 180 * np.pi
        theta5 = thetafi - theta2
        d3 = - (np.cos(theta2) * (
                d1 - pz + a5 * np.cos(theta2) * np.cos(theta5) - a5 * np.sin(theta2) * np.sin(theta5))) - (
                     np.sin(theta2) * (
                     a5 * np.cos(theta1) * np.cos(theta2) * np.sin(theta5) - py - px + a5 * np.cos(
                 theta1) * np.cos(theta5) * np.sin(theta2) + a5 * np.cos(theta2) * np.sin(theta1) * np.sin(
                 theta5) + a5 * np.cos(theta5) * np.sin(theta1) * np.sin(theta2))) / (
                     np.cos(theta1) + np.sin(theta1))
        d4 = (np.sin(theta2) * (
                d1 - pz + a5 * np.cos(theta2) * np.cos(theta5) - a5 * np.sin(theta2) * np.sin(theta5))) - (
                     np.cos(theta2) * (
                     a5 * np.cos(theta1) * np.cos(theta2) * np.sin(theta5) - py - px + a5 * np.cos(
                 theta1) * np.cos(theta5) * np.sin(theta2) + a5 * np.cos(theta2) * np.sin(theta1) * np.sin(
                 theta5) + a5 * np.cos(theta5) * np.sin(theta1) * np.sin(theta2))) / (
                     np.cos(theta1) + np.sin(theta1))

        if d3 >= d3_limit_min and d3 <= d3_limit_max and d4 >= d4_limit_min and d4 <= d4_limit_max:
            pxt = d4 * np.cos(theta1) * np.cos(theta2) + d3 * np.cos(theta1) * np.sin(theta2) + a5 * np.cos(
                theta1) * np.cos(theta2) * np.sin(theta5) + a5 * np.cos(theta1) * np.cos(theta5) * np.sin(theta2)
            pyt = d4 * np.cos(theta2) * np.sin(theta1) + d3 * np.sin(theta1) * np.sin(theta2) + a5 * np.cos(
                theta2) * np.sin(theta1) * np.sin(theta5) + a5 * np.cos(theta5) * np.sin(theta1) * np.sin(theta2)
            pzt = d1 + d3 * np.cos(theta2) - d4 * np.sin(theta2) + a5 * np.cos(theta2) * np.cos(theta5) - a5 * np.sin(
                theta2) * np.sin(theta5)
            variance = (px - pxt) ** 2 + (py - pyt) ** 2 + (pz - pzt) ** 2
            flag_d = 1
            if variance <= variance_e:
                variance_e = variance
                theta2_e = theta2
                d3_e = d3
                d4_e = d4
                flag = 1
                flag_v = 1

    theta5_e = thetafi - theta2_e

    if flag == 0:
        if flag_d == 0:
            return ['solution not found:DBL']
        elif flag_v == 0:
            return ['solution not found:VAR']
    elif flag == 1:
        theta1_s = theta1 / np.pi * 180
        if theta1_s < 0:
            theta1_s = theta1_s + 360
        theta2_s = theta2_e / np.pi * 180
        theta5_s = theta5_e / np.pi * 180
        return [theta1_s, theta2_s, d3_e - 129, d4_e - 48, theta5_s]


####################################################

# 通信协议
def communication(ser, point_infos, Flag):
    """
    第四代通信协议，发送24帧数据
    :return:
    """
    style = list(map(chr, range(ord('A'), ord('X') + 1)))
    count = 0
    i = 0
    print('Flag', Flag)
    while True:
        if Flag == 0:
            point_format = point_infos[count]
            # sendMessage(conection, request['id'], 1, count)
        while True:
            recv = ser.read()
            print('recv', recv)

            if (Flag == 1 or Flag == 3 or Flag == 5) and recv == b'S':
                print('SZF00000!')
                logging.error('Flag: %s,SZF00000!' % Flag)
                Mechanical_arm_reset(ser)
                if recv == b'3':
                    pass
                break
            elif (Flag == 2 or Flag == 4 or Flag == 6) and recv == b'S':
                print('SZF00001!')
                logging.error('Flag: %s,SZF00001!' % Flag)
                Mechanical_arm_reset(ser)
                if recv == b'3':
                    pass
                break
            else:
                while i < 24:
                    if recv == b'S':
                        print('S' + style[i] + point_format[i] + '!')
                        ser.write(bytes('S' + style[i] + point_format[i] + '!', 'utf-8'))
                        i += 1
                        break
                    elif recv == b'A' or recv == b'2':
                        print('data error')
                        logging.warning('data Error')
                        ser.write(bytes('S' + style[i - 1] + point_format[i - 1] + 'P', 'utf-8'))
            if recv == b'W':
                i = 0
                count += 1
                break
        if Flag != 0:
            break
        elif count > len(point_infos) - 1:
            break


def control_algorithm(point_0, point_0_t, point_1, point_1_t, d0, d1):
    points = []
    Flag = 0

    for i in range(36 + 1):
        point_info = []
        if i == 36:
            i = 0
        point_info.append(int(d0 * 10 + 0.5) / 10)
        point_info.extend(
            ctrlbot(point_0[i][0], point_0[i][1], point_0[i][2], point_0[i][3], point_0[i][4],
                    point_0[i][5]))  # 0号机械臂的位置
        point_info.append(int(d0 * 10 + 0.5) / 10)
        point_info.extend([i*10,0,0,10,0])  # 0号机械臂的抬高
        point_info.append(int(d1 * 10 + 0.5) / 10)
        point_info.extend(
            ctrlbot(point_1[i][0], point_1[i][1], point_1[i][2], point_1[i][3], point_1[i][4],
                    point_1[i][5]))  # 1号机械臂的位置
        point_info.append(int(d1 * 10 + 0.5) / 10)
        point_info.extend([i*10,0,0,10,0])  # 1号机械臂的抬高
        print('-----------------------------------------------')
        print(point_info)  # 打印看看
        print('-----------------------------------------------')
        # 报错信息
        if 'solution not found:VAR' in point_info:
            print('Error: solution not found:SZF00000!:方差太大')
            logging.error('Error: solution not found:SZF00000!:方差太大')
            Flag = 1
            break
        elif 'solution not found:DBL' in point_info:
            print('Error: solution not found:SZF00001!:到不了')
            Flag = 2
            logging.error('Error: solution not found:SZF00001!:到不了')
            break
        else:
            point_format = data_format(point_info)
        points.append(point_format)

    return points, Flag


def Square_scanning(normal, edge_scan_num, pointSet_edge_scan_l, pointSet_edge_scan_r, d0, d1):
    points = []
    Flag = 0
    print('edge_scan_num', edge_scan_num)
    for i in range(edge_scan_num):
        if pointSet_edge_scan_l[i][0] == 0:
            pointSet_edge_scan_l[i][0] = 15
        if pointSet_edge_scan_r[i][0] == 0:
            pointSet_edge_scan_r[i][0] = 15
        print('pointSet_edge_scan_l[i][0]', pointSet_edge_scan_l[i][0])
        point_info = []
        point_info.append(pointSet_edge_scan_l[i][1] + d0)  # d0 = y
        point_info.extend(ctrlbot(normal[0], normal[1], normal[2], pointSet_edge_scan_l[i][0], 0,
                                  pointSet_edge_scan_l[i][2]))  # 0号机械臂的位置

        point_info.append(pointSet_edge_scan_l[i][1] + d0)  # d0 = y
        point_info.extend(ctrlbot(normal[0], normal[1], normal[2], pointSet_edge_scan_l[i][0], 0,
                                  pointSet_edge_scan_l[i][2] - 5))  # 0号机械臂的抬高

        point_info.append(pointSet_edge_scan_r[i][1] + d1)  # d0 = y
        point_info.extend(ctrlbot(normal[0], normal[1], normal[2], pointSet_edge_scan_r[i][0], 0,
                                  pointSet_edge_scan_r[i][2]))  # 1号机械臂的位置

        point_info.append(pointSet_edge_scan_r[i][1] + d1)  # d0 = y
        point_info.extend(ctrlbot(normal[0], normal[1], normal[2], pointSet_edge_scan_r[i][0], 0,
                                  pointSet_edge_scan_r[i][2] - 5))  # 1号机械臂的抬高

        print('-----------------------------------------------')
        print(point_info)  # 打印看看
        print('-----------------------------------------------')
        # 报错信息
        if 'solution not found:VAR' in point_info:
            print('Error: solution not found:SZF00000!:方差太大')
            Flag = 5
            break
        elif 'solution not found:DBL' in point_info:
            print('Error: solution not found:SZF00001!:到不了')
            Flag = 6
            break
        else:
            point_format = data_format(point_info)
        points.append(point_format)
    return points, Flag


def scan_s(num, normal, point_0, point_1, d0, d1):
    points = []
    Flag = 0
    for i in range(num):
        if point_0[i][0] == 0:
            point_0[i][0] = 15
        if point_1[i][0] == 0:
            point_1[i][0] = 15
        print('pointSet_edge_scan_l[i][0]', point_0[i][0])
        point_info = []
        point_info.append(point_0[i][1] + d0)  # d0 = y
        point_info.extend(ctrlbot(normal[0], normal[1], normal[2], point_0[i][0], 0,
                                  point_0[i][2]))  # 0号机械臂的位置

        point_info.append(point_0[i][1] + d0)  # d0 = y
        point_info.extend(ctrlbot(normal[0], normal[1], normal[2], point_0[i][0], 0,
                                  240))  # 0号机械臂的抬高

        point_info.append(point_1[i][1] + d1)  # d0 = y
        point_info.extend(ctrlbot(normal[0], normal[1], normal[2], point_1[i][0], 0,
                                  point_1[i][2]))  # 1号机械臂的位置

        point_info.append(point_1[i][1] + d1)  # d0 = y
        point_info.extend(ctrlbot(normal[0], normal[1], normal[2], point_1[i][0], 0,
                                  240))  # 1号机械臂的抬高

        print('-----------------------------------------------')
        print(point_info)  # 打印看看
        print('-----------------------------------------------')
        # 报错信息
        if 'solution not found:VAR' in point_info:
            print('Error: solution not found:SZF00000!:方差太大')
            Flag = 7
            break
        elif 'solution not found:DBL' in point_info:
            print('Error: solution not found:SZF00001!:到不了')
            Flag = 8
            break
        else:
            point_format = data_format(point_info)
        points.append(point_format)
    return points, Flag


def sendInstructions(ser, instruction):
    """
    发送指令函数
    :param ser: 串口
    :param instruction: 指令
    :return:
    """
    while True:
        if ser.read() == b'S':
            ser.write(bytes(instruction, 'utf-8'))
            break

def demo_main_business():

    # 前期处理
    ser = serial_open('/dev/ttyAMA0', 115200)  # 打开串口与单片机通信 串口打开函数，波特率115200
    GPIO.setwarnings(False)  # 如果RPi.GRIO检测到一个引脚已经被设置成了非默认值，那么你将看到一个警告信息 可以通过代码禁用警告
    GPIO.setmode(GPIO.BCM)  # 设置树莓派GPIO管脚模式为 BCM 模式
    print('11111')
    ser.write(bytes('ScA00000!', 'utf-8'))
    # ser.write(bytes('SaA00007!', 'utf-8'))
    sendInstructions(ser, 'SaA00007!')
    time.sleep(0.5)
    sendInstructions(ser, 'Sb800000!')  #

    Flag = cantilever_height_adjustment(ser)  # 悬臂高度位置函数
    # sendMessage(conection, request['id'], 1, "The height of the cantilever is suitable")
    logging.info('The height of the cantilever is suitable')
    # nipple_left, nipple_night, nipple_bone = two_cam_disparity()  # 双目采集



    if Flag:
        sendInstructions(ser, 'Sb100000!')
        print('33344')
        sendInstructions(ser, 'SaA00000!')
        logging.info('机械臂运动开始')
        gl.set_value('picture', 8)
        print('SaA00000!')


    while True:  # 等待机械臂初始化完成
        recv = ser.read()
        if recv == b'K':

            break

            # 数据收集
    points_0, points_1 = get_points(ser)  # data collection

    # # 数据处理
    # pointSet_e0, pointSet_m0, nipple_y_0, r_0_e = points_processing(points_0, nipple_left, nipple_bone, mode=0)
    #
    # pointSet_e1, pointSet_m1, nipple_y_1, r_1_e = points_processing(points_1, nipple_night, nipple_bone, mode=1)
    #
    # # 到达的坐标点
    # pointSet_eL, pointSet_mL, d0 = data_handle(pointSet_e0, pointSet_m0, nipple_y_0, mode=0)
    # pointSet_eR, pointSet_mR, d1 = data_handle(pointSet_e1, pointSet_m1, nipple_y_1, mode=1)
    # # 抬升的坐标点
    # pointSend_e0_t, pointSend_m0_t = normal_uplift(pointSet_eL, pointSet_mL)
    # pointSend_e1_t, pointSend_m1_t = normal_uplift(pointSet_eR, pointSet_mR)
    #
    # probe_width = 50  # 普博设备的宽度
    # probe_depth = 10  # 普博设备的厚度
    # r = np.mean([r_0_e, r_1_e])  # 对两边半径进行取平均值
    # pointSet_edge_scan_l, edge_scan_num_l = edge_scan(probe_width, probe_depth, -1 * r, pointSet_m0[0][5])
    # pointSet_edge_scan_r, edge_scan_num_r = edge_scan(probe_width, probe_depth, r, pointSet_m0[0][5])
    # edge_scan_num = min(edge_scan_num_l, edge_scan_num_r)
    #
    # print('r_0_e', r_0_e)
    # print('r_1_e', r_1_e)
    # logging.info(r_0_e)
    # logging.info(r_1_e)

    # sendInstructions(ser, 'Sb90000!')
    ########## S-scan################
    # point_s_0, num = s_scan(probe_width, probe_depth, -1 * r, pointSet_m0[0][5])
    # point_s_1, num = s_scan(probe_width, probe_depth, r, pointSet_m1[0][5])
    #
    #
    # # 运动算法处理
    # point_edge, Flag_edge = control_algorithm(pointSet_eL, pointSend_e0_t, pointSet_eR, pointSend_e1_t, d0, d1)
    #
    # point_mid, Flag_mid = control_algorithm(pointSet_mL, pointSend_m0_t, pointSet_mR, pointSend_m1_t, d0, d1)
    #
    # normal = [0, 0, 1]
    # point_side, Flag_side = Square_scanning(normal, edge_scan_num, pointSet_edge_scan_l, pointSet_edge_scan_r, d0,
    #                                         d1)

    # point_s, Flag_s = scan_s(num, normal, point_s_0, point_s_1, d0, d1)

    # communication(ser, point_s, Flag_s,conection,request)
    # 数据传输
    # 保存信息
    # save_file('point_s_0.txt', point_s_0, 1)
    # save_file('point_s_1.txt', point_s_1, 1)
    # save_file('pointSet_eL.txt', pointSet_eL, 1)
    # save_file('pointSet_eR.txt', pointSet_eR, 1)
    # save_file('pointSet_mL.txt', pointSet_mL, 1)
    # save_file('pointSet_mR.txt', pointSet_mR, 1)
    # save_file('pointSend_e0_t.txt', pointSend_e0_t, 1)
    # save_file('pointSend_m0_t.txt', pointSend_m0_t, 1)
    # save_file('pointSend_e1_t.txt', pointSend_e1_t, 1)
    # save_file('pointSend_m1_t.txt', pointSend_m1_t, 1)
    # save_file('pointSet_edge_scan_l.txt', pointSet_edge_scan_l, mode=1)
    # save_file('pointSet_edge_scan_r.txt', pointSet_edge_scan_r, mode=1)

    Flag_edge = 0
    Flag_mid = 0
    Flag_side = 0
    points_edge = np.loadtxt('points_edge.txt')
    points_mid = np.loadtxt('points_mid.txt')
    points_side = np.loadtxt('points_side.txt')
    point_edge = []
    point_mid =[]
    point_side = []
    for i in range(37):
        if i==36:
            i=0
        point_format = data_format(points_edge[i])
        print(point_format)
        point_edge.append(point_format)

    for i in range(37):
        if i==36:
            i=0
        point_format = data_format(points_mid[i])
        point_mid.append(point_format)

    for i in range(len(points_side)):
        point_format = data_format(points_side[i])
        point_side.append(point_format)


    
    if Flag_edge == 0:
        ser.write(bytes('Sb00000!', 'utf-8'))
        # sendInstructions(ser, 'Sb00000!')
        communication(ser, point_edge, Flag_edge)
    if Flag_mid == 0 and Flag_side == 0:
        communication(ser, point_mid, Flag_mid)
        communication(ser, point_side, Flag_side)
        gl.set_value('picture', 11)
        time.sleep(1)
        logging.info('SYA00000!')
        # ser.write(bytes('SaA00008!', 'utf-8'))
        sendInstructions(ser, 'SaA00008!')
        sendInstructions(ser, 'Sb700000!')
        gl.set_value('picture', 12)
        sendInstructions(ser, 'SYA00000!')
        GPIO.output(1, GPIO.LOW)
        GPIO.output(12, GPIO.LOW)
        while True:
            recv_y = ser.read()
            print('recv_y', recv_y)
            if recv_y == b'1':
                print('-----------------------end---------------------')
                break

    else:
        print('----------error----------------')
        gl.set_value('picture', 12)
        Mechanical_arm_reset(ser)
        print('----------error----------------')





