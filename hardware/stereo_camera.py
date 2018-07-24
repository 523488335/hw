import numpy as np
import cv2


##################################---子函数---########################################################

def point_xyz(x, y,Circle_crop_i,Circle_crop_j,threeD):
    av = 10
    three3Dx = []
    three3Dy = []
    three3Dz = []

    for i in range(x + Circle_crop_j - av, x + Circle_crop_j + av, 1):
        for j in range(y + Circle_crop_i - av, y + Circle_crop_i + av, 1):
            threeDD = threeD[j][i]
            three3Dx.append(threeDD[0])
            three3Dy.append(threeDD[1])
            three3Dz.append(threeDD[2])

    left_x_median = np.median(three3Dx)
    left_y_median = np.median(three3Dy)
    left_z_median = np.median(three3Dz)

    left_x_average = np.average(three3Dx)
    left_y_average = np.average(three3Dy)
    left_z_average = np.average(three3Dz)

    left_x_median = left_x_median-16
    left_y_median = left_y_median+5
    left_z_median = left_z_median

    
    print('aa',left_y_median, -(left_x_median-15), left_z_median-120)

    return(left_y_median, -(left_x_median-15), left_z_median-120)

    
    # return (left_x_average, left_y_average, left_z_average)
    # return (left_y_average, -(left_x_average-15), left_z_average-98)


#    return(left_x_median,left_y_median,left_z_median)

######################################################################################################


#######################################################
# 双目摄像头保存图片

def pic_aquisition():
    print('--------------start camera--------------')
    size = (320, 240)
    camera1 = cv2.VideoCapture("/dev/v1")
    camera2 = cv2.VideoCapture("/dev/v0")
    camera1.set(3, 320)
    camera1.set(4, 240)
    camera2.set(3, 320)
    camera2.set(4, 240)
    i = 0
    q = 30
    left_list = []
    right_list = []

    while i < q:
        i += 1
        ret1, frame1 = camera1.read()
        ret2, frame2 = camera2.read()

        name_left = "left_" + str(i) + ".jpg"
        name_right = "right_" + str(i) + ".jpg"
        left_list.append(frame1)
        right_list.append(frame2)

    i = 15
    average_left = left_list[9]
    average_right = right_list[9]

    while i < q:
        i += 1
        average_left = cv2.addWeighted(average_left, 0.9, left_list[i - 1], 0.1, -1)
        average_right = cv2.addWeighted(average_right, 0.9, right_list[i - 1], 0.1, -1)

    cv2.imwrite('average_left.jpg', average_left)
    cv2.imwrite('average_right.jpg', average_right)

    camera1.release()
    camera2.release()
    cv2.destroyAllWindows()
    print('***************pictures_acquired***************')
    return (average_left,average_right)


#######################################################
# 双目摄像头计算视差图，求得三维图，提取乳房位置

def two_cam_disparity():
#    pic_aquisition()
#    average_left = cv2.imread("average_left.jpg")
#    average_right = cv2.imread('average_right.jpg')
    average_left, average_right = pic_aquisition()
#    size = average_left.shape
#    size = (size[1], size[0])  # 重新读取图片计算视差需要把图片i，j方向交换
    size = (320, 240)
    Circle_crop_i = 50  # 竖直方向裁剪图片
    Circle_crop_j = 40  # 横向方向裁剪图片

    # 视差图滤波参数
    ksize = 5
    sigma = 2.4
    pix = 1
    pixk = 3
    kernel = np.ones((pixk, pixk), np.float32) / pixk / pixk

    # 左摄像头的内参
    left_camera_matrix = np.array([[236.053622169018, 0., 167.901230790036],
                                   [0., 235.900857271680, 119.392424753879],
                                   [0., 0., 1.]])
    # 左摄像头的畸变参数
    left_distortion = np.array(
        [[-0.455824859753551, 0.409450227615763, 0.000937573182827801, 0.00179278069505717, -0.356689010861526]])

    # 右摄像头的内参
    right_camera_matrix = np.array([[235.896220303675, 0., 156.647997421874],
                                    [0., 235.524808336496, 123.865978579107],
                                    [0., 0., 1.]])

    # 右摄像头的畸变参数 k1,k2,p1,p2,k3
    right_distortion = np.array(
        [[-0.448119802022551, 0.356456249147210, 0.00122792896049192, 0.000253495673483851, -0.244437721695928]])

    # om = np.array([ -0.00413, -0.00176, 0.02087])   # 旋转关系向量
    # R = cv2.Rodrigues(om)[0]  # 使用Rodrigues变换将om变换为R
    # matlab 直接换算R矩阵 注意
    R = np.array([[0.999856036857291, -0.0151424835546409, 0.00765576592042478],
                  [0.0151620818420849, 0.999881902605028, -0.00250840928953117],
                  [-0.00761687824799803, 0.00262412552049596, 0.999967548038939]])
    T = np.array([-29.9198422905687, -0.573039251247837, -0.138077340002760])  # 平移关系向量

    # 进行立体校正
    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                      right_camera_matrix,
                                                                      right_distortion, size, R, T)
    # 计算更正map
    left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
    right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size,
                                                         cv2.CV_16SC2)

    ##__________________camera_config_end________________________

    # 根据更正map对图片进行重构


    img1_rectified = cv2.remap(average_left, left_map1, left_map2, cv2.INTER_LINEAR)
    img2_rectified = cv2.remap(average_right, right_map1, right_map2, cv2.INTER_LINEAR)

    # 将图片置为灰度图，为StereoBM作准备
    imgL = cv2.cvtColor(img1_rectified, cv2.COLOR_BGR2GRAY)
    imgR = cv2.cvtColor(img2_rectified, cv2.COLOR_BGR2GRAY)

    # 前处理，滤波平滑
    imgL = cv2.medianBlur(imgL, 3)
    imgR = cv2.medianBlur(imgR, 3)

    cv2.imwrite("imgL.jpg", imgL)
    cv2.imwrite("imgR.jpg", imgR)

    # 根据Semi-Global Block Matching算法生成差异图
    uniquenessRatio = 1
    min_disp = 0  # Normally, it is zero
    num_disp = 32 - min_disp  # this parameter must be divisible by 16
    blockSize = 1  # It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range
    P1 = 2 * blockSize ** 2  # The first parameter controlling the disparity smoothness.
    P2 = 40 * blockSize ** 2  # The larger the values are, the smoother the disparity is
    # reasonably good P1 and P2 values are shown (like 8*number_of_image_channels*SADWindowSize*SADWindowSize
    #                                             32*number_of_image_channels*SADWindowSize*SADWindowSize ,
    disp12MaxDiff = -1  # Maximum allowed difference (in integer pixel units) in the left-right disparity check
    preFilterCap = 0  # Truncation value for the prefiltered image pixels
    #   uniquenessRatio = 9  # 10  # 误匹配信息 主要可以防止误匹配 5-15为宜
    speckleWindowSize = 100  # 50~200 # 平滑视差区域的最大窗口尺寸
    speckleRange = 2  # 1~2 good enough  # 每个已连接部分的最大视差变化
    mode = -1

    stereo = cv2.StereoSGBM_create(min_disp,
                                   num_disp,
                                   blockSize,
                                   P1,
                                   P2,
                                   disp12MaxDiff,
                                   preFilterCap,
                                   uniquenessRatio,
                                   speckleWindowSize,
                                   speckleRange,
                                   mode
                                   )

    # 计算出视差值
    disparity = stereo.compute(imgL, imgR).astype(np.float32)

    # cv2.imwrite("origin_disparity.jpg", disparity)

    # ----------------------------------------------------------

    # 视差图填充，把小于0（-16）的都填平

    for j in range(0, size[1]):
        for i in range(num_disp + min_disp, size[0]):
            if (disparity[j][i] < 0):
                disparity[j][i] = np.median(disparity[j][i - 1])

    # ---------------------------------------------------------

   


    # ---------------------------------------------------------

    # 滤波参数
    # -------median_filter--------------------------------------
    pixk = 9
    kernel = np.ones((pixk, pixk), np.float32) / pixk / pixk
    median_pix = 5
    bi_pix = 9
    bi_color = 75
    bi_space = 75

    # disparity = cv2.medianBlur(disparity,median_pix)
    disparity = cv2.filter2D(disparity, -1, kernel)
    disparity = cv2.bilateralFilter(disparity, bi_pix, bi_color, bi_space)

    # disparity = cv2.medianBlur(disparity,median_pix)
    disparity = cv2.filter2D(disparity, -1, kernel)
    disparity = cv2.bilateralFilter(disparity, bi_pix, bi_color, bi_space)

    # disparity = cv2.medianBlur(disparity,median_pix)
    disparity = cv2.filter2D(disparity, -1, kernel)
    disparity = cv2.bilateralFilter(disparity, bi_pix, bi_color, bi_space)

    # ---------------------------------------------------------

    # -----------------------------------------------------------


    # -----------------------------------------------------------

    disp = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)

    # 将图片扩展至3d空间中，其z方向的值则为当前的距离
    threeD = cv2.reprojectImageTo3D(disparity.astype(np.float32) / 16., Q, -1)

    doc = open('3d_output.xyz', 'w')
    for i in range (Circle_crop_j, size[0] - Circle_crop_j,1):
        for j in range (Circle_crop_i, size[1]  - Circle_crop_i,1):
#    for i in range(0, size[0], 1):
#        for j in range(0, size[1], 1):
            threeDz = threeD[j][i]
            while (threeDz[2] < 500.0 and threeDz[2] > 60.0):
                print(threeD[j][i][0], threeD[j][i][1], threeD[j][i][2], file=doc)
                break
    doc.close()

    cv2.imwrite("depth.jpg", disp)

    # ----------------------------------------------------------------------------------
    # 乳房判断参数
    ksize_c = 3  # 中值滤波参数
    AR = 2.5  # contour的长细比阈值
    loop_num = 5  # 过滤contour的循环次数
    ksize_d = 5  # 膨胀内核大小
    d_kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (ksize_d, ksize_d))  # 膨胀内核

    # 自适应二值化参数
    # aT_a 越大，二值化能够提取不明显得区域（不易过大）， aT_b 越大，二值化后图像越模糊越离散（不易过大）
    aT_a = 9
    aT_b = 3

    img = imgL
    # 读取左视图
    size = img.shape

    # 裁剪后，图像中点的pix值
    midx = int((size[1] - 2 * Circle_crop_j) * 0.5)
    midy = int((size[0] - 2 * Circle_crop_i) * 0.5)

    # contour面积过滤阈值
    minarea = 50
    maxarea = midx * midy / 2.0

    weight_y = 5.0  # contour质心半径大小过滤的y方向权重，加重权重排除距离y中心方向比较远的点
    fan_area = 0.45  # contour质心距离图片中心的斜率

    img = img[Circle_crop_i:(size[0] - Circle_crop_i), Circle_crop_j:(size[1] - Circle_crop_j)]

    #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #imgg = cv2.medianBlur(img, ksize)

    imgg = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, aT_a, aT_b)

    imgg = cv2.medianBlur(imgg, ksize)

    # ____________________________

    dilated = cv2.dilate(imgg, d_kernel)  # t图片膨胀
    dilated = cv2.medianBlur(dilated, ksize)

    eroded = cv2.erode(imgg, d_kernel)
    eroded = cv2.medianBlur(eroded, ksize)

 #   cv2.imshow('dilateT', dilated)
  #  cv2.imshow('eroded', eroded)

    diff_result = cv2.absdiff(dilated, eroded)
    diff_result = cv2.medianBlur(diff_result, ksize)

   # cv2.imshow('diff_result', diff_result)

    # ____________________________

    # imgg,contours,hierarchy = cv2.findContours(imgg,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    diff_result, contours, hierarchy = cv2.findContours(diff_result, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
  #  print(len(contours))

    # 采用多次循环实现区域面积滤波（一次滤不干净）
    n = loop_num
    while (n > 0):
        i = 0
        for item in contours:
            garea = cv2.contourArea(contours[i])
            if (minarea > garea or garea > maxarea):
                del (contours[i])
            i = i + 1
   #     print(len(contours))
        n -= 1

    # 采用循环实现区域长细比滤波
    n = loop_num
    while (n > 0):
        i = 0
        for item in contours:
            cnt = contours[i]
            minirect = cv2.minAreaRect(cnt)
            print("minirect", minirect)
            aspect_ratio = minirect[1][0] / minirect[1][1]
            if (1.0 / AR > aspect_ratio or aspect_ratio > AR):
                del (contours[i])
            i = i + 1
 #       print(len(contours))
        n -= 1

    cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
 #   cv2.imshow('contour', img)

    R = np.sqrt(midx ** 2 + midy ** 2)  # 判断每个contour的质心到图像中心的距离。R为最大距离，最小的就是乳头
    #print("radius", R)
    radiusl = R
    radiusr = R

    ###--------------初始化乳头位置------------###

    cxl = int(midx / 2)
    cyl = midy
    cxr = int(3 * midx / 2)
    cyr = midy
    Rl = int(midx / 2)
    Rr = int(midx / 2)

    for i in range(len(contours)):
        cnt = contours[i]
        M = cv2.moments(cnt)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        radius = np.sqrt((cx - midx) * (cx - midx) + weight_y * (cy - midy) * (cy - midy))

        if (cx < midx):
            if (radius < radiusl and abs((cy - midy) / (cx - midx)) < fan_area):
                cxl = cx
                cyl = cy
                Rl = radius
                radiusl = radius
        else:
            if (radius < radiusr and abs((cy - midy) / (cx - midx)) < fan_area):
                cxr = cx
                cyr = cy
                Rr = radius
                radiusr = radius

    cv2.imwrite("contour.jpg", img)

    cv2.circle(imgg, (cxl, cyl), int(Rl), (0, 255, 0), 2)
    cv2.circle(imgg, (cxr, cyr), int(Rr), (0, 255, 0), 2)
    cv2.circle(imgg, (cxl, cyl), 2, (0, 255, 0), 3)
    cv2.circle(imgg, (cxr, cyr), 2, (0, 255, 0), 3)

    print("left", cxl, cyl, Rl)
    print("right", cxr, cyr, Rr)

    three3DL = threeD[cyl + Circle_crop_i][cxl + Circle_crop_j]
    three3DR = threeD[cyr + Circle_crop_i][cxr + Circle_crop_j]

    print("left", three3DL)
    print("right", three3DR)

 #   cv2.imshow('img', imgg)
 #   cv2.waitKey(0)
    cv2.imwrite("nipples.jpg", imgg)

    # 输入的x，y应该以未裁剪的图片为标准的 像素点。
    # 在相应的特征点周围找到一个2*av的小面积区域，取中值为该中心点的三维坐标

    # -------#输入的x，y 以未裁剪的图片为标准的像素点,求得该像素点的空间坐标-----------

    nipple_left = point_xyz(cxl, cyl,Circle_crop_i,Circle_crop_j,threeD)
    nipple_right = point_xyz(cxr, cyr,Circle_crop_i,Circle_crop_j,threeD)

    mid_bone_x_pix = int((cxl + cxr) / 2.0)
    mid_bone_y_pix = int((cyl + cyr) / 2.0)
    mid_bone_xyz = point_xyz(mid_bone_x_pix, mid_bone_y_pix,Circle_crop_i,Circle_crop_j,threeD)

    print("left_average", nipple_left)
    print("right_average", nipple_right)
    print("mid_bone_xyz", mid_bone_xyz)

    print("camera—end")
    cv2.destroyAllWindows()

    return nipple_left, nipple_right, mid_bone_xyz


