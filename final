import cv2
import keyboard
import sys
import time
import numpy as np
import math
import pyzbar.pyzbar as pyzbar
import pyrealsense2 as rs
import rospy
from std_msgs.msg import String



def realsense():


    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    color_image = np.asanyarray(color_frame.get_data())
    return color_image


#ＱＲコードの検出結果を表現する関数
def decodeDisplay(image):
    barcodes = pyzbar.decode(image)
    rects_list = []
    polygon_points_list = []
    QR_info = []
    # 複数的なQR-CODEがある可能性がありますので、LOOPを設置します
    for barcode in barcodes:
        (x, y, w, h) = barcode.rect
        rects_list.append((x, y, w, h))
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        polygon_points = barcode.polygon
        extract_polygon_points = np.zeros((4, 2), dtype=np.int)

        for idx, points in enumerate(polygon_points):
            point_x, point_y = points  # 默认得到的point_x, point_y是float64类型
            extract_polygon_points[idx] = [point_x, point_y]
        print(extract_polygon_points.shape)  # (4, 2)
        extract_polygon_points = extract_polygon_points.reshape((-1, 1, 2))
        polygon_points_list.append(extract_polygon_points)
        #赤色正方形を描きます
        cv2.polylines(image, [extract_polygon_points], isClosed=True,
                      color=(255, 0, 255), thickness=2,lineType=cv2.LINE_AA)
        # データの種類の変換
        barcodeData = barcode.data.decode("utf-8")
        barcodeType = barcode.type
        text = "{} ({})".format(barcodeData, barcodeType)
        QR_info.append(text)
        cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, (0, 0, 125), 2)
            # ＱＲコードのメッセージを表示する
        print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
        cv2.imwrite(filename,image)
    return image, rects_list, polygon_points_list, QR_info

#ＱＲコードを検出する関数
def detect(vc):

    frame = vc

    # grayじゃなければ、ＱＲコードの検出がかなり難しい
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    im, rects_list, polygon_points_list, QR_info = decodeDisplay(gray)
    # ＱＲコードのメッセージを表示する

    for data_ in zip(rects_list, polygon_points_list, QR_info):
        print(f"data: {data_}")
        x, y, w, h = data_[0]
        polygon_points = data_[1]
        text = data_[2]

        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.polylines(frame, [polygon_points], isClosed=True, color=(255, 0, 255), thickness=2,
                      lineType=cv2.LINE_AA)
        cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, (0, 0, 125), 2)

    return frame








def getDist_P2L(PointP, Pointa, Pointb):
    """计算点到直线的距离
        PointP：定点坐标
        Pointa：直线a点坐标
        Pointb：直线b点坐标
    """
    # 求直线方程
    A = 0
    B = 0
    C = 0
    A = Pointa[1] - Pointb[1]
    B = Pointb[0] - Pointa[0]
    C = Pointa[0] * Pointb[1] - Pointa[1] * Pointb[0]
    # 代入点到直线距离公式
    distance = 0
    distance = (A * PointP[0] + B * PointP[1] + C) / math.sqrt(A * A + B * B)

    return abs(distance)

def position_angle(winx,winy,cenx,ceny):
    if cenx >= winx/2:
        if ceny <= winy/2:
            return math.atan(abs(cenx - winx/2)/abs(ceny-winy/2+0.01))*57.3
        else:
            return math.atan(abs(ceny-winy/2)/abs(cenx - winx/2+0.01))*57.3+90

    else:
        if ceny <= winy/2:
            return -math.atan(abs(cenx - winx/2)/abs(ceny-winy/2+0.01))*57.3
        else:
            return -math.atan(abs(ceny-winy/2)/abs(cenx - winx/2+0.01))*57.3-90

#画像処理して、面積一番大きな目標物を検出します
def VideoCapture(frame):
    global index
    global distance_postion_
    global angle_path_
    global cen_angle_
    max_index = 0
    distance = []
    angle_path = None
    distance_postion = None
    cen_angle = None


    frame_LAB = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    frame_L,frame_A,frame_B = cv2.split(frame_LAB)
    frame_L[0:20,0:50] = 0

    frame_L[win_y-30:win_y,0:50] = 0

    frame_L[0:50, 0:20] = 0

    frame_L[0:50, win_x-20:win_x] = 0



    ret2,i_binary = cv2.threshold(frame_L,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    frame_1 = cv2.morphologyEx(i_binary, cv2.MORPH_ERODE, kernel,iterations=3)
    frame_1 = cv2.morphologyEx(frame_1, cv2.MORPH_CLOSE, kernel,iterations=2)




    r,contours, h = cv2.findContours(frame_1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        global contours_1
        contours_1.append(cv2.contourArea(contour))
    if len(contours_1) != 0:
        max_ = max(contours_1)
        max_index = contours_1.index(max_)
    contours_1 =[]
    if max_index !=0:
        (x, y, w, h) = cv2.boundingRect(contours[max_index])
        if w<100 or h<100 or w>= 500 or h>= 500 :
            pass

        else:#エージを近似する
            peri = cv2.arcLength(contours[max_index], True)
            approx = cv2.approxPolyDP(contours[max_index], 0.02 * peri, True)
            if len(approx) == 4 :
                for counter in range(len(approx) - 1):
                    distance.append(getDist_P2L((320, 240), approx[counter][0], approx[counter + 1][0]))
                    min_distance = distance.index(min(distance))
                    line_1 = approx[min_distance][0]
                    line_2 = approx[min_distance + 1][0]
                    angle_path = np.arctan((line_1[1] - line_2[1]) / (line_1[0] - line_2[0]) + 0.1) * 57.3
            if len(approx) >= 4:
                screenCnt = approx
                cv2.drawContours(frame, [screenCnt], -1, (255, 0, 0), 2)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cen = (x + w / 2, y + h / 2)
                cen_x = round(cen[0])
                cen_y = round(cen[1])

                cv2.circle(frame, (cen_x, cen_y), 2, (0, 0, 255), 1)
                cen_angle =position_angle(win_x,win_y,cen_x,cen_y)

                distance_postion = (math.sqrt(math.pow(( (win_x/2)-cen_x), 2) + math.pow(((win_y/2)-cen_y), 2)))


        index += 1
    index = 0
    if distance_postion != None:
        distance_postion_ = distance_postion
    if cen_angle != None:
        cen_angle_ = cen_angle
    if angle_path != None:
        angle_path_ =angle_path

    return [distance_postion_,cen_angle_],[distance_postion_, angle_path_],frame,frame_1


if __name__ == '__main__':
    i = 0
    I = 1
    index = 0
    contours_1 = []
    win_x = 640
    win_y = 480
    contours_1 = []
    path = 2
    file_name_counter = 1
    filename = f'F:/KB_Sim3_Image_v01/image/{file_name_counter}.jpg'

    camera = 0
    angle_path_ = None
    distance_postion_ = None
    cen_angle_ = None
    image_name = 1


    cv2.namedWindow('video',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('video',win_x,win_y)
    cv2.namedWindow('video2',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('video2',win_x,win_y)

### Ros topic register
    rospy.init_node("ivp_node")
    rospy.logwarn("Info is coming")

    pub = rospy.Publisher("/ImageValueCon",String,queue_size=10)

    rate = rospy.Rate(10)


###
    while not rospy.is_shutdown():
        msg = String()
 ###
        try:
            ###
            if I == 2:
                # Configure depth and color streams
                pipeline = rs.pipeline()
                config = rs.config()

                # Get device product line for setting a supporting resolution
                pipeline_wrapper = rs.pipeline_wrapper(pipeline)
                pipeline_profile = config.resolve(pipeline_wrapper)
                device = pipeline_profile.get_device()
                device_product_line = str(device.get_info(rs.camera_info.product_line))

                found_rgb = False
                for s in device.sensors:
                    if s.get_info(rs.camera_info.name) == 'RGB Camera':
                        found_rgb = True
                        break
                if not found_rgb:
                    print("The demo requires Depth camera with Color sensor")
                    exit(0)

                if device_product_line == 'L500':
                    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
                else:
                    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

                # Start streaming
                pipeline.start(config)


####  realsense QR detect
            if camera == 1:
                vc = realsense()
                frame_QR = detect(vc)

                cv2.imshow('video3',frame_QR)
                key = cv2.waitKey(30)  # この数値はFPSに影響があります、小さくなるなら、FPSが上がります
                I = 0
                file_name_counter += 1
                filename = f'F:/KB_Sim3_Image_v01/image/{file_name_counter}.jpg'

                if False:
                    camera = 0
                    cv2.destroyWindow('video3')
                    I = 1


###### path detect
            if camera == 0:
                if I == 1:
                    cap = cv2.VideoCapture(path,cv2.CAP_DSHOW)
                    I = 0
                ret_alpha, frame = cap.read()
                if ret_alpha is False:
                    continue
###
                stageA, stageB, frame, frame_1 = VideoCapture(frame)
                pub.publish(*stageA,*stageB)
                cv2.imshow('video', frame_1)
                cv2.imshow('video2', frame)
                rate.sleep()
###

                if False:
                    camera = 1
                    cv2.destroyWindow('video')
                    cv2.destroyWindow('video2')
                    I = 2

            if key == 27:
                cv2.destroyWindow('video')
                cv2.destroyWindow('video2')

        except KeyboardInterrupt:
            print("停止")
            vc.release()
            cv2.destroyAllWindows()
            break
