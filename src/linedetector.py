# -*- coding: utf-8 -*-
import time
import cv2
import numpy as np
try:
    import rospy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except:
    pass

class LineDetector:

    def __init__(self, topic):
        self.bridge = CvBridge()
        self.image_width = 640
        self.scan_width, self.scan_height = 130, 40
        self.area_width, self.area_height = 10, 10
        area = self.area_width * self.area_height
        self.pxl_cnt_threshold = area * 0.5
        self.linescan_offset = 15
        self.roi_vertical_pos = 280
        self.left, self.right = -1, -1

        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.mask = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.edge = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)

        rospy.Subscriber(topic, Image, self.conv_image)

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')


    def detect_lines(self):
        # image_width = 640
        # area_width, area_height = 20, 10
        # pixel_cnt_threshold = 0.5 * area_width * area_height
        # gray = cv2.cvtColor(self.cam_img, cv2.COLOR_BGR2GRAY)
        #
        # Blur = cv2.GaussianBlur(gray, (5, 5), 0)
        #
        # sobelx = cv2.Sobel(Blur, cv2.CV_64F, 1, 0, ksize=3)
        # sobelx = cv2.convertScaleAbs(sobelx)
        #
        # sobely = cv2.Sobel(Blur, cv2.CV_64F, 0, 1, ksize=5)
        # sobely = cv2.convertScaleAbs(sobely)
        #
        # img_sobel = cv2.addWeighted(sobelx, 1, sobely, 1, 0)
        #
        # height, width = self.cam_img.shape[:2]
        # vertices1 = np.array(
        #     [[(0, 330), (width / 2 - 130, height / 2), (width / 2 - 130, 330)]],
        #     dtype=np.int32)
        # vertices2 = np.array(
        #     [[(width / 2 + 130, 330), (width / 2 + 130, height / 2), (width, 330)]],
        #     dtype=np.int32)
        #
        # mask = np.zeros_like(img_sobel)
        #
        # if len(img_sobel.shape) > 2:
        #     color = (255, 255, 255)
        # else:
        #     color = 255
        #
        # cv2.fillPoly(mask, vertices1, color)
        # cv2.fillPoly(mask, vertices2, color)
        #
        # ROI_image = cv2.bitwise_and(img_sobel, mask)
        #
        # ret, ROI_image = cv2.threshold(ROI_image, 220, 255, cv2.THRESH_BINARY)
        #
        # zero = np.zeros_like(self.cam_img)
        #
        # cv2.fillPoly(zero, vertices1, (0, 0, 0))
        # cv2.fillPoly(zero, vertices2, (0, 0, 0))
        #
        # bin = ROI_image
        # ROI_image = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)
        # left, right = -1, -1
        #
        # for l in range(80, 290):
        #     area = bin[285: 295, l - 10: l + 10]
        #     if cv2.countNonZero(area) > pixel_cnt_threshold:
        #         left = l
        #         break
        #
        # for r in range(image_width - area_width, 350, -1):
        #     area = bin[285:295, r - 10:r + 10]
        #     if cv2.countNonZero(area) > pixel_cnt_threshold:
        #         right = r
        #         break
        #
        # if left != -1:
        #     lsquare = cv2.rectangle(ROI_image,
        #                             (left - 10, 285),
        #                             (left + 10, 295),
        #                             (0, 255, 0), 3)
        # else:
        #     print("Lost left line")
        #
        # if right != -1:
        #     rsquare = cv2.rectangle(ROI_image,
        #                             (right - 10, 285),
        #                             (right + 10, 295),
        #                             (0, 255, 0), 3)
        # else:
        #     print("Lost right line")
        #
        # cv2.imshow("origin", self.cam_img)
        # cv2.imshow("sobel_image", img_sobel)
        # cv2.imshow("ROI_img", ROI_image)
        # # cv2.imshow("a", a)
        #
        # time.sleep(0.0001)
        #
        # return left, right

        image_width = 640
        area_width, area_height = 20, 10
        pixel_cnt_threshold = 0.66 * area_width * area_height

        gray = cv2.cvtColor(self.cam_img, cv2.COLOR_BGR2GRAY)

        Blur = cv2.GaussianBlur(gray, (7, 7), 0)

        # Sobel mask처리
        sobelx = cv2.Sobel(Blur, cv2.CV_64F, 1, 0, ksize=1)
        sobelx = cv2.convertScaleAbs(sobelx)

        sobely = cv2.Sobel(Blur, cv2.CV_64F, 0, 1, ksize=5)
        sobely = cv2.convertScaleAbs(sobely)

        self.img_sobel = cv2.addWeighted(sobelx, 1, sobely, 1, 0)

        height, width = self.cam_img.shape[:2]
        vertices = np.array(
            [[(0, 330), (width / 2 - 160, height / 2), (width / 2 + 160, height / 2), (width, 340)]],
            dtype=np.int32)

        mask = np.zeros_like(self.img_sobel)  # mask = img와 같은 크기의 빈 이미지

        if len(self.img_sobel.shape) > 2:  # Color 이미지(3채널)라면 :
            color = (255, 255, 255)
        else:  # 흑백 이미지(1채널)라면 :
            color = 255

        # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움
        cv2.fillPoly(mask, vertices, color)

        # 이미지와 color로 채워진 ROI를 합침
        ROI_image = cv2.bitwise_and(self.img_sobel, mask)

        tmp, ROI_image = cv2.threshold(ROI_image, 220, 255, cv2.THRESH_BINARY)
        lines = cv2.HoughLines(ROI_image, 1, np.pi / 180, 140)

        self.zero = np.zeros_like(self.cam_img)

        cv2.fillPoly(self.zero, vertices, (0, 0, 0))

        leftx1, lefty1, leftx2, lefty2 = 2, 0, 1, 0
        rightx1, righty1, rightx2, righty2 = 2, 0, 1, 0

        if lines is not None:
            for i in range(len(lines)):
                for rho, theta in lines[i]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))

                    if float(y2 - y1) / float(x2 - x1) < -0.2 and float(y2 - y1) / float(x2 - x1) > -0.8:
                        if float(y2 - y1) / float(x2 - x1) < float(lefty2 - lefty1) / float(leftx2 - leftx1):
                            leftx1, lefty1, leftx2, lefty2 = x1, y1, x2, y2
                    elif float(y2 - y1) / float(x2 - x1) > 0.2 and float(y2 - y1) / float(x2 - x1) < 0.8:
                        if float(y2 - y1) / float(x2 - x1) > float(righty2 - righty1) / float(rightx2 - rightx1):
                            rightx1, righty1, rightx2, righty2 = x1, y1, x2, y2

        cv2.line(self.zero, (leftx1, lefty1), (leftx2, lefty2), (0, 0, 255), 5, cv2.LINE_AA)
        cv2.line(self.zero, (rightx1, righty1), (rightx2, righty2), (0, 0, 255), 5, cv2.LINE_AA)

        bin = cv2.cvtColor(self.zero, cv2.COLOR_BGR2GRAY)

        left, right = -1, -1

        for l in range(80, 290):
            area = bin[285: 295, l - 10: l + 10]
            if cv2.countNonZero(area) > pixel_cnt_threshold:
                left = l
                break

        for r in range(image_width - area_width, 350, -1):
            area = bin[285:295, r - 10:r + 10]
            if cv2.countNonZero(area) > pixel_cnt_threshold:
                right = r
                break

        if right == -1:
            right = 560
        if left == -1:
            left = 80

        print left, right
        return left, right

    def detect_lights(self):

        font = cv2.FONT_HERSHEY_SIMPLEX
        img = self.cam_img
        cimg = img
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # color range
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([90, 255, 255])
        # lower_yellow = np.array([15,100,100])
        # upper_yellow = np.array([35,255,255])
        lower_yellow = np.array([15, 150, 150])
        upper_yellow = np.array([35, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        maskg = cv2.inRange(hsv, lower_green, upper_green)
        masky = cv2.inRange(hsv, lower_yellow, upper_yellow)
        maskr = cv2.add(mask1, mask2)

        size = img.shape
        # print size

        # hough circle detect
        r_circles = cv2.HoughCircles(maskr, cv2.HOUGH_GRADIENT, 1, 80,
                                     param1=50, param2=10, minRadius=0, maxRadius=100)

        g_circles = cv2.HoughCircles(maskg, cv2.HOUGH_GRADIENT, 1, 60,
                                     param1=50, param2=10, minRadius=0, maxRadius=100)

        y_circles = cv2.HoughCircles(masky, cv2.HOUGH_GRADIENT, 1, 30,
                                     param1=50, param2=5, minRadius=0, maxRadius=100)

        red, green, yellow = False

        # traffic light detect
        r = 5
        bound = 4.0 / 10
        if r_circles is not None:
            r_circles = np.uint16(np.around(r_circles))

            for i in r_circles[0, :]:
                if i[0] > size[1] or i[1] > size[0] or i[1] > size[0] * bound:
                    continue

                h, s = 0.0, 0.0
                for m in range(-r, r):
                    for n in range(-r, r):

                        if (i[1] + m) >= size[0] or (i[0] + n) >= size[1]:
                            continue
                        h += maskr[i[1] + m, i[0] + n]
                        s += 1
                if h / s > 50:
                    red = True

        if g_circles is not None:
            g_circles = np.uint16(np.around(g_circles))

            for i in g_circles[0, :]:
                if i[0] > size[1] or i[1] > size[0] or i[1] > size[0] * bound:
                    continue

                h, s = 0.0, 0.0
                for m in range(-r, r):
                    for n in range(-r, r):

                        if (i[1] + m) >= size[0] or (i[0] + n) >= size[1]:
                            continue
                        h += maskg[i[1] + m, i[0] + n]
                        s += 1
                if h / s > 100:
                    green = True

        if y_circles is not None:
            y_circles = np.uint16(np.around(y_circles))

            for i in y_circles[0, :]:
                if i[0] > size[1] or i[1] > size[0] or i[1] > size[0] * bound:
                    continue

                h, s = 0.0, 0.0
                for m in range(-r, r):
                    for n in range(-r, r):

                        if (i[1] + m) >= size[0] or (i[0] + n) >= size[1]:
                            continue
                        h += masky[i[1] + m, i[0] + n]
                        s += 1
                if h / s > 50:
                    yellow = True

        return red, green, yellow

    def show_images(self, left, right):
        if cv2.waitKey(1) & 0xFF == 27:
            exit()

        if left != -1:
            lsquare = cv2.rectangle(self.zero,
                                    (left - 10, 285),
                                    (left + 10, 295),
                                    (0, 255, 0), 3)
        else:
            print("Lost left line")

        if right != -1:
            rsquare = cv2.rectangle(self.zero,
                                    (right - 10, 285),
                                    (right + 10, 295),
                                    (0, 255, 0), 3)
        else:
            print("Lost right line")

        cv2.imshow("origin", self.cam_img)
        cv2.imshow("view zero", self.zero)
        cv2.imshow("sobel_image", self.img_sobel)
