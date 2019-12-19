#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, time

from linedetector import LineDetector
from motordriver import MotorDriver
from obstacledetector import ObstacleDetector
from filter import MovingAverage

class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.value_filter = MovingAverage(5)
        self.obs_filter = MovingAverage(5)
        self.stop_time = None

    def trace(self):
        line_l, line_r = self.line_detector.detect_lines()
        left, mid, right = self.obstacle_detector.get_distance()
        red, green, yellow = self.line_detector.detect_lights()
        self.line_detector.show_images(line_l, line_r)
        angle = self.steer(line_l, line_r)
        speed = self.accelerate(angle, mid, red, green, yellow)

        if speed == 0:
            if not self.stop_time == None && time.time() - self.stop_time >= 10:
                angle = 45
                speed = 40
            else:
                self.stop_time = time.time()
        else:
            self.stop_time = None

        if angle != None:
            self.driver.drive(angle + 90, speed + 90)

    def steer(self, left, right):
        mid = (left + right) // 2
        self.value_filter.add_sample(mid)

        if len(self.value_filter.data) >= 3:

            if 280 < mid < 360:
                angle = 0
                print(angle)
            else:
                angle = int((self.value_filter.get_wmm() - 320) / 1.5)
                print(angle)

            return angle

    def accelerate(self, angle, mid, red, green, yellow):
        if angle < -50 or angle > 50:
            speed = 40
        else:
            speed = 40

        self.obs_filter.add_sample(mid)
        if len(self.obs_filter.data) >= 3:
            if self.obs_filter.get_wmm() <= 50:
                speed = 0

        if red:
            speed = 0
        elif yellow:
            speed = 25
        elif green:
            speed = 40

        return speed

    def exit(self):
        print('finished')

if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)

