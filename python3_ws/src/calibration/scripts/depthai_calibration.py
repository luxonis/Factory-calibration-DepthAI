#!/usr/bin/env python3

import cv2
import sys
import copy
import platform
import signal
import subprocess
import json
import csv
import time
import numpy as np
import os
from pathlib import Path
import shutil
from datetime import datetime, timedelta
import rospy
from std_msgs.msg import String
from calibration.srv import Capture
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import depthai as dai
import consts.resource_paths
from depthai_helpers.calibration_utils import *

from depthai_helpers.pygame_checkbox import Checkbox, pygame_render_text
import pygame
from pygame.locals import *

from depthai_helpers import utils
os.environ['SDL_VIDEO_WINDOW_POS'] = '100,50'

on_embedded = platform.machine().startswith(
    'arm') or platform.machine().startswith('aarch64')


white = [255, 255, 255]
orange = [143, 122, 4]
red = [230, 9, 9]
green = [4, 143, 7]
black = [0, 0, 0]
pygame.init()


class depthai_calibration_node:
    def __init__(self, depthai_args):
        self.package_path = depthai_args['package_path']
        self.args = depthai_args
        self.bridge = CvBridge()
        self.is_service_active = False

        self.disp = pygame.display
        self.screen = self.disp.set_mode((800, 600))
        self.screen.fill(white)
        self.disp.set_caption("Calibration - Device check ")

        self.ccm_selector()

        self.focus_value = 135
        self.defaultLensPosition = 135
        self.focusSigmaThreshold = 22
        if self.rgbCcm == 'Sunny':
            self.focus_value = 135
        elif self.rgbCcm == 'KingTop':
            self.focus_value = 135
        elif self.rgbCcm == 'ArduCam':
            self.focus_value = 135

        # self.frame_count = 0
        self.init_time = time.time()
        if self.args['board']:
            board_path = Path(self.args['board'])
            print(board_path)
            if not board_path.exists():
                board_path = Path(consts.resource_paths.boards_dir_path) / \
                    Path(self.args['board'].upper()).with_suffix('.json')
                print(board_path)
                if not board_path.exists():
                    raise ValueError(
                        'Board config not found: {}'.format(board_path))
            with open(board_path) as fp:
                self.board_config = json.load(fp)

        self.aruco_dictionary = cv2.aruco.Dictionary_get(
            cv2.aruco.DICT_4X4_1000)

        # Connection checks ----------->
        title = "Device Status"
        pygame_render_text(self.screen, title, (350, 20), orange, 50)
        self.auto_checkbox_names = []
        
        if self.args['usbMode']:
            self.auto_checkbox_names.append("USB3")
        header = ['time', 'Mx_serial_id','Mono-CCM', 'RGB-CCM',
                  'left_camera', 'right_camera', 'rgb_camera', 'Epipolar error L-R', 'Epipolar error R-Rgb', 'RGB Reprojection Error']

        if not self.args['disableLR']:
            self.auto_checkbox_names.append("Left Camera Conencted")
            self.auto_checkbox_names.append("Right Camera Conencted")
            self.auto_checkbox_names.append("Left Stream")
            self.auto_checkbox_names.append("Right Stream")
        if not self.args['disableRgb']:
            self.auto_checkbox_names.append("Rgb Camera Conencted")
            self.auto_checkbox_names.append("Rgb Stream")
        
        log_file = self.args['log_path'] + "/calibration_logs_" + arg['board'] + ".csv"
        if not os.path.exists(log_file):
            with open(log_file, mode='w') as log_fopen:
                log_csv_writer = csv.writer(log_fopen, delimiter=',')
                log_csv_writer.writerow(header)

        y = 110
        x = 200
        self.start_disp = False
        font = pygame.font.Font(None, 20)
        self.auto_checkbox_dict = {}

        for i in range(len(self.auto_checkbox_names)):
            w, h = font.size(self.auto_checkbox_names[i])
            x_axis = x - w
            y_axis = y + (40*i)
            font_surf = font.render(self.auto_checkbox_names[i], True, green)
            self.screen.blit(font_surf, (x_axis, y_axis))
            self.auto_checkbox_dict[self.auto_checkbox_names[i]] = Checkbox(self.screen, x + 10, y_axis-5, outline_color=green,
                                                                            check_color=green, check=False)

        # text = 'call rosservice of device_status_handler to update the device status'
        for i in range(len(self.auto_checkbox_names)):
            self.auto_checkbox_dict[self.auto_checkbox_names[i]].render_checkbox()
        pygame.draw.rect(self.screen, red, no_button)
        pygame_render_text(self.screen, 'Exit', (500, 505))
        self.no_active = False
        self.click = False
        # self.disp.update()
        # creating services and publishers at the end to avoid calls before initialization
        self.capture_srv = rospy.Service(
            self.args["capture_service_name"], Capture, self.capture_servive_handler)
        self.calib_srv = rospy.Service(
            self.args["calibration_service_name"], Capture, self.calibration_servive_handler)
        self.dev_status_srv = rospy.Service(
            "device_status", Capture, self.device_status_handler)
        self.focus_setting_srv = rospy.Service(
            "rgb_focus_adjuster", Capture, self.rgb_focus_adjuster)
        # self.rgb_focus_srv = rospy.Service(
        #     "set_rgb_focus", Capture, self.rgb_focus_handler)

        self.args['cameraModel'] = 'perspective'
        if not self.args['disableLR']:
            self.image_pub_left = rospy.Publisher("left", Image, queue_size=10)
            self.image_pub_right = rospy.Publisher("right", Image, queue_size=10)
        if not self.args['disableRgb']:
            self.image_pub_color = rospy.Publisher("color", Image, queue_size=10)
        self.device = None

    def ccm_selector(self):
        title = "Select the mono Camera and RGB camera vendor"
        pygame_render_text(self.screen, title, (70, 20), black, 40)
        title = "MONO Camera Vendor                  RGB Camera Vendor"
        pygame_render_text(self.screen, title, (200, 100), green, 25)
        ccm_names = ['Sunny', 'KingTop', 'ArduCam']
        ccm_names_dict = dict()
        y = 200
        x = 110
        font = pygame.font.Font(None, 30)
        for i in range(len(ccm_names)):
            w, h = font.size(ccm_names[i])
            x_axis = x - w
            y_axis = y + (60*i)
            font_surf = font.render(ccm_names[i], True, black)
            self.screen.blit(font_surf, (x_axis, y_axis))
            ccm_names_dict[ccm_names[i]] = []
            ccm_names_dict[ccm_names[i]].append(Checkbox(self.screen, x + 150, y_axis-5, outline_color=green,
                                                                            check_color=green, check=False, disable_pass = True))
            ccm_names_dict[ccm_names[i]].append(Checkbox(self.screen, x + 420, y_axis-5, outline_color=green, 
                                                                            check_color=green, check=False, disable_pass = True))
            ccm_names_dict[ccm_names[i]][0].render_checkbox()
            ccm_names_dict[ccm_names[i]][1].render_checkbox()
            fill_color = pygame.Rect(20, y_axis + 40, 750, 2)
            pygame.draw.rect(self.screen, black, fill_color)
        
        next_button =  pygame.Rect(600, 430, 60, 35)
        pygame.draw.rect(self.screen, orange, next_button)
        pygame_render_text(self.screen, 'Next', (605, 440))

        is_saved = False
        is_mono_selected = False
        is_rgb_selected = False
        self.monoCcm = None
        self.rgbCcm = None

        while not is_saved:
            self.disp.update()
            for event in pygame.event.get():
                # catching NEXT button and checkboxes clicks
                if event.type == pygame.MOUSEMOTION:
                    x, y = event.pos
                    px, py, w, h = next_button
                    if px < x < px + w and py < y < py + h:
                        active = True
                    else:
                        active = False

                if event.type == pygame.MOUSEBUTTONDOWN:
                    click = True
                if event.type == pygame.MOUSEBUTTONUP:
                    if active and not is_saved and click:
                        click = False
                        if is_rgb_selected and is_mono_selected:
                            is_saved = True                
                            pygame.draw.rect(self.screen, green, next_button)
                            pygame_render_text(self.screen, 'Next', (605, 440))
                        else:
                            pygame_render_text(self.screen, "Select the type of module before clicking next ", (150, 480), red)
                    if active and is_saved and click:
                        click = False
                        pygame_render_text(self.screen, "Saving Selection", (605, 480), green)

                ccm1 = ccm_names_dict[ccm_names[0]]
                ccm2 = ccm_names_dict[ccm_names[1]]
                ccm3 = ccm_names_dict[ccm_names[2]]

                ccm1[0].update_checkbox_rel(event, ccm2[0], ccm3[0])
                ccm2[0].update_checkbox_rel(event, ccm1[0], ccm3[0])
                ccm3[0].update_checkbox_rel(event, ccm1[0], ccm2[0]) 
                ccm1[0].render_checkbox()
                ccm2[0].render_checkbox()
                ccm3[0].render_checkbox()

                ccm1[1].update_checkbox_rel(event, ccm2[1], ccm3[1])
                ccm2[1].update_checkbox_rel(event, ccm1[1], ccm3[1])
                ccm3[1].update_checkbox_rel(event, ccm1[1], ccm2[1]) 
                ccm1[1].render_checkbox()
                ccm2[1].render_checkbox()
                ccm3[1].render_checkbox()
                
                if not is_mono_selected:
                    if ccm1[0].is_checked() or ccm2[0].is_checked() or ccm3[0].is_checked():
                        is_mono_selected = True

                if not is_rgb_selected:
                    if ccm1[1].is_checked() or ccm2[1].is_checked() or ccm3[1].is_checked():
                        is_rgb_selected = True

        for i in range(len(ccm_names)):
            if ccm_names_dict[ccm_names[i]][0].is_checked():
                self.monoCcm = ccm_names[i]
            if ccm_names_dict[ccm_names[i]][1].is_checked():
                self.rgbCcm = ccm_names[i]
        
        self.screen.fill(white)
        self.disp.update()
        print("selected CCMS:")
        print(self.monoCcm)
        print(self.rgbCcm)

    def create_pipeline(self):
        pipeline = dai.Pipeline()

        if not self.args['disableLR']:
            cam_left = pipeline.createMonoCamera()
            cam_right = pipeline.createMonoCamera()

            xout_left = pipeline.createXLinkOut()
            xout_right = pipeline.createXLinkOut()

            if self.args['swapLR']:
                cam_left.setBoardSocket(dai.CameraBoardSocket.RIGHT)
                cam_right.setBoardSocket(dai.CameraBoardSocket.LEFT)
            else:
                cam_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
                cam_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
                    
            cam_left.setResolution(
                dai.MonoCameraProperties.SensorResolution.THE_800_P)

            cam_right.setResolution(
                dai.MonoCameraProperties.SensorResolution.THE_800_P)
            cam_left.setFps(5)
            cam_right.setFps(5)

            xout_left.setStreamName("left")
            cam_left.out.link(xout_left.input)

            xout_right.setStreamName("right")
            cam_right.out.link(xout_right.input)

        if not self.args['disableRgb']:
            rgb_cam = pipeline.createColorCamera()
            controlIn = pipeline.createXLinkIn()
            
            rgb_cam.setResolution(
                dai.ColorCameraProperties.SensorResolution.THE_4_K)
            rgb_cam.setInterleaved(False)
            rgb_cam.setBoardSocket(dai.CameraBoardSocket.RGB)
            rgb_cam.setFps(5)
            rgb_cam.setIspScale(1, 3)
            rgb_cam.initialControl.setManualFocus(self.defaultLensPosition)
            self.focus_value = self.defaultLensPosition

            controlIn.setStreamName('control')
            controlIn.out.link(rgb_cam.inputControl)

            xout_rgb_isp = pipeline.createXLinkOut()
            xout_rgb_isp.setStreamName("rgb")
            rgb_cam.isp.link(xout_rgb_isp.input)

        return pipeline

    def capture_exit(self):
        is_clicked = False
        for event in pygame.event.get():
            # self.disp.update()
            if event.type == pygame.MOUSEMOTION:
                x, y = event.pos
                px, py, w, h = no_button
                if px < x < px + w and py < y < py + h:
                    self.no_active = True
                    pygame.draw.rect(self.screen, orange, no_button)
                    pygame_render_text(self.screen, 'Exit', (500, 505))
                else:
                    self.no_active = False
                    pygame.draw.rect(self.screen, red, no_button)
                    pygame_render_text(self.screen, 'Exit', (500, 505))

            if event.type == pygame.MOUSEBUTTONDOWN and self.no_active:
                print("setting click")
                self.click = True
            if event.type == pygame.MOUSEBUTTONUP:
                if self.no_active and self.click:
                    print("No clicked")
                    is_clicked = True
                    break
        return is_clicked

    def publisher(self):
        while not rospy.is_shutdown():
            if self.capture_exit():
                print("signaling...")
                rospy.signal_shutdown("Finished calibration")
            # if self.start_disp:
            self.disp.update()

            if not self.is_service_active and self.device is not None and not self.device.isClosed(): 
                if not self.args['disableLR']:
                    left_frame = self.left_camera_queue.tryGet()
                    if left_frame is not None:
                        self.image_pub_left.publish(
                            self.bridge.cv2_to_imgmsg(left_frame.getCvFrame(), "passthrough"))
                    
                    right_frame = self.right_camera_queue.tryGet()
                    if right_frame is not None:
                        self.image_pub_right.publish(
                            self.bridge.cv2_to_imgmsg(right_frame.getCvFrame(), "passthrough"))
                
                if not self.args['disableRgb']:
                    rgb_frame = self.rgb_camera_queue.tryGet()
                    if rgb_frame is not None:
                        frame = cv2.cvtColor(rgb_frame.getCvFrame(), cv2.COLOR_BGR2GRAY) 
                        self.image_pub_color.publish(
                            self.bridge.cv2_to_imgmsg(frame, "passthrough"))
    
    def cvt_bgr(self, packet):
        meta = packet.getMetadata()
        w = meta.getFrameWidth()
        h = meta.getFrameHeight()
        # print((h, w))
        packetData = packet.getData()
        yuv420p = packetData.reshape((h * 3 // 2, w))
        return cv2.cvtColor(yuv420p, cv2.COLOR_YUV2BGR_IYUV)

    def parse_frame(self, frame, stream_name, file_name):
        if frame is None:
            print("Frame with stream name -> {} was None".format(stream_name))
            return True

        file_name += '.png'
        # filename = image_filename(stream_name, self.current_polygon, self.images_captured)
        print(self.package_path + "/dataset/{}/{}".format(stream_name, file_name))
        ds_path = self.package_path + "/dataset/{}".format(stream_name)
        # print(ds_path)
        if not os.path.exists(ds_path):
            os.makedirs(ds_path)

        cv2.imwrite(self.package_path +
                    "/dataset/{}/{}".format(stream_name, file_name), frame)
        print("py: Saved image as: " + str(file_name) +
              "in folder ->" + stream_name)

        ## adding backup
        self.backup_ds(stream_name, file_name, frame)    
        return True

    def retest(self):
        retest_button =  pygame.Rect(400, 430, 100, 35)
        pygame.draw.rect(self.screen, orange, retest_button)
        pygame_render_text(self.screen, 'RETEST', (410, 440))

        is_reset = False
        is_exit = False
        while True:
            for event in pygame.event.get():
                # self.disp.update()
                if event.type == pygame.MOUSEMOTION:
                    x, y = event.pos
                    px, py, w, h = no_button
                    rx, ry, rw, rh = retest_button

                    if px < x < px + w and py < y < py + h:
                        self.no_active = True
                        pygame.draw.rect(self.screen, orange, no_button)
                        pygame_render_text(self.screen, 'Exit', (500, 505))
                    else:
                        self.no_active = False
                        pygame.draw.rect(self.screen, red, no_button)
                        pygame_render_text(self.screen, 'Exit', (500, 505))

                    if rx < x < rx + rw and ry < y < ry + rh:
                        self.reset_active = True
                        pygame.draw.rect(self.screen, green, retest_button)
                        pygame_render_text(self.screen, 'RETEST', (410, 440))
                    else:
                        self.reset_active = False
                        pygame.draw.rect(self.screen, orange, retest_button)
                        pygame_render_text(self.screen, 'RETEST', (410, 440))

                if event.type == pygame.MOUSEBUTTONDOWN and self.no_active:
                    print("setting click")
                    self.click = True
                if event.type == pygame.MOUSEBUTTONDOWN and self.reset_active:
                    print("setting click")
                    self.click = True

                if event.type == pygame.MOUSEBUTTONUP:
                    if self.no_active and self.click:
                        print("No clicked")
                        rospy.signal_shutdown("Shutting down calibration")
                    if self.reset_active and self.click:
                        print("Reset clicked")
                        return

    def backup_ds(self, stream_name, file_name, frame):
        now_tim = datetime.now()
        local_ds = self.args['ds_backup_path'] + '/' + now_tim.strftime("%m_%d_%Y_%H") + '/{}'.format(stream_name)
        if not os.path.exists(local_ds):
            os.makedirs(local_ds)
        cv2.imwrite(local_ds + "/{}".format(file_name), frame)

    def is_markers_found(self, frame):
        marker_corners, _, _ = cv2.aruco.detectMarkers(
            frame, self.aruco_dictionary)
        print("Markers count ... {}".format(len(marker_corners)))
        return not (len(marker_corners) < 30)

    def close_device(self):
        if hasattr(self, 'left_camera_queue') and hasattr(self, 'right_camera_queue'):
            delattr(self, 'left_camera_queue')
            delattr(self, 'right_camera_queue')
        if hasattr(self, 'rgb_camera_queue') and hasattr(self, 'rgb_control_queue'):
            delattr(self, 'rgb_camera_queue')
            delattr(self, 'rgb_control_queue')
        self.device.close()

    def device_status_handler(self, req):
        self.is_service_active = True
        self.start_disp = True # TODO(sachin): Change code to Use this for display 

        fill_color = pygame.Rect(280, 400, 450, 55)
        pygame.draw.rect(self.screen, white, fill_color)

        # to remove previous date and stuff
        fill_color = pygame.Rect(400, 70, 500, 300)
        pygame.draw.rect(self.screen, white, fill_color)
        now_time = datetime.now()
        text = "date/time : " + now_time.strftime("%m-%d-%Y %H:%M:%S")
        pygame_render_text(self.screen, text, (400, 80), black, 30)
        """ 
        dev_info = self.device.getDeviceInfo()
        text = "device Mx_id : " + dev_info.getMxId()
        pygame_render_text(self.screen, text, (400, 120), black, 30)
        rospy.sleep(1)
        """
        fill_color_2 = pygame.Rect(50, 520, 400, 80)
        pygame.draw.rect(self.screen, white, fill_color_2)

        for _, checkbox in self.auto_checkbox_dict.items():
            checkbox.setUnattended()
            checkbox.render_checkbox()

        if self.device is not None:
            if not self.device.isClosed():
                self.device.isClose()

        finished = False
        while not finished:
            if self.capture_exit():
                rospy.signal_shutdown("Stopping calibration")
            while self.device is None or self.device.isClosed():
                if self.capture_exit():
                    rospy.signal_shutdown("Stopping calibration")
                # TODO(Sachin): add a check for available devices..
                searchTime = timedelta(seconds=80)
                isFound, deviceInfo = dai.Device.getAnyAvailableDevice(searchTime)
                if isFound:
                    self.device = dai.Device() 
                    cameraList = self.device.getConnectedCameras()
                    fill_color_2 = pygame.Rect(390, 120, 500, 100)
                    pygame.draw.rect(self.screen, white, fill_color_2)

                    dev_info = self.device.getDeviceInfo()
                    text = "device Mx_id : " + dev_info.getMxId()
                    pygame_render_text(self.screen, text, (400, 120), black, 30)
                    text = "Device Connected!!!"
                    pygame_render_text(self.screen, text, (400, 150), green, 30)

                    lost_camera = False
                    if not self.args['disableLR']:
                        if dai.CameraBoardSocket.LEFT not in cameraList:
                            self.auto_checkbox_dict["Left Camera Conencted"].uncheck()
                            lost_camera = True
                        else:
                            self.auto_checkbox_dict["Left Camera Conencted"].check()
                        self.auto_checkbox_dict["Left Camera Conencted"].render_checkbox()
                        
                        if dai.CameraBoardSocket.RIGHT not in cameraList:
                            self.auto_checkbox_dict["Right Camera Conencted"].uncheck()
                            lost_camera = True
                        else:
                            self.auto_checkbox_dict["Right Camera Conencted"].check()
                        self.auto_checkbox_dict["Right Camera Conencted"].render_checkbox()

                    if not self.args['disableRgb']:
                        if dai.CameraBoardSocket.RGB not in cameraList:
                            self.auto_checkbox_dict["Rgb Camera Conencted"].uncheck()
                            lost_camera = True
                        else:
                            self.auto_checkbox_dict["Rgb Camera Conencted"].check()
                        self.auto_checkbox_dict["Rgb Camera Conencted"].render_checkbox()
                        # print(self.device.getUsbSpeed())

                    if self.args['usbMode']:
                        if self.device.getUsbSpeed() == dai.UsbSpeed.SUPER:
                            self.auto_checkbox_dict["USB3"].check()
                        else:
                            lost_camera = True
                            self.auto_checkbox_dict["USB3"].uncheck()
                        self.auto_checkbox_dict["USB3"].render_checkbox()

                    if not lost_camera:    
                        pipeline = self.create_pipeline()
                        self.device.startPipeline(pipeline)
                        if not self.args['disableLR']:
                            self.left_camera_queue = self.device.getOutputQueue("left", 5, False)
                            self.right_camera_queue = self.device.getOutputQueue("right", 5, False)
                        if not self.args['disableRgb']:
                            self.rgb_camera_queue  = self.device.getOutputQueue("rgb", 5, False)
                            self.rgb_control_queue  = self.device.getInputQueue("control", 5, False)
                    else:
                        print("Closing Device...")

                        fill_color_2 = pygame.Rect(390, 150, 220, 35)
                        pygame.draw.rect(self.screen, white, fill_color_2)
                        text = "Device Disconnected!!!"
                        pygame_render_text(self.screen, text, (400, 150), red, 30)
                        text = "Click RETEST when device is ready!!!"
                        pygame_render_text(self.screen, text, (400, 180), red, 30)

                        self.close_device()
                        self.retest()
                        print("Restarting Device...")

                        fill_color_2 = pygame.Rect(390, 430, 120, 35)
                        pygame.draw.rect(self.screen, white, fill_color_2)
            left_mipi = False
            right_mipi = False
            rgb_mipi = False

            for _ in range(120):
                if not self.args['disableLR']:
                    left_frame = self.left_camera_queue.tryGet()
                    if left_frame is not None:
                        left_mipi = True                
                        self.image_pub_left.publish(
                            self.bridge.cv2_to_imgmsg(left_frame.getCvFrame(), "passthrough"))

                    right_frame = self.right_camera_queue.tryGet()
                    if right_frame is not None:
                        right_mipi = True
                        self.image_pub_right.publish(
                            self.bridge.cv2_to_imgmsg(right_frame.getCvFrame(), "passthrough"))
                else:
                    left_mipi = True
                    right_mipi = True

                if not self.args['disableRgb']:
                    rgb_frame = self.rgb_camera_queue.tryGet()
                    if rgb_frame is not None:
                        rgb_mipi = True
                        frame = cv2.cvtColor(rgb_frame.getCvFrame(), cv2.COLOR_BGR2GRAY) 
                        self.image_pub_color.publish(
                            self.bridge.cv2_to_imgmsg(frame, "passthrough"))
                else:
                    rgb_mipi = True

                if left_mipi and right_mipi and rgb_mipi:
                    break
                rospy.sleep(1)

            if not self.args['disableLR']:
                if not left_mipi:
                    self.auto_checkbox_dict["Left Stream"].uncheck()
                else:
                    self.auto_checkbox_dict["Left Stream"].check()

                if not right_mipi:
                    self.auto_checkbox_dict["Right Stream"].uncheck()
                else:
                    self.auto_checkbox_dict["Right Stream"].check()

            if not self.args['disableRgb']:
                if not rgb_mipi:
                    self.auto_checkbox_dict["Rgb Stream"].uncheck()
                else:
                    self.auto_checkbox_dict["Rgb Stream"].check()

            for i in range(len(self.auto_checkbox_names)):
                self.auto_checkbox_dict[self.auto_checkbox_names[i]].render_checkbox()

            isAllPassed = True
            for key in self.auto_checkbox_dict.keys():
                isAllPassed = isAllPassed and self.auto_checkbox_dict[key].is_checked

            if isAllPassed:
                finished = True
            else:
                self.close_device()
                self.retest()

        dataset_path = Path(self.package_path + "/dataset")
        if 1 and dataset_path.exists():
            shutil.rmtree(str(dataset_path))
        dev_info = self.device.getDeviceInfo()
        self.is_service_active = False
        return (finished, dev_info.getMxId())

    def capture_servive_handler(self, req):
        print("Capture image Service Started")
        recent_left = None
        recent_right = None
        recent_color = None
        finished = False
        self.is_service_active = True
        rospy.sleep(1)

        # TODO(Sachin): Add time synchronization here and get the most recent frame instead.
        while not finished:
            # left_frame = self.left_camera_queue.get()
            if not self.args['disableLR']:
                left_frame = self.left_camera_queue.getAll()[-1]
                self.image_pub_left.publish(
                            self.bridge.cv2_to_imgmsg(left_frame.getCvFrame(), "passthrough"))

                # right_frame = self.right_camera_queue.get()
                right_frame = self.right_camera_queue.getAll()[-1]
                self.image_pub_right.publish(
                            self.bridge.cv2_to_imgmsg(right_frame.getCvFrame(), "passthrough"))
  
                recent_left = left_frame.getCvFrame()
                recent_right = right_frame.getCvFrame()

            # rgb_frame = self.rgb_camera_queue.get()
            if not self.args['disableRgb']:
                rgb_frame = self.rgb_camera_queue.getAll()[-1]
                recent_color = cv2.cvtColor(rgb_frame.getCvFrame(), cv2.COLOR_BGR2GRAY)
                self.image_pub_color.publish(
                        self.bridge.cv2_to_imgmsg(recent_color, "passthrough"))


            if (not self.args['disableLR']) and (not self.args['disableRgb']):
                if recent_left is not None and recent_right is not None and recent_color is not None:
                    finished = True
            elif self.args['disableLR'] and (not self.args['disableRgb']) and recent_color is not None:
                finished = True
            elif self.args['disableRgb'] and (not self.args['disableLR']) and recent_left is not None and recent_right is not None:
                finished = True

        if not self.args['disableLR']:
            is_board_found_l = self.is_markers_found(recent_left)
            is_board_found_r = self.is_markers_found(recent_right)
        else:
            is_board_found_l = True
            is_board_found_r = True

        if not self.args['disableRgb']:
            is_board_found_rgb = self.is_markers_found(recent_color)
        else:
            is_board_found_rgb = True

        if is_board_found_l and is_board_found_r and is_board_found_rgb:
            self.parse_frame(recent_left, "left", req.name)
            self.parse_frame(recent_right, "right", req.name)
            self.parse_frame(recent_color, "rgb", req.name)
        else:
            self.is_service_active = False
            self.parse_frame(recent_left, "left_not", req.name)
            self.parse_frame(recent_right, "right_not", req.name)
            self.parse_frame(recent_color, "rgb_not", req.name)
            self.close_device()
            return (False, "Calibration board not found")

        print("Service ending")
        self.is_service_active = False
        return (True, "No Error")

    def rgb_focus_adjuster(self, req):
        mode = 0
        localLensPosition = self.defaultLensPosition
        if not self.args['disableRgb']:
            isFocused = False
            while not isFocused:
                rgb_frame = self.rgb_camera_queue.getAll()[-1]
                rgb_gray = cv2.cvtColor(rgb_frame.getCvFrame(), cv2.COLOR_BGR2GRAY)
                laplace_res = cv2.Laplacian(rgb_gray, cv2.CV_64F)
                mu, sigma = cv2.meanStdDev(laplace_res)
                print("at lens pose {},  Std Deviationn {}".format(localLensPosition, sigma))
                if sigma < self.focusSigmaThreshold:
                    lenPosDiff = localLensPosition - self.defaultLensPosition
                    if mode == 0:
                        if lenPosDiff >= 0 and lenPosDiff < 10:
                            localLensPosition += 1
                        else:
                            mode = 1
                            localLensPosition = self.defaultLensPosition
                            continue

                    if mode == 1:
                        if lenPosDiff > -10 and lenPosDiff <= 0:
                            localLensPosition -= 1
                        else:
                            print("Printing Lens Position: {}".format(localLensPosition))
                            self.close_device()
                            return (False, "RGB Camera out of Focus ")

                    ctrl = dai.CameraControl()
                    ctrl.setManualFocus(localLensPosition)
                    print("Sending Control")
                    self.rgb_control_queue.send(ctrl)
                    rospy.sleep(2)
                else:
                    isFocused = True
                    self.focus_value = localLensPosition
                    return (True, "RGB Camera image in Focus ")

    def calibration_servive_handler(self, req):
        self.is_service_active = True
        print("calibration Service Started")
        # pygame.draw.rect(self.screen, white, no_button)

        # mx_serial_id = self.device.get_mx_id()
        dev_info = self.device.getDeviceInfo()
        mx_serial_id = dev_info.getMxId()
        calib_dest_path = os.path.join(
            arg['calib_path'], arg["board"] + '_' + mx_serial_id + '.json')
        
        stereo_calib = StereoCalibration()
        rgb_reproject_error, avg_epipolar_error_lr, avg_epipolar_error_r_rgb, calib_data = stereo_calib.calibrate(
            self.package_path + "/dataset",
            self.args['square_size_cm'],
            self.args['marker_size_cm'],
            self.args['squares_x'],
            self.args['squares_y'],
            self.args['cameraModel'],
            not self.args['disableLR'], # turn on L-R calibration
            not self.args['disableRgb'], # turn on rgb calibration
            False) # Turn off enable disp rectify
 
        start_time = datetime.now()
        time_stmp = start_time.strftime("%m-%d-%Y %H:%M:%S")

        log_list = [time_stmp, mx_serial_id]
        log_list.append(self.monoCcm)
        log_list.append(self.rgbCcm)

        if self.args['disableLR']:
            log_list.append("LR Disabled")
            log_list.append("LR Disabled")
        else:
            log_list.append("Calibrated")
            log_list.append("Calibrated")

        if self.args['disableRgb']:
            log_list.append("Rgb Disabled")
        else:
            log_list.append("Calibrated")

        log_list.append(avg_epipolar_error_lr)
        log_list.append(avg_epipolar_error_r_rgb)
        log_list.append(rgb_reproject_error)

        log_file = self.args['log_path'] + "/calibration_logs_" + arg['board'] + ".csv"
        with open(log_file, mode='a') as log_fopen:
            # header =
            log_csv_writer = csv.writer(log_fopen, delimiter=',')
            log_csv_writer.writerow(log_list)

        def print_epipolar_error(color):
            if avg_epipolar_error_lr is not None:
                text = "Avg Epipolar error L-R: " + \
                    format(avg_epipolar_error_lr, '.6f')
                pygame_render_text(self.screen, text, (400, 180), color, 30)

            if avg_epipolar_error_r_rgb is not None:
                text = "Avg Epipolar error RGB-R: " + \
                    format(avg_epipolar_error_r_rgb, '.6f')
                pygame_render_text(self.screen, text, (400, 210), color, 30)

            if rgb_reproject_error is not None:
                text = "RGB Reprojection Error: " + \
                    format(rgb_reproject_error, '.6f')
                pygame_render_text(self.screen, text, (400, 240), color, 30)

        if avg_epipolar_error_lr is not None and avg_epipolar_error_lr > 0.5:
            text = "Failed due to high calibration error L-R"
            pygame_render_text(self.screen, text, (400, 270), red, 30)
            print_epipolar_error(red)
            self.close_device()
            return (False, text)

        if avg_epipolar_error_r_rgb is not None and avg_epipolar_error_r_rgb > 0.7:
            text = "Failed due to high calibration error RGB-R"
            pygame_render_text(self.screen, text, (400, 300), red, 30)
            print_epipolar_error(red)
            self.close_device()
            return (False, text)

        if rgb_reproject_error is not None and rgb_reproject_error > 0.5:
            text = "Failed due to high Reprojection Error"
            pygame_render_text(self.screen, text, (400, 330), red, 30)
            print_epipolar_error(red)
            self.close_device()
            return (False, text)

        calibration_handler = dai.CalibrationHandler()
        calibration_handler.setBoardInfo(self.board_config['board_config']['name'], self.board_config['board_config']['revision'])

        left  = dai.CameraBoardSocket.LEFT
        right = dai.CameraBoardSocket.RIGHT
        if self.args['swapLR']:
            left  = dai.CameraBoardSocket.RIGHT
            right = dai.CameraBoardSocket.LEFT

        if not self.args['disableLR']:
            calibration_handler.setStereoLeft(left, calib_data[0])
            calibration_handler.setStereoRight(right, calib_data[1])

            calibration_handler.setDistortionCoefficients(left, calib_data[9])
            calibration_handler.setDistortionCoefficients(right, calib_data[10])
            calibration_handler.setCameraIntrinsics(left, calib_data[2], 1280, 800)
            calibration_handler.setCameraIntrinsics(right, calib_data[3], 1280, 800)
            calibration_handler.setFov(left, self.board_config['board_config']['left_fov_deg'])
            calibration_handler.setFov(right, self.board_config['board_config']['left_fov_deg'])

            measuredTranslation = [-self.board_config['board_config']['left_to_right_distance_cm'], 0.0, 0.0]
            calibration_handler.setCameraExtrinsics(left, right, calib_data[5], calib_data[6], measuredTranslation)

        if not self.args['disableRgb']:
            calibration_handler.setDistortionCoefficients(dai.CameraBoardSocket.RGB, calib_data[11])
            calibration_handler.setCameraIntrinsics(dai.CameraBoardSocket.RGB, calib_data[4], 1920, 1080)
            calibration_handler.setFov(dai.CameraBoardSocket.RGB, self.board_config['board_config']['rgb_fov_deg'])
            calibration_handler.setLensPosition(dai.CameraBoardSocket.RGB, self.focus_value)

            if not self.args['disableLR']:
                measuredTranslation = [self.board_config['board_config']['left_to_rgb_distance_cm'], 0.0, 0.0]
                calibration_handler.setCameraExtrinsics(right, dai.CameraBoardSocket.RGB, calib_data[7], calib_data[8], measuredTranslation)

        calibration_handler.eepromToJsonFile(calib_dest_path)
        try:
            is_write_succesful = self.device.flashCalibration(calibration_handler)
        except:
            print("Writing in except...")
            is_write_succesful = self.device.flashCalibration(calibration_handler)

        print("Finished writing to EEPROM")
        
        self.is_service_active = False
        self.close_device()
        if is_write_succesful:
            print_epipolar_error(green)
            text = "EEPROM written succesfully"
            pygame_render_text(self.screen, text, (400, 270), green, 30)
            return (True, "EEPROM written succesfully")
        else:
            print_epipolar_error(red)
            text = "EEPROM write Failed!!"
            pygame_render_text(self.screen, text, (400, 270), red, 30)
            return (False, "EEPROM write Failed!!")


no_button = pygame.Rect(490, 500, 80, 45)

if __name__ == "__main__":

    rospy.init_node('depthai_calibration', anonymous=True)
    arg = {}
    arg["swapLR"] = rospy.get_param('~swap_lr')
    arg["disableRgb"] = rospy.get_param('~disableRgb')
    arg["disableLR"] = rospy.get_param('~disableLR')
    arg["usbMode"] = rospy.get_param('~usbMode')

    arg["package_path"] = rospy.get_param('~package_path')

    arg["square_size_cm"] = rospy.get_param('~square_size_cm')
    arg["marker_size_cm"] = rospy.get_param('~marker_size_cm')
    arg["squares_x"] = rospy.get_param('~squares_x')
    arg["squares_y"] = rospy.get_param('~squares_y')

    arg["board"] = rospy.get_param('~brd')
    arg["depthai_path"] = rospy.get_param(
        '~depthai_path')  # Path of depthai repo
    # local path to store calib files with using mx device id.
    arg["calib_path"] = str(Path.home()) + rospy.get_param('~calib_path')
    arg["log_path"] = str(Path.home()) + rospy.get_param("~log_path")
    arg["ds_backup_path"] = str(Path.home()) + '/Desktop/ds_backup'

    # Adding service names to arg
    arg["capture_service_name"] = rospy.get_param(
        '~capture_service_name')  # Add  capture_checkerboard to launch file
    arg["calibration_service_name"] = rospy.get_param(
        '~calibration_service_name')  # Add capture_checkerboard to launch file

    if not os.path.exists(arg['calib_path']):
        os.makedirs(arg['calib_path'])

    if not os.path.exists(arg['log_path']):
        os.makedirs(arg['log_path'])

    if arg['board']:
        board_path = Path(arg['board'])
        if not board_path.exists():
            board_path = Path(consts.resource_paths.boards_dir_path) / \
                Path(arg['board'].upper()).with_suffix('.json')
            print(board_path)
            if not board_path.exists():
                raise ValueError(
                    'Board config not found: {}'.format(board_path))
        with open(board_path) as fp:
            board_config = json.load(fp)
    # assert os.path.exists(arg['depthai_path']), (arg['depthai_path'] +" Doesn't exist. \
    # Please add the correct path using depthai_path:=[path] while executing launchfile")

    depthai_dev = depthai_calibration_node(arg)
    depthai_dev.publisher()
    rospy.spin()
