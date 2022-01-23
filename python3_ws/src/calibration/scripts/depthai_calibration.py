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

stringToCam = {
                'RGB'   : dai.CameraBoardSocket.CAM_A,
                'LEFT'  : dai.CameraBoardSocket.CAM_B,
                'RIGHT' : dai.CameraBoardSocket.CAM_C,
                'CAM_A' : dai.CameraBoardSocket.CAM_A,
                'CAM_B' : dai.CameraBoardSocket.CAM_B,
                'CAM_C' : dai.CameraBoardSocket.CAM_C,
                'CAM_D' : dai.CameraBoardSocket.CAM_D,
                'CAM_E' : dai.CameraBoardSocket.CAM_E,
                'CAM_F' : dai.CameraBoardSocket.CAM_F,
                'CAM_G' : dai.CameraBoardSocket.CAM_G,
                'CAM_H' : dai.CameraBoardSocket.CAM_H
                }

CamToString = {
                dai.CameraBoardSocket.CAM_A : 'RGB'  ,
                dai.CameraBoardSocket.CAM_B : 'LEFT' ,
                dai.CameraBoardSocket.CAM_C : 'RIGHT',
                dai.CameraBoardSocket.CAM_A : 'CAM_A',
                dai.CameraBoardSocket.CAM_B : 'CAM_B',
                dai.CameraBoardSocket.CAM_C : 'CAM_C',
                dai.CameraBoardSocket.CAM_D : 'CAM_D',
                dai.CameraBoardSocket.CAM_E : 'CAM_E',
                dai.CameraBoardSocket.CAM_F : 'CAM_F',
                dai.CameraBoardSocket.CAM_G : 'CAM_G',
                dai.CameraBoardSocket.CAM_H : 'CAM_H'
                }

camToMonoRes = {
                'OV7251' : dai.MonoCameraProperties.SensorResolution.THE_480_P,
                'OV9*82' : dai.MonoCameraProperties.SensorResolution.THE_800_P
                }

camToRgbRes = {
                'IMX378' : dai.ColorCameraProperties.SensorResolution.THE_4_K,
                'IMX214' : dai.ColorCameraProperties.SensorResolution.THE_4_K,
                'OV9*82' : dai.ColorCameraProperties.SensorResolution.THE_800_P
                }


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

        # self.focus_value = 0
        # self.defaultLensPosition = 135
        self.focusSigmaThreshold = 25
        # if self.rgbCcm == 'Sunny':
        #     self.focus_value = 135
        # elif self.rgbCcm == 'KingTop':
        #     self.focus_value = 135
        # elif self.rgbCcm == 'ArduCam':
        #     self.focus_value = 135

        # self.frame_count = 0
        self.init_time = time.time()
        if self.args['board']:
            board_path = Path(self.args['board'])
            if not board_path.exists():
                board_path = Path(consts.resource_paths.boards_dir_path) / \
                    Path(self.args['board'].upper()).with_suffix('.json')
                if not board_path.exists():
                    raise ValueError(
                        'Board config not found: {}'.format(board_path))
            with open(board_path) as fp:
                self.board_config = json.load(fp)
                self.board_config = self.board_config['board_config']
                self.board_config_backup = self.board_config


        self.aruco_dictionary = cv2.aruco.Dictionary_get(
            cv2.aruco.DICT_4X4_1000)
        
        self.ccm_selector()

        # Connection checks ----------->
        title = "Device Status"
        pygame_render_text(self.screen, title, (350, 20), orange, 50)
        self.auto_checkbox_names = []
        self.auto_focus_checkbox_names = []

        if self.args['usbMode']:
            self.auto_checkbox_names.append("USB3")
        header = ['time', 'Mx_serial_id']
        for cam_id in self.board_config['cameras'].keys():
            cam_info = self.board_config['cameras'][cam_id]
            header.append(cam_info['name'] + '-CCM')
            # header.append(cam_info['name'] + '-camera')
            header.append(cam_info['name'] + '-focus-stdDev')
            header.append(cam_info['name'] + '-Reprojection-Error')
            self.auto_checkbox_names.append(cam_info['name']  + '-Camera-connected')
            self.auto_checkbox_names.append(cam_info['name']  + '-Stream')
            self.auto_focus_checkbox_names.append(cam_info['name']  + '-Focus')
            if 'extrinsics' in cam_info:
                if 'to_cam' in cam_info['extrinsics']:
                    right_cam = self.board_config['cameras'][cam_info['extrinsics']['to_cam']]['name']    
                    header.append('Epipolar-error-' + cam_info['name'] + '-' + right_cam)
            
        # ['Mono-CCM', 'RGB-CCM',
        #           'left_camera', 'right_camera', 'rgb_camera', 
        #           'left_focus_stdDev', 'right_focus_stdDev', 'rgb_focus_stdDev',
        #           'Epipolar error L-R', 'Epipolar error R-Rgb', 'RGB Reprojection Error']
        
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

        y = y + (40*len(self.auto_checkbox_names))
        self.auto_focus_checkbox_dict = {}
        
        for i in range(len(self.auto_focus_checkbox_names)):
            w, h = font.size(self.auto_focus_checkbox_names[i])
            x_axis = x - w
            y_axis = y + (40*i)
            font_surf = font.render(self.auto_focus_checkbox_names[i], True, green)
            self.screen.blit(font_surf, (x_axis, y_axis))
            self.auto_focus_checkbox_dict[self.auto_focus_checkbox_names[i]] = Checkbox(self.screen, x + 10, y_axis-5, outline_color=green,
                                                                            check_color=green, check=False)

        for i in range(len(self.auto_focus_checkbox_names)):
            self.auto_focus_checkbox_dict[self.auto_focus_checkbox_names[i]].render_checkbox()

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
            "rgb_focus_adjuster", Capture, self.camera_focus_adjuster)
        # self.rgb_focus_srv = rospy.Service(
        #     "set_rgb_focus", Capture, self.rgb_focus_handler)

        self.args['cameraModel'] = 'perspective'
        self.imgPublishers = dict()
        for cam_id in self.board_config['cameras']:
            name = self.board_config['cameras'][cam_id]['name']
            self.imgPublishers[name] = rospy.Publisher(name, Image, queue_size=10)

        self.device = None

    def ccm_selector(self):
        title = "Select the mono Camera and RGB camera vendor"
        pygame_render_text(self.screen, title, (70, 20), black, 40)
        space = "         "
        title = ""
        for camera in self.board_config['cameras'].keys():
            title += self.board_config['cameras'][camera]['name'] + space

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
            offset = 150
            offset_increment = 1
            for camera in self.board_config['cameras'].keys():
                # ccm_names_dict[ccm_names[i]].append()
                ccm_names_dict[ccm_names[i]].append(Checkbox(self.screen, x + (offset * offset_increment), y_axis-5, outline_color=green,
                                                                                check_color=green, check=False, disable_pass = True))
                ccm_names_dict[ccm_names[i]][-1].render_checkbox()
                offset_increment += 1

            # ccm_names_dict[ccm_names[i]].append(Checkbox(self.screen, x + 420, y_axis-5, outline_color=green, 
            #                                                                 check_color=green, check=False, disable_pass = True))
            # ccm_names_dict[ccm_names[i]][0].render_checkbox()
            # ccm_names_dict[ccm_names[i]][1].render_checkbox()
            fill_color = pygame.Rect(20, y_axis + 40, 750, 2)
            pygame.draw.rect(self.screen, black, fill_color)
        
        next_button =  pygame.Rect(600, 430, 60, 35)
        pygame.draw.rect(self.screen, orange, next_button)
        pygame_render_text(self.screen, 'Next', (605, 440))

        is_saved = False

        self.monoCcm = None
        self.rgbCcm = None
        is_ccm_selected = []
        self.ccm_selected = {}

        for camera in self.board_config['cameras'].keys():
            is_ccm_selected.append(False)
            self.ccm_selected[self.board_config['cameras'][camera]['name']] = None
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
                        isAllChecked = True
                        for val in is_ccm_selected:
                            isAllChecked = isAllChecked and val

                        if isAllChecked:
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
                for i in range(len(list(self.board_config['cameras']))):
                    ccm1[i].update_checkbox_rel(event, ccm2[i], ccm3[i])
                    ccm2[i].update_checkbox_rel(event, ccm1[i], ccm3[i])
                    ccm3[i].update_checkbox_rel(event, ccm1[i], ccm2[i]) 
                    ccm1[i].render_checkbox()
                    ccm2[i].render_checkbox()
                    ccm3[i].render_checkbox()
                    if ccm1[i].is_checked() or ccm2[i].is_checked() or ccm3[i].is_checked():
                        is_ccm_selected[i] = True

        camList = list(self.board_config['cameras'])
        print("selected CCMS:")
    
        for i in range(len(ccm_names)):
            for j in range(len(camList)):
                if ccm_names_dict[ccm_names[i]][j].is_checked():
                    self.ccm_selected[self.board_config['cameras'][camList[j]]['name']] = ccm_names[i]
                    print(self.board_config['cameras'][camList[j]]['name'], '=> ', ccm_names[i])
        
        self.screen.fill(white)
        self.disp.update()

    def create_pipeline(self, camProperties):
        pipeline = dai.Pipeline()

        fps = 10
        for cam_id in self.board_config['cameras']:
            cam_info = self.board_config['cameras'][cam_id]
            if cam_info['type'] == 'mono':
                cam_node = pipeline.createMonoCamera()
                xout = pipeline.createXLinkOut()

                cam_node.setBoardSocket(stringToCam[cam_id])
                cam_node.setResolution(camToMonoRes[cam_info['sensorName']])
                cam_node.setFps(fps)

                xout.setStreamName(cam_info['name'])
                cam_node.out.link(xout.input)
            else:
                cam_node = pipeline.createColorCamera()
                xout = pipeline.createXLinkOut()
                
                cam_node.setBoardSocket(stringToCam[cam_id])
                cam_node.setResolution(camToRgbRes[cam_info['sensorName']])
                cam_node.setFps(fps)

                xout.setStreamName(cam_info['name'])
                cam_node.isp.link(xout.input)

                if cam_info['hasAutofocus']:
                    controlIn = pipeline.createXLinkIn()
                    controlIn.setStreamName(cam_info['name'] + '-control')
                    controlIn.out.link(cam_node.inputControl)
            xout.input.setBlocking(False)
            xout.input.setQueueSize(1)

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
                for config_cam in self.board_config['cameras']:
                    cam_info = self.board_config['cameras'][config_cam]
                    frame = self.camera_queue[cam_info['name']].tryGet()
                    if frame is not None:
                        currFrame = None
                        if frame.getType() == dai.RawImgFrame.Type.RAW8:
                            currFrame = frame.getCvFrame()
                        else:
                            currFrame = cv2.cvtColor(frame.getCvFrame(), cv2.COLOR_BGR2GRAY)
                        self.imgPublishers[cam_info['name']].publish(
                                self.bridge.cv2_to_imgmsg(currFrame, "passthrough"))

    def cvt_bgr(self, packet):
        meta = packet.getMetadata()
        w = meta.getFrameWidth()
        h = meta.getFrameHeight()
        packetData = packet.getData()
        yuv420p = packetData.reshape((h * 3 // 2, w))
        return cv2.cvtColor(yuv420p, cv2.COLOR_YUV2BGR_IYUV)

    def parse_frame(self, frame, stream_name, file_name):
        if frame is None:
            print("Frame with stream name -> {} was None".format(stream_name))
            return False

        file_name += '.png'
        # filename = image_filename(stream_name, self.current_polygon, self.images_captured)
        print(self.package_path + "/dataset/{}/{}".format(stream_name, file_name))
        ds_path = self.package_path + "/dataset/{}".format(stream_name)
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
                        for _, checkbox in self.auto_checkbox_dict.items():
                            checkbox.setUnattended()
                            checkbox.render_checkbox()

                        for _, checkbox in self.auto_focus_checkbox_dict.items():
                            checkbox.setUnattended()
                            checkbox.render_checkbox()
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
        # if hasattr(self, 'left_camera_queue') and hasattr(self, 'right_camera_queue'):
        #     delattr(self, 'left_camera_queue')
        #     delattr(self, 'right_camera_queue')
        # if hasattr(self, 'rgb_camera_queue') and hasattr(self, 'rgb_control_queue'):
        #     delattr(self, 'rgb_camera_queue')
        #     delattr(self, 'rgb_control_queue')
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

        fill_color_2 = pygame.Rect(50, 520, 400, 80)
        pygame.draw.rect(self.screen, white, fill_color_2)

        for _, checkbox in self.auto_checkbox_dict.items():
            checkbox.setUnattended()
            checkbox.render_checkbox()

        for _, checkbox in self.auto_focus_checkbox_dict.items():
            checkbox.setUnattended()
            checkbox.render_checkbox()

        if self.device is not None:
            if not self.device.isClosed():
                self.device.close()
        self.board_config = self.board_config_backup
        finished = False
        while not finished:
            if self.capture_exit():
                rospy.signal_shutdown("Stopping calibration")
            while self.device is None or self.device.isClosed():
                if self.capture_exit():
                    rospy.signal_shutdown("Stopping calibration")
                
                searchTime = timedelta(seconds=80)
                isFound, deviceInfo = dai.Device.getAnyAvailableDevice(searchTime)
                if isFound:
                    self.device = dai.Device() 
                    # cameraList = self.device.getConnectedCameras()
                    cameraProperties = self.device.getConnectedCameraProperties()
                    fill_color_2 = pygame.Rect(390, 120, 500, 100)
                    pygame.draw.rect(self.screen, white, fill_color_2)

                    rospy.sleep(2)
                    # dev_info = self.device.getDeviceInfo()
                    text = "device Mx_id : " + self.device.getMxId()
                    pygame_render_text(self.screen, text, (400, 120), black, 30)
                    text = "Device Connected!!!"
                    pygame_render_text(self.screen, text, (400, 150), green, 30)

                    lost_camera = False
                    for properties in cameraProperties:
                        for in_cam in self.board_config['cameras'].keys():
                            cam_info = self.board_config['cameras'][in_cam]
                            if properties.socket == stringToCam[in_cam]:
                                self.board_config['cameras'][in_cam]['sensorName'] = properties.sensorName
                                print('Cam: {} and focus: {}'.format(cam_info['name'], properties.hasAutofocus))
                                self.board_config['cameras'][in_cam]['hasAutofocus'] = properties.hasAutofocus
                                self.auto_checkbox_dict[cam_info['name']  + '-Camera-connected'].check()
                                break

                    for config_cam in self.board_config['cameras'].keys():
                        cam_info = self.board_config['cameras'][config_cam]
                        if self.auto_checkbox_dict[cam_info['name']  + '-Camera-connected'].isUnattended():
                            self.auto_checkbox_dict[cam_info['name']  + '-Camera-connected'].uncheck()
                            lost_camera = True
                        self.auto_checkbox_dict[cam_info['name']  + '-Camera-connected'].render_checkbox()

                    if self.args['usbMode']:
                        if self.device.getUsbSpeed() == dai.UsbSpeed.SUPER:
                            self.auto_checkbox_dict["USB3"].check()
                        else:
                            lost_camera = True
                            self.auto_checkbox_dict["USB3"].uncheck()
                        self.auto_checkbox_dict["USB3"].render_checkbox()

                    if not lost_camera:
                        pipeline = self.create_pipeline(cameraProperties)
                        self.device.startPipeline(pipeline)
                        self.camera_queue = {}
                        self.control_queue = {}
                        for config_cam in self.board_config['cameras']:
                            cam = self.board_config['cameras'][config_cam]
                            self.camera_queue[cam['name']] = self.device.getOutputQueue(cam['name'], 1, False)
                            if cam['hasAutofocus']:
                                self.control_queue[cam['name']] = self.device.getInputQueue(cam['name'] + '-control', 5, False)
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

            mipi = {}
            for config_cam in self.board_config['cameras']:
                mipi[self.board_config['cameras'][config_cam]['name']] = False

            # left_mipi = False
            # right_mipi = False
            # rgb_mipi = False

            for _ in range(120):
                for config_cam in self.board_config['cameras']:
                    name = self.board_config['cameras'][config_cam]['name']
                    imageFrame = self.camera_queue[name].tryGet()
                   
                    if imageFrame is not None:
                        mipi[name] = True
                        frame = None

                        if imageFrame.getType() == dai.RawImgFrame.Type.RAW8:
                            frame = imageFrame.getCvFrame()
                        else:
                            frame = cv2.cvtColor(imageFrame.getCvFrame(), cv2.COLOR_BGR2GRAY)
                        self.imgPublishers[name].publish(
                            self.bridge.cv2_to_imgmsg(frame, "passthrough"))


                """ if not self.args['disableLR']:
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
                    rgb_mipi = True """

                isMipiReady = True
                for config_cam in self.board_config['cameras']:
                    name = self.board_config['cameras'][config_cam]['name']
                    isMipiReady = isMipiReady and mipi[name] 
                if isMipiReady:
                    break
                rospy.sleep(1)
            
            for key in mipi.keys():
                if not mipi[key]:
                    self.auto_checkbox_dict[key + "-Stream"].uncheck()
                else:
                    self.auto_checkbox_dict[key + "-Stream"].check()

            for i in range(len(self.auto_checkbox_names)):
                self.auto_checkbox_dict[self.auto_checkbox_names[i]].render_checkbox()

            isAllPassed = True
            for key in self.auto_checkbox_dict.keys():
                #FIXME(sachin): is_checked is a function not a variable
                isAllPassed = isAllPassed and self.auto_checkbox_dict[key].is_checked()

            if isAllPassed:
                finished = True
            else:
                self.close_device()
                self.retest()

        dataset_path = Path(self.package_path + "/dataset")
        if 0 and dataset_path.exists():
            shutil.rmtree(str(dataset_path))
        # dev_info = self.device.getDeviceInfo()
        self.is_service_active = False
        return (finished, self.device.getMxId())

    def camera_focus_adjuster(self, req):
        self.is_service_active = True
        maxCountFocus   = 50
        focusCount = {}
        isFocused = {}
        trigCount = {}
        capturedFrames = {}
        
        self.lensPosition = {}
        self.focusSigma = {}
        
        ctrl = dai.CameraControl()
        ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
        ctrl.setAutoFocusTrigger()

        for config_cam in self.board_config['cameras'].keys():
            cam_info = self.board_config['cameras'][config_cam]
            focusCount[cam_info['name']] = 0
            self.focusSigma[cam_info['name']] = 0
            self.lensPosition[cam_info['name']] = 0
            isFocused[config_cam] = False
            capturedFrames[cam_info['name']] = None

            if cam_info['hasAutofocus']:
                trigCount[cam_info['name']] = 0
                self.control_queue[cam_info['name']].send(ctrl)

        rospy.sleep(1)
        focusFailed  = False
        while True:
            for config_cam in self.board_config['cameras'].keys():
                cam_info = self.board_config['cameras'][config_cam]
                frame = self.camera_queue[cam_info['name']].getAll()[-1]
                currFrame = frame.getCvFrame()
                if frame.getType() != dai.RawImgFrame.Type.RAW8:
                    currFrame = cv2.cvtColor(currFrame, cv2.COLOR_BGR2GRAY)
                print('Resolution: {}'.format(currFrame.shape))
                capturedFrames[cam_info['name']] = currFrame 
                self.imgPublishers[cam_info['name']].publish(
                            self.bridge.cv2_to_imgmsg(currFrame, "passthrough"))

                if cam_info['hasAutofocus']:
                    marker_corners, _, _ = cv2.aruco.detectMarkers(currFrame, self.aruco_dictionary)
                    if len(marker_corners) < 30:
                        print("Board not detected. Waiting...!!!")
                        trigCount[cam_info['name']] += 1
                        focusCount[cam_info['name']] += 1
                        if trigCount[cam_info['name']] > 31:
                            trigCount[cam_info['name']] = 0
                            self.control_queue[cam_info['name']].send(ctrl)
                            time.sleep(1)
                        if focusCount[cam_info['name']] > maxCountFocus:
                            focusFailed = True
                            break
                        continue

                dst_laplace = cv2.Laplacian(currFrame, cv2.CV_64F)
                mu, sigma = cv2.meanStdDev(dst_laplace)

                print('Sigma of {} is {}'.format(cam_info['name'], sigma))
                localFocusThreshold = self.focusSigmaThreshold 
                if dst_laplace.shape[1] > 2000:
                    localFocusThreshold = localFocusThreshold / 2

                if sigma > localFocusThreshold:
                    isFocused[config_cam] = True
                    self.focusSigma[cam_info['name']] = sigma
                    if cam_info['hasAutofocus']:
                        self.lensPosition[cam_info['name']] = frame.getLensPosition()
                else:
                    if cam_info['hasAutofocus']:
                        trigCount[cam_info['name']] += 1
                        if trigCount[cam_info['name']] > 31:
                            trigCount[cam_info['name']] = 0
                            self.control_queue[cam_info['name']].send(ctrl)
                            time.sleep(1)
                focusCount[cam_info['name']] += 1

            if focusFailed:
                break
            
            isCountFull = True 
            for key in focusCount.keys():
                if focusCount[key] < maxCountFocus:
                    isCountFull = False
                    break
            if isCountFull:
                break
        
        backupFocusPath = self.args['ds_backup_path'] + '/focus/' + self.device.getMxId()
        if not os.path.exists(backupFocusPath):
            os.makedirs(backupFocusPath)

        for name, image in capturedFrames.items():
            print('Backing up images {}'.format(name))
            cv2.imwrite(backupFocusPath + "/" + name + '.png', image)

        for key in isFocused.keys():
            cam_name = self.board_config['cameras'][key]['name']
            if isFocused[key]:
                if self.board_config['cameras'][key]['hasAutofocus']:
                    ctrl = dai.CameraControl()
                    ctrl.setManualFocus(self.lensPosition[cam_name])
                    print("Sending manual focus Control to {}".format(cam_name))
                    self.control_queue[cam_name].send(ctrl)
                self.auto_focus_checkbox_dict[cam_name + "-Focus"].check()
                self.auto_focus_checkbox_dict[cam_name + "-Focus"].render_checkbox()
            else:
                self.auto_focus_checkbox_dict[cam_name + "-Focus"].uncheck()
                self.auto_focus_checkbox_dict[cam_name + "-Focus"].render_checkbox()

        for key in self.auto_focus_checkbox_dict.keys():
            if not self.auto_focus_checkbox_dict[key].is_checked():
                self.close_device()
                self.is_service_active = False
                return (False, key + " is out of Focus")
            # else:
            #     print(key + " is in Focus")
        
        self.is_service_active = False        
        return (True, "RGB in Focus")


    def capture_servive_handler(self, req):
        print("Capture image Service Started")
        self.is_service_active = True
        wait_time = 1
        if "POE" in self.board_config['name']: 
            wait_time = 2
        rospy.sleep(wait_time)

        # TODO(Sachin): Add time synchronization here and get the most recent frame instead.
        frameCount = 0
        detection_failed = False
        # while not finished:

        for key in self.camera_queue.keys():
            frame = self.camera_queue[key].getAll()[-1]
            gray_frame = None
            if frame.getType() == dai.RawImgFrame.Type.RAW8:
                gray_frame = frame.getCvFrame()
            else:
                gray_frame = cv2.cvtColor(frame.getCvFrame(), cv2.COLOR_BGR2GRAY)
            self.imgPublishers[key].publish(
                        self.bridge.cv2_to_imgmsg(gray_frame, "passthrough"))
            
            is_board_found = self.is_markers_found(gray_frame)
            if is_board_found:
                self.parse_frame(gray_frame, key, req.name)
            else:
                self.parse_frame(gray_frame, key + '_not', req.name)
                detection_failed = True
        #TODO(sachin): Do I need to cross check lens position of autofocus camera's ?

        if detection_failed:
            self.device.close()
            self.is_service_active = False
            return (False, "Calibration board not found")
        else:
            self.is_service_active = False
            return (True, "No Error")

    def calibration_servive_handler(self, req):
        self.is_service_active = True
        print("calibration Service Started")
        # pygame.draw.rect(self.screen, white, no_button)

        mx_serial_id = self.device.getMxId()
        # dev_info = self.device.getDeviceInfo()
        # mx_serial_id = dev_info.getMxId()
        calib_dest_path = os.path.join(
            self.args['calib_path'], self.args["board"] + '_' + mx_serial_id + '.json')
        # print(self.package_path)
        stereo_calib = StereoCalibration()
        status, result_config = stereo_calib.calibrate(
                                        self.board_config,
                                        self.package_path + "/dataset",
                                        self.args['square_size_cm'],
                                        self.args['marker_size_cm'],
                                        self.args['squares_x'],
                                        self.args['squares_y'],
                                        self.args['cameraModel'],
                                        False) # Turn off enable disp rectify
         
        start_time = datetime.now()
        time_stmp = start_time.strftime("%m-%d-%Y %H:%M:%S")

        log_list = [time_stmp, mx_serial_id]
        # for key in self.ccm_selected.keys():
        #     log_list.append(self.ccm_selected[key])

        if status == -1:
            self.close_device()
            self.is_service_active = False
            return result_config

        vis_x = 400
        vis_y = 180
        error_text = []
        calibration_handler = dai.CalibrationHandler()
        for camera in result_config['cameras'].keys():
            cam_info = result_config['cameras'][camera]
            log_list.append(self.ccm_selected[cam_info['name']])

            color = green
            reprojection_error_threshold = 0.7
            if cam_info['size'][1] > 720:
                print(cam_info['size'][1])
                reprojection_error_threshold = reprojection_error_threshold * cam_info['size'][1] / 720

            print('Reprojection error threshold -> {}'.format(reprojection_error_threshold))
            if cam_info['reprojection_error'] > reprojection_error_threshold:
                color = red
                error_text.append("high Reprojection Error")
            text = cam_info['name'] + ' Reprojection Error: ' + format(cam_info['reprojection_error'], '.6f')
            pygame_render_text(self.screen, text, (vis_x, vis_y), color, 30)
            
            calibration_handler.setDistortionCoefficients(stringToCam[camera], cam_info['dist_coeff'])
            calibration_handler.setCameraIntrinsics(stringToCam[camera], cam_info['intrinsics'],  cam_info['size'][0], cam_info['size'][1])
            calibration_handler.setFov(stringToCam[camera], cam_info['hfov'])

            if cam_info['hasAutofocus']:
                calibration_handler.setLensPosition(stringToCam[camera], self.lensPosition[cam_info['name']])
            
            log_list.append(self.focusSigma[cam_info['name']])
            log_list.append(cam_info['reprojection_error'])
            
            vis_y += 30
            color = green
            
            if 'extrinsics' in cam_info:
                
                if 'to_cam' in cam_info['extrinsics']:
                    right_cam = result_config['cameras'][cam_info['extrinsics']['to_cam']]['name']
                    left_cam = cam_info['name']
                    
                    epipolar_threshold = 0.6
                    #TODO(sachin):  Remove this higher threshold and reduce the speed of the arm robot to avoid wobbling
                    if "POE" in self.board_config['name']: 
                        epipolar_threshold = 0.8

                    if cam_info['extrinsics']['epipolar_error'] > epipolar_threshold:
                        color = red
                        error_text.append("high epipolar error between " + left_cam + " and " + right_cam)
                    elif cam_info['extrinsics']['epipolar_error'] == -1:
                        color = red
                        error_text.append("Epiploar validation failed between " + left_cam + " and " + right_cam)
                   
                    log_list.append(cam_info['extrinsics']['epipolar_error'])
                    text = left_cam + "-" + right_cam + ' Avg Epipolar error: ' + format(cam_info['extrinsics']['epipolar_error'], '.6f')
                    pygame_render_text(self.screen, text, (vis_x, vis_y), color, 30)
                    vis_y += 30
                    specTranslation = np.array([cam_info['extrinsics']['specTranslation']['x'], cam_info['extrinsics']['specTranslation']['y'], cam_info['extrinsics']['specTranslation']['z']], dtype=np.float32)

                    calibration_handler.setCameraExtrinsics(stringToCam[camera], stringToCam[cam_info['extrinsics']['to_cam']], cam_info['extrinsics']['rotation_matrix'], cam_info['extrinsics']['translation'], specTranslation)
                    if result_config['stereo_config']['left_cam'] == camera and result_config['stereo_config']['right_cam'] == cam_info['extrinsics']['to_cam']:
                        calibration_handler.setStereoLeft(stringToCam[camera], result_config['stereo_config']['rectification_left'])
                        calibration_handler.setStereoRight(stringToCam[cam_info['extrinsics']['to_cam']], result_config['stereo_config']['rectification_right'])
            # else:
                # log_list.append("N/A")
        
        log_file = self.args['log_path'] + "/calibration_logs_" + self.args['board'] + ".csv"
        with open(log_file, mode='a') as log_fopen:
            # header =
            log_csv_writer = csv.writer(log_fopen, delimiter=',')
            log_csv_writer.writerow(log_list)
        calibration_handler.setBoardInfo(self.board_config['name'], self.board_config['revision'])
        
        if len(error_text) == 0:
            print('Flashing Calibration data into ')
            print(calib_dest_path)
            calibration_handler.eepromToJsonFile(calib_dest_path)
            try:
                is_write_succesful = self.device.flashCalibration(calibration_handler)
            except:
                print("Writing in except...")
                is_write_succesful = self.device.flashCalibration(calibration_handler)
            self.close_device()
            self.is_service_active = False
            if is_write_succesful:
                text = "EEPROM written succesfully"
                pygame_render_text(self.screen, text, (vis_x, vis_y), green, 30)
                return (True, "EEPROM written succesfully")
            else:
                text = "EEPROM write Failed!!"
                pygame_render_text(self.screen, text, (vis_x, vis_y), red, 30)
                return (False, "EEPROM write Failed!!")
        else:
            text = error_text[0]
            pygame_render_text(self.screen, text, (vis_x, vis_y), red, 30)
            print(error_text)

            self.close_device()
            self.is_service_active = False
            return (False, error_text[0])


no_button = pygame.Rect(490, 500, 80, 45)

if __name__ == "__main__":

    rospy.init_node('depthai_calibration', anonymous=True)
    arg = {}
    arg["swapLR"] = rospy.get_param('~swap_lr')
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
