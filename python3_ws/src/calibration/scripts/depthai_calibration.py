#!/usr/bin/env python

import cv2
import sys
import copy
import rospy
from std_msgs.msg import String
import depthai
import platform
import signal
import subprocess
import json
import datetime
from collections import deque  

from calibration.srv import Capture
import time
import numpy as np
import os
from pathlib import Path
import shutil
import consts.resource_paths
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import shutil
from depthai_helpers.calibration_utils import *

from depthai_helpers.pygame_checkbox import Checkbox, pygame_render_text
import pygame
from pygame.locals import *

from depthai_helpers import utils
os.environ['SDL_VIDEO_WINDOW_POS'] = '100,50'

on_embedded = platform.machine().startswith(
    'arm') or platform.machine().startswith('aarch64')


def find_chessboard(frame):
    chessboard_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + \
        cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
    small_frame = cv2.resize(frame, (0, 0), fx=0.3, fy=0.3)
    return cv2.findChessboardCorners(small_frame, (9, 6), chessboard_flags)[0] and \
        cv2.findChessboardCorners(frame, (9, 6), chessboard_flags)[0]


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
        self.config = {
            'streams':
                ['left', 'right', 'meta_d2h', 'color'] if not on_embedded else
                [{'name': 'left', "max_fps": 30.0}, {'name': 'right', "max_fps": 30.0}, {
                    'name': 'meta_d2h', "max_fps": 30.0}, {'name': 'color', "max_fps": 30.0}],
            'depth':
                {
                    'calibration_file': consts.resource_paths.calib_fpath,
                    'padding_factor': 0.3
                },
            'ai':
                {
                    'blob_file': consts.resource_paths.blob_fpath,
                    'blob_file_config': consts.resource_paths.blob_config_fpath,
                    'shaves': 7,
                    'cmx_slices': 7,
                    'NN_engines': 1,
                },
            'board_config':
                {
                    'swap_left_and_right_cameras': self.args['swap_lr'],
                    'left_fov_deg':  self.args['field_of_view'],
                    'left_to_right_distance_cm': self.args['baseline'],
                    'override_eeprom': False,
                    'stereo_center_crop': True,
                },
            'camera':
                {
                    'rgb':
                    {
                        'resolution_h': 1080,
                        'fps': 30.0,
                        'initial_focus': 141,
                        'enable_autofocus': False
                    },
                    'mono':
                    {
                        # 1280x720, 1280x800, 640x400 (binning enabled)
                        'resolution_h': 720,
                        'fps': 30.0,
                    },
                },
            'app':
                {
                    'enable_imu': True
                },
        }
        self.frame_count = 0
        self.focus_value = 141
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
                self.board_config = json.load(fp)
        utils.merge(self.board_config, self.config)

        self.aruco_dictionary = cv2.aruco.Dictionary_get(
            cv2.aruco.DICT_4X4_1000)
        # self.charuco_board = aruco.CharucoBoard_create(
        #                                     22,16,
        #                                     self.args['square_size_cm'],
        #                                     self.args['marker_size_cm'],
        #                                     self.aruco_dictionary)
        self.start_device()

        self.disp = pygame.display
        self.screen = self.disp.set_mode((800, 600))
        self.screen.fill(white)
        self.disp.set_caption("Calibration - Device check ")
        title = "Device Status"
        pygame_render_text(self.screen, title, (350, 20), orange, 50)
        self.auto_checkbox_names = ["USB3", "Left camera connected", "Right camera connected",
                                    "Left Stream", "Right Stream"]
        if self.args['enable_IMU_test']:
            self.auto_checkbox_names.append("IMU connected")

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
            self.auto_checkbox_dict[self.auto_checkbox_names[i]
                                    ].render_checkbox()
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
        self.rgb_focus_srv = rospy.Service(
            "set_rgb_focus", Capture, self.rgb_focus_handler)

        self.image_pub_left = rospy.Publisher("left", Image, queue_size=10)
        self.image_pub_right = rospy.Publisher("right", Image, queue_size=10)
        self.image_pub_color = rospy.Publisher("color", Image, queue_size=10)

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
                    # is_device_ready = False
                    # if arm_control is not None:
                    # arm_control.go_to_next_joint_state(joints_goal_dict['sleep'])
                    break
        # pygame.event.pump()
        return is_clicked

    def start_device(self):
        self.device = depthai.Device('', False)
        # self.device = depthai.Device('/home/nuc/Desktop/depthai/.fw_cache/depthai-6fc8c54e33b8aa6d16bf70ac5193d10090dcd0d8.cmd', '')
        self.pipeline = self.device.create_pipeline(self.config)
        self.mx_id = self.device.get_mx_id()
        # rospy.sleep(2)
        # self.device.request_af_mode(depthai.AutofocusMode.AF_MODE_EDOF)
        # self.device.request_af_mode(depthai.AutofocusMode.AF_MODE_AUTO)
        # setting manual focus to rgb camera
        # self.set_focus()
        self.set_focus()

    def set_focus(self):
        if 1:
            print("RGB set_focus() disabled. TODO")
            return
        cam_c = depthai.CameraControl.CamId.RGB
        # Disabling AF mode
        print('Disabling AF mode')
        # self.device.send_camera_control(
        #     cam_c, depthai.CameraControl.Command.AF_MODE, '0')
        cmd_set_focus = depthai.CameraControl.Command.MOVE_LENS
        # Disabling AF mode
        print('setting Focus')
        print("Setting focus value to {}".format(self.focus_value))
        self.device.send_camera_control(
            cam_c, cmd_set_focus, str(self.focus_value))

    def rgb_focus_handler(self, req):
        is_num = req.name.isnumeric()
        if is_num:
            rgb_focus_val = int(req.name)
            if rgb_focus_val >= 0 and rgb_focus_val <= 255:
                # self.device.request_af_mode(depthai.AutofocusMode.AF_MODE_AUTO)
                cam_c = depthai.CameraControl.CamId.RGB
                # self.device.send_camera_control(
                #     cam_c, depthai.CameraControl.Command.AF_MODE, '0')
                cmd_set_focus = depthai.CameraControl.Command.MOVE_LENS
                self.device.send_camera_control(cam_c, cmd_set_focus, req.name)
                return True, 'Focus changed to ' + req.name
            # else :
                # return False, 'Invalid focus input.!! Focus number should be between 0-255'
        # else:
        return False, 'Invalid focus input.!! Focus number should be between 0-255'

    def publisher(self):
        while not rospy.is_shutdown():
            if self.capture_exit():
                print("signaling...")
                rospy.signal_shutdown("Finished calibration")
            if self.start_disp:
                self.disp.update()
            # else:
            #     pygameX.event.pump()
            # print("updating dis-----")
            if not self.is_service_active:
                # pygame.draw.rect(self.screen, red, no_button)
                # pygame_render_text(self.screen, 'Exit', (500, 505))
                # print("SERVICE NOT ACTIVE")
                if not hasattr(self, "pipeline"):
                    self.start_device()
                    # print("restarting device---->")
                _, data_list = self.pipeline.get_available_nnet_and_data_packets(
                    True)
                for packet in data_list:
                    # print("found packets:")
                    # print(packet.stream_name)
                    if packet.stream_name == "left":
                        recent_left = packet.getData()
                        # print(recent_left.shape)
                        self.image_pub_left.publish(
                            self.bridge.cv2_to_imgmsg(recent_left, "passthrough"))
                    elif packet.stream_name == "right":
                        recent_right = packet.getData()
                        # print(recent_right.shape)
                        self.image_pub_right.publish(
                            self.bridge.cv2_to_imgmsg(recent_right, "passthrough"))
                    elif packet.stream_name == "color":
                        # since getting python3 working on ros melodic is an
                        # hack cannot publish rgb. it will throw an error.
                        self.frame_count += 1
                        recent_color = cv2.cvtColor(
                            self.cvt_bgr(packet), cv2.COLOR_BGR2GRAY)
                        self.image_pub_color.publish(
                            self.bridge.cv2_to_imgmsg(recent_color, "passthrough"))
                        meta = packet.getMetadata()
                        # print(
                        #     ' Frame seq number <-< {} '.format(meta.getSequenceNum()))
                        # print(' Frame TS number <-< {} '.format(meta.getTimestamp()))

                        # print('Local frame rate: {}'.format(self.frame_count))
                    elif packet.stream_name == "meta_d2h":
                        str_ = packet.getDataAsStr()
                        dict_ = json.loads(str_)
                        # print('last frame tstamp: {:.6f}'.format(
                        #     dict_['camera']['last_frame_timestamp']))
                        print('meta_d2h frame focus ----------?: {}'.format(
                            dict_['camera']['rgb']['focus_pos']))
                        # print(
                        #     'Metad2h frame cpunt <-< {}'.format(dict_['camera']['rgb']['frame_count']))

    def cvt_bgr(self, packet):
        meta = packet.getMetadata()
        w = meta.getFrameWidth()
        h = meta.getFrameHeight()
        # print((h, w))
        packetData = packet.getData()
        yuv420p = packetData.reshape((h * 3 // 2, w))
        return cv2.cvtColor(yuv420p, cv2.COLOR_YUV2BGR_IYUV)

    def parse_frame(self, frame, stream_name, file_name):
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
        return True

    def is_markers_found(self, frame):
        # print(frame.shape)
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, _, _ = cv2.aruco.detectMarkers(
            frame, self.aruco_dictionary)
        return not (len(marker_corners) == 0)

    def capture_servive_handler(self, req):
        print("Capture image Service Started")
        recent_left = None
        recent_right = None
        recent_color = None
        finished = False
        self.is_service_active = True
        # now = rospy.get_rostime()
        # pygame.draw.rect(self.screen, white, no_button)
        rospy.sleep(1)
        # ts_color = None
        # ts_color_dev = None
        # current_focus = None
        # current_color_pkt = None
        rgb_check_count = 0
        m_d2h_seq_focus = dict()
        # color_pkt_queue = deque()  
        while not finished:
            _, data_list = self.pipeline.get_available_nnet_and_data_packets(
                True)
            # print(len(data_list))

            for packet in data_list:
                # print(packet.stream_name)
                # print("packet time")
                # print(packet.getMetadata().getTimestamp())
                # print("ros time")
                # print(now.secs)
                if packet.stream_name == "left":
                    recent_left = packet.getData()
                elif packet.stream_name == "right":
                    recent_right = packet.getData()
                elif packet.stream_name == "color":
                    seq_no = packet.getMetadata().getSequenceNum()
                    if seq_no in m_d2h_seq_focus:
                        curr_focus = m_d2h_seq_focus[seq_no]
                        print('rgb_check_count -> {}'.format(rgb_check_count))
                        print('seq_no -> {}'.format(seq_no))
                        print('curr_focus -> {}'.format(curr_focus))
                        
                        if curr_focus < self.focus_value + 1 and curr_focus > self.focus_value - 1:
                            rgb_check_count += 1
                        else:
                            self.set_focus()
                            rgb_check_count = -2
                            # rospy.sleep(1)
                        # color_pkt_queue.append(packet)
                    if rgb_check_count >= 5:
                        recent_color = cv2.cvtColor(self.cvt_bgr(packet), cv2.COLOR_BGR2GRAY)
                    else:
                        recent_color = None
                elif packet.stream_name == "meta_d2h":
                    str_ = packet.getDataAsStr()
                    dict_ = json.loads(str_)
                    # ts_color_dev = dict_['camera']['last_frame_timestamp']
                    # current_focus = dict_['camera']['rgb']['focus_pos']
                    m_d2h_seq_focus[dict_['camera']['rgb']['frame_count']] = dict_['camera']['rgb']['focus_pos'] 
                    print('series of focus ---> {}'.format(dict_['camera']['rgb']['focus_pos'] ))

            if recent_left is not None and recent_right is not None and recent_color is not None:
                finished = True

        # print("looping")
        # is_board_found_l = find_chessboard(recent_left)
        # is_board_found_r = find_chessboard(recent_right)
        # is_board_found_l = True
        # is_board_found_r = True
        is_board_found_l = self.is_markers_found(recent_left)
        is_board_found_r = self.is_markers_found(recent_right)
        is_board_found_rgb = self.is_markers_found(recent_color)
        if is_board_found_l and is_board_found_r and is_board_found_rgb:
            print("Found------------------------->")
            self.parse_frame(recent_left, "left", req.name)
            self.parse_frame(recent_right, "right", req.name)
            self.parse_frame(recent_color, "rgb", req.name)
        else:
            print("Not found--------------------->")
            self.is_service_active = False
            self.parse_frame(recent_left, "left_not", req.name)
            self.parse_frame(recent_right, "right_not", req.name)
            self.parse_frame(recent_color, "rgb_not", req.name)
            return (False, "Calibration board not found")
        # elif is_board_found_l and not is_board_found_r: ## TODO: Add errors after srv is built
        print("Service ending")
        self.is_service_active = False
        return (True, "No Error")

    def device_status_handler(self, req):
        self.is_service_active = True
        self.start_disp = True
        # pygame.draw.rect(self.screen, white, no_button)
        while not self.device.is_device_changed():
            text = "Waiting for device change"
            pygame_render_text(self.screen, text, (250, 400), orange, 40)

        # to remove waiting for device change
        fill_color = pygame.Rect(200, 400, 450, 55)
        pygame.draw.rect(self.screen, white, fill_color)

        # to remove previous date and stuff
        fill_color = pygame.Rect(400, 70, 500, 150)
        pygame.draw.rect(self.screen, white, fill_color)
        now_time = datetime.datetime.now()
        text = "date/time : " + now_time.strftime("%m-%d-%Y %H:%M:%S")
        pygame_render_text(self.screen, text, (400, 80), black, 30)
        text = "device Mx_id : " + self.device.get_mx_id()
        pygame_render_text(self.screen, text, (400, 120), black, 30)
        rospy.sleep(3)
        fill_color_2 = pygame.Rect(50, 520, 400, 80)
        pygame.draw.rect(self.screen, white, fill_color_2)
        while self.device.is_device_changed():
            # print(self.device.is_device_changed())
            # if self.capture_exit():
            #     print("signaling...")
            #     rospy.signal_shutdown("Finished calibration")
            is_usb3 = False
            left_mipi = False
            right_mipi = False
            if self.args['enable_IMU_test']:
                is_IMU_connected = False
            else:
                is_IMU_connected = True
            is_usb3 = self.device.is_usb3()
            left_status = self.device.is_left_connected()
            right_status = self.device.is_right_connected()

            imu_times = 0
            if self.capture_exit():
                rospy.signal_shutdown("Finished calibration")

            # else
            if left_status and right_status:
                # mipi check using 20 iterations
                # ["USB3", "Left camera connected", "Right camera connected", "left Stream", "right Stream"]
                for _ in range(90):
                    _, data_list = self.pipeline.get_available_nnet_and_data_packets(
                        True)
                    # print(len(data_list))
                    for packet in data_list:
                        # print("found packets:")
                        # print(packet.stream_name)
                        if packet.stream_name == "left":
                            recent_left = packet.getData()
                            left_mipi = True
                            self.image_pub_left.publish(
                                self.bridge.cv2_to_imgmsg(recent_left, "passthrough"))
                        elif packet.stream_name == "right":
                            recent_right = packet.getData()
                            right_mipi = True
                            self.image_pub_right.publish(
                                self.bridge.cv2_to_imgmsg(recent_right, "passthrough"))
                        elif packet.stream_name == "meta_d2h":
                            str_ = packet.getDataAsStr()
                            if not self.args['enable_IMU_test']:
                                continue
                            dict_ = json.loads(str_)
                            if 'imu' in dict_:
                                if imu_times >= 5:
                                    is_IMU_connected = True
                                fill_color = pygame.Rect(50, 500, 400, 100)
                                pygame.draw.rect(
                                    self.screen, white, fill_color)
                                selected_clr = red

                                if dict_['imu']['status'] == 'IMU init FAILED':
                                    selected_clr = red
                                else:
                                    imu_times += 1
                                    selected_clr = green
                                    text = 'IMU acc x: {:7.4f}  y:{:7.4f}  z:{:7.4f}'.format(
                                        dict_['imu']['accel']['x'], dict_['imu']['accel']['y'], dict_['imu']['accel']['z'])
                                    pygame_render_text(
                                        self.screen, text, (50, 545), font_size=25)
                                    text = 'IMU acc-raw x: {:7.4f}  y:{:7.4f}  z:{:7.4f}'.format(
                                        dict_['imu']['accelRaw']['x'], dict_['imu']['accelRaw']['y'], dict_['imu']['accelRaw']['z'])
                                    pygame_render_text(
                                        self.screen, text, (50, 570), font_size=25)

                                text = 'IMU status: ' + dict_['imu']['status']
                                pygame_render_text(
                                    self.screen, text, (50, 500), font_size=30, color=selected_clr)

                            # print(dict_)
                            # if 'logs' in dict_:
                            #     for log in dict_['logs']:
                            #         if log != 'IMU init FAILED':
                            #             text = 'IMU status: ' + log
                            #             is_IMU_connected = True
                            #             pygame_render_text(self.screen, text, (50, 500), font_size=25, color=green)
                            #         else:
                            #             text = 'IMU status: ' + log
                            #             pygame_render_text(self.screen, text, (50, 500), font_size=25, color=red)
                                # print('meta_d2h LOG------------------>:', log)
                    if left_mipi and right_mipi and is_IMU_connected:
                        if is_usb3:
                            # # setting manual focus to rgb camera
                            # cam_c = depthai.CameraControl.CamId.RGB
                            # cmd_set_focus = depthai.CameraControl.Command.MOVE_LENS
                            # self.device.send_camera_control(cam_c, cmd_set_focus, '111')
                            self.device.reset_device_changed()
                        # for i in range(len(self.auto_checkbox_names)):
                        #     self.auto_checkbox_dict[self.auto_checkbox_names[i]].render_checkbox()
                        # print("device reste")
                        # print(self.device.is_device_changed())
                        break

            is_usb3 = self.device.is_usb3()
            left_status = self.device.is_left_connected()
            right_status = self.device.is_right_connected()

            if not is_usb3:
                self.auto_checkbox_dict["USB3"].uncheck()
            else:
                self.auto_checkbox_dict["USB3"].check()

            if not left_status:
                self.auto_checkbox_dict["Left camera connected"].uncheck()
            else:
                self.auto_checkbox_dict["Left camera connected"].check()

            if not left_mipi:
                self.auto_checkbox_dict["Left Stream"].uncheck()
            else:
                self.auto_checkbox_dict["Left Stream"].check()

            if not right_status:
                self.auto_checkbox_dict["Right camera connected"].uncheck()
            else:
                self.auto_checkbox_dict["Right camera connected"].check()

            if not right_mipi:
                # img = np.zeros((720, 1280, 3), np.uint8)
                # img[:] = (0, 0, 255)
                # cv2.putText(img, "right mipi",  (100, 300), cv2.FONT_HERSHEY_DUPLEX, 10.0, (0,0,0), 20)
                # cv2.putText(img, "Failed ", (140, 600), cv2.FONT_HERSHEY_SIMPLEX, 10.0, (0,0,0), 20)
                # cv2.imshow('right mipi camera failed', img)
                self.auto_checkbox_dict["Right Stream"].uncheck()
            else:
                self.auto_checkbox_dict["Right Stream"].check()

            if self.args['enable_IMU_test']:
                if is_IMU_connected:
                    self.auto_checkbox_dict["IMU connected"].check()
                else:
                    self.auto_checkbox_dict["IMU connected"].uncheck()

            for i in range(len(self.auto_checkbox_names)):
                self.auto_checkbox_dict[self.auto_checkbox_names[i]].render_checkbox(
                )

        # self.set_focus()
        rospy.sleep(2)
        self.is_service_active = False
        return (True, self.device.get_mx_id())

    def calibration_servive_handler(self, req):
        self.is_service_active = True
        print("calibration Service Started")
        # pygame.draw.rect(self.screen, white, no_button)

        mx_serial_id = self.device.get_mx_id()
        calib_dest_path = os.path.join(
            arg['calib_path'], arg["board"] + '_' + mx_serial_id + '.calib')

        flags = [self.config['board_config']['stereo_center_crop']]
        stereo_calib = StereoCalibration()
        avg_epipolar_error, calib_data = stereo_calib.calibrate(
            self.package_path + "/dataset",
            self.args['square_size_cm'],
            calib_dest_path,
            flags,
            req.name,
            self.args['marker_size_cm'])

        text = "Avg Epipolar error : " + format(avg_epipolar_error, '.6f')
        pygame_render_text(self.screen, text, (400, 160), green, 30)
        if avg_epipolar_error > 0.5:
            text = "Failed due to high calibration error"
            pygame_render_text(self.screen, text, (400, 200), red, 30)
            return (False, text)
        # self.rundepthai()

        dev_config = {
            'board': {},
            '_board': {}
        }
        dev_config["board"]["clear-eeprom"] = False
        dev_config["board"]["store-to-eeprom"] = True
        dev_config["board"]["override-eeprom"] = False
        dev_config["board"]["swap-left-and-right-cameras"] = self.board_config['board_config']['swap_left_and_right_cameras']
        dev_config["board"]["left_fov_deg"] = self.board_config['board_config']['left_fov_deg']
        dev_config["board"]["rgb_fov_deg"] = self.board_config['board_config']['rgb_fov_deg']
        dev_config["board"]["left_to_right_distance_m"] = self.board_config['board_config']['left_to_right_distance_cm'] / 100
        dev_config["board"]["left_to_rgb_distance_m"] = self.board_config['board_config']['left_to_rgb_distance_cm'] / 100
        dev_config["board"]["name"] = self.board_config['board_config']['name']
        dev_config["board"]["stereo_center_crop"] = True
        dev_config["board"]["revision"] = self.board_config['board_config']['revision']
        dev_config["_board"]['calib_data'] = list(calib_data)
        dev_config["_board"]['mesh_right'] = [0.0]
        dev_config["_board"]['mesh_left'] = [0.0]

        self.device.write_eeprom_data(dev_config)

        # calib_src_path = os.path.join(arg['depthai_path'], "resources/depthai.calib")
        # shutil.copy(calib_src_path, calib_dest_path)
        print("finished writing to EEPROM with Epipolar error of")

        print(avg_epipolar_error)
        print('Validating...')
        is_write_succesful = False
        run_thread = True
        while run_thread:
            _, data_packets = self.pipeline.get_available_nnet_and_data_packets(
                blocking=True)
            for packet in data_packets:
                if packet.stream_name == 'meta_d2h':
                    str_ = packet.getDataAsStr()
                    dict_ = json.loads(str_)
                    if 'logs' in dict_:
                        for log in dict_['logs']:
                            print(log)
                            if 'EEPROM' in log:
                                if 'write OK' in log:
                                    text = "EEPROM Write succesfull"
                                    pygame_render_text(
                                        self.screen, text, (400, 200), green, 30)
                                    is_write_succesful = True
                                    run_thread = False
                                elif 'FAILED' in log:
                                    text = "EEPROM write Failed"
                                    pygame_render_text(
                                        self.screen, text, (400, 200), red, 30)
                                    is_write_succesful = False
                                    run_thread = False
        self.is_service_active = False
        if is_write_succesful:
            return (True, "EEPROM written succesfully")
        else:
            return (False, "EEPROM write Failed!!")

    def rundepthai(self):
        test_cmd = "python3 " + \
            self.args['depthai_path'] + \
            "/depthai_demo.py -e -brd " + self.args['brd']
        print(test_cmd)
        # test_cmd = """python3 depthai.py -brd bw1098obc -e""" ## TODO(sachin): Parameterize the board name
        self.p = subprocess.Popen(test_cmd, shell=True, preexec_fn=os.setsid)
        time.sleep(4)
        os.killpg(os.getpgid(self.p.pid), signal.SIGKILL)

        time.sleep(1)


no_button = pygame.Rect(490, 500, 80, 45)

if __name__ == "__main__":

    rospy.init_node('depthai_calibration', anonymous=True)
    arg = {}
    arg["swap_lr"] = rospy.get_param('~swap_lr')
    arg["field_of_view"] = rospy.get_param('~field_of_view')
    arg["baseline"] = rospy.get_param('~baseline')
    arg["package_path"] = rospy.get_param('~package_path')
    arg["square_size_cm"] = rospy.get_param('~square_size_cm')
    arg["marker_size_cm"] = rospy.get_param('~marker_size_cm')

    # arg["depthai_path"] = rospy.get_param('~depthai_path') ## Add  capture_checkerboard to launch file
    # Add  capture_checkerboard to launch file
    arg["board"] = rospy.get_param('~brd')
    arg["capture_service_name"] = rospy.get_param(
        '~capture_service_name')  # Add  capture_checkerboard to launch file
    arg["calibration_service_name"] = rospy.get_param(
        '~calibration_service_name')  # Add  capture_checkerboard to launch file
    arg["depthai_path"] = rospy.get_param(
        '~depthai_path')  # Path of depthai repo
    arg["enable_IMU_test"] = rospy.get_param('~enable_IMU_test')
    # local path to store calib files with using mx device id.
    arg["calib_path"] = str(Path.home()) + rospy.get_param('~calib_path')

    # print("Hone------------------------>")
    # print(type(arg["enable_IMU_test"]))

    if not os.path.exists(arg['calib_path']):
        os.makedirs(arg['calib_path'])

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
    #     Please add the correct path using depthai_path:=[path] while executing launchfile")

    depthai_dev = depthai_calibration_node(arg)
    depthai_dev.publisher()
    rospy.spin()
