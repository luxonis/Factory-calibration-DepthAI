#!/usr/bin/env python

import cv2
import sys
import copy
import platform
import signal
import subprocess
import json
import datetime
import csv
import time
import numpy as np
import os
from pathlib import Path
import shutil
from datetime import datetime
import socket
import netifaces as ni
import pickle

import rospy
from std_msgs.msg import String
from calibration.srv import Capture
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import struct

import depthai
import consts.resource_paths
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


class socket_calibration_node:
    def __init__(self, depthai_args):
        self.package_path = depthai_args['package_path']
        self.args = depthai_args
        self.bridge = CvBridge()
        self.is_service_active = False
        self.focus_value = 130
        # self.frame_count = 0
        self.init_time = time.time()

        ip = ni.ifaddresses('enp6s0')[ni.AF_INET][0]['addr']
        PORT = 51264
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((ip, PORT))
        self.sock.listen()
        self.conn, addr = self.sock.accept()

        self.aruco_dictionary = cv2.aruco.Dictionary_get(
            cv2.aruco.DICT_4X4_1000)
        # self.charuco_board = aruco.CharucoBoard_create(
        #                                     22,16,
        #                                     self.args['square_size_cm'],
        #                                     self.args['marker_size_cm'],
        #                                     self.aruco_dictionary)
        # self.start_device()

        self.disp = pygame.display
        self.screen = self.disp.set_mode((800, 600))
        self.screen.fill(white)
        self.disp.set_caption("Calibration - Device check ")
        title = "Device Status"
        pygame_render_text(self.screen, title, (350, 20), orange, 50)
        self.auto_checkbox_names = ["USB3", "Left camera connected", "Right camera connected", "RGB camera connected",
                                    "Left Stream", "Right Stream", "RGB Stream"]
        header = ['time', 'Mx_serial_id', 'USB3',
                  'left_camera', 'right_camera', 'rgb_camera']

        if self.args['enable_IMU_test']:
            self.auto_checkbox_names.append("IMU connected")
            header.append('IMU')

        header.append('Epipolar error L-R')
        header.append('Epipolar error RGB-R')

        log_file = self.args['log_path'] + "/calibration_logs.csv"
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

        # self.image_pub_left = rospy.Publisher("left", Image, queue_size=10)
        # self.image_pub_right = rospy.Publisher("right", Image, queue_size=10)
        # self.image_pub_color = rospy.Publisher("color", Image, queue_size=10)

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

        # adding backup
        # now_tim = datetime.now()
        self.backup_ds(stream_name, file_name, frame)
        return True

    def backup_ds(self, stream_name, file_name, frame):
        now_tim = datetime.now()
        local_ds = self.args['ds_backup_path'] + '/' + \
            now_tim.strftime("%m_%d_%Y_%H") + '/{}'.format(stream_name)
        if not os.path.exists(local_ds):
            os.makedirs(local_ds)
        cv2.imwrite(local_ds + "/{}".format(file_name), frame)

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
        self.is_service_active = True
        # now = rospy.get_rostime()
        # pygame.draw.rect(self.screen, white, no_button)
        rospy.sleep(1)

        payload_size = struct.calcsize(">L")
        data = b""
        self.conn.sendall('capture_req')

        # Fetching left image
        while len(data) < payload_size:
            data += self.conn.recv(4096)
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]

        while len(data) < msg_size:
            data += self.conn.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]
        recent_left = pickle.loads(
            frame_data, fix_imports=True, encoding="bytes")

        # Fetching right image
        while len(data) < payload_size:
            data += self.conn.recv(4096)
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]

        while len(data) < msg_size:
            data += self.conn.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]
        recent_right = pickle.loads(
            frame_data, fix_imports=True, encoding="bytes")

        # Fetching color image
        while len(data) < payload_size:
            data += self.conn.recv(4096)
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]

        while len(data) < msg_size:
            data += self.conn.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]
        recent_color = pickle.loads(
            frame_data, fix_imports=True, encoding="bytes")

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

        if req.name == "recalibrate":
            self.conn.sendall(b'check_conn_rep')
        else:
            text = "Waiting for device to connect"
            pygame_render_text(self.screen, text, (300, 400), orange, 40)
            self.conn, _ = self.sock.accept()

        # to remove waiting for device change
        fill_color = pygame.Rect(280, 400, 450, 55)
        pygame.draw.rect(self.screen, white, fill_color)

        recv_data = self.conn.recv(1024)
        check_list = pickle.loads(recv_data)
        # to remove previous date and stuff
        fill_color = pygame.Rect(400, 70, 500, 300)
        pygame.draw.rect(self.screen, white, fill_color)
        now_time = datetime.now()
        text = "date/time : " + now_time.strftime("%m-%d-%Y %H:%M:%S")
        pygame_render_text(self.screen, text, (400, 80), black, 30)
        text = "device Mx_id : " + check_list[0]
        pygame_render_text(self.screen, text, (400, 120), black, 30)
        rospy.sleep(1)
        fill_color_2 = pygame.Rect(50, 520, 400, 80)
        pygame.draw.rect(self.screen, white, fill_color_2)

        dataset_path = Path(self.package_path + "/dataset")
        if dataset_path.exists():
            shutil.rmtree(str(dataset_path))

        is_valid = False
        while not is_valid:
            # print(self.device.is_device_changed())
            # if self.capture_exit():
            #     print("signaling...")
            #     rospy.signal_shutdown("Finished calibration")

            is_usb3 = check_list[1]
            # i2c connection check
            left_status = check_list[2]
            right_status = check_list[3]
            rgb_status = check_list[4]
            # mipi connection check
            left_mipi = check_list[5]
            right_mipi = check_list[6]
            rgb_mipi = check_list[7]

            if self.capture_exit():
                rospy.signal_shutdown("Finished calibration")

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

            if not rgb_status:
                self.auto_checkbox_dict["RGB camera connected"].uncheck()
            else:
                self.auto_checkbox_dict["RGB camera connected"].check()

            if not rgb_mipi:
                self.auto_checkbox_dict["RGB Stream"].uncheck()
            else:
                self.auto_checkbox_dict["RGB Stream"].check()

            if not right_status:
                self.auto_checkbox_dict["Right camera connected"].uncheck()
            else:
                self.auto_checkbox_dict["Right camera connected"].check()

            if not right_mipi:
                self.auto_checkbox_dict["Right Stream"].uncheck()
            else:
                self.auto_checkbox_dict["Right Stream"].check()

            for i in range(len(self.auto_checkbox_names)):
                self.auto_checkbox_dict[self.auto_checkbox_names[i]].render_checkbox(
                )

            is_valid = is_usb3 and left_status and right_status and rgb_status and left_mipi and right_mipi and rgb_mipi

            if not is_valid:
                recv_data = self.conn.recv(1024)
                check_list = pickle.loads(recv_data)

        # self.set_focus()
        # rospy.sleep(2)
        self.is_service_active = False
        self.final_check_list = check_list
        return (True, self.final_check_list[0])

    def calibration_servive_handler(self, req):
        self.is_service_active = True
        print("calibration Service Started")
        # pygame.draw.rect(self.screen, white, no_button)

        mx_serial_id = self.final_check_list[0]
        calib_dest_path = os.path.join(
            arg['calib_path'], arg["board"] + '_' + mx_serial_id + '.calib')

        flags = [True]
        stereo_calib = StereoCalibration()
        avg_epipolar_error_l_r, avg_epipolar_error_rgb_r, calib_data = stereo_calib.calibrate(
            self.package_path + "/dataset",
            self.args['square_size_cm'],
            calib_dest_path,
            flags,
            req.name,
            self.args['marker_size_cm'])

        start_time = datetime.now()
        time_stmp = start_time.strftime("%m-%d-%Y %H:%M:%S")

        # mx_serial_id = self.device.get_mx_id()
        log_list = [time_stmp, self.final_check_list[0]]
        log_list.append(self.final_check_list[1])
        log_list.append(self.final_check_list[2])
        log_list.append(self.final_check_list[3])
        log_list.append(self.final_check_list[4])

        # if self.args['enable_IMU_test']:
        #     log_list.append(self.imu_version)

        log_list.append(avg_epipolar_error_l_r)
        log_list.append(avg_epipolar_error_rgb_r)

        log_file = self.args['log_path'] + "/calibration_logs.csv"
        with open(log_file, mode='a') as log_fopen:
            # header =
            log_csv_writer = csv.writer(log_fopen, delimiter=',')
            log_csv_writer.writerow(log_list)

        text = "Avg Epipolar error L-R: " + \
            format(avg_epipolar_error_l_r, '.6f')
        pygame_render_text(self.screen, text, (400, 160), green, 30)
        text = "Avg Epipolar error RGB-R: " + \
            format(avg_epipolar_error_rgb_r, '.6f')
        pygame_render_text(self.screen, text, (400, 190), green, 30)

        if avg_epipolar_error_l_r > 0.5:
            text = "Failed due to high calibration error L-R"
            pygame_render_text(self.screen, text, (400, 230), red, 30)
            return (False, text)
        # self.rundepthai()

        if avg_epipolar_error_rgb_r > 0.7:
            text = "Failed due to high calibration error RGB-R"
            pygame_render_text(self.screen, text, (400, 270), red, 30)
            return (False, text)

        # calib_src_path = os.path.join(arg['depthai_path'], "resources/depthai.calib")
        # shutil.copy(calib_src_path, calib_dest_path)
        print("finished writing to EEPROM with Epipolar error of")

        print(avg_epipolar_error_l_r)
        print('Validating...')
        is_write_succesful = False
        run_thread = True
        self.conn.sendall('write_eeprom')
        data = pickle.dumps(calib_data)
        self.conn.sendall(data)

        recv_data = self.conn.recv(1024)
        eeprom_status = pickle.loads(recv_data)

        self.is_service_active = False

        if eeprom_status:
            text = "EEPROM Write succesfull"
            pygame_render_text(
                self.screen, text, (400, 220), green, 30)
            return (True, "EEPROM written succesfully")
        else:
            text = "EEPROM write Failed"
            pygame_render_text(
                self.screen, text, (400, 220), red, 30)
            return (False, "EEPROM write Failed!!")


no_button = pygame.Rect(490, 500, 80, 45)

if __name__ == "__main__":

    rospy.init_node('1097_calibration', anonymous=True)
    arg = {}
    arg["swap_lr"] = rospy.get_param('~swap_lr')
    arg["field_of_view"] = rospy.get_param('~field_of_view')
    arg["baseline"] = rospy.get_param('~baseline')
    arg["package_path"] = rospy.get_param('~package_path')
    arg["square_size_cm"] = rospy.get_param('~square_size_cm')
    arg["marker_size_cm"] = rospy.get_param('~marker_size_cm')
    arg["board"] = rospy.get_param('~brd')
    arg["depthai_path"] = rospy.get_param(
        '~depthai_path')  # Path of depthai repo
    arg["enable_IMU_test"] = rospy.get_param('~enable_IMU_test')
    # local path to store calib files with using mx device id.
    arg["calib_path"] = str(Path.home()) + rospy.get_param('~calib_path')
    arg["log_path"] = str(Path.home()) + rospy.get_param("~log_path")
    arg["ds_backup_path"] = str(Path.home()) + '/Desktop/ds_backup'

    # Adding service names to arg
    arg["capture_service_name"] = rospy.get_param(
        '~capture_service_name')  # Add  capture_checkerboard to launch file
    arg["calibration_service_name"] = rospy.get_param(
        '~calibration_service_name')  # Add  capture_checkerboard to launch file

    # print("Hone------------------------>")
    # print(type(arg["enable_IMU_test"]))

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
    #     Please add the correct path using depthai_path:=[path] while executing launchfile")

    depthai_dev = socket_calibration_node(arg)
    # depthai_dev.publisher()
    rospy.spin()
