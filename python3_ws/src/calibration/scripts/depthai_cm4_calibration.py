#!/usr/bin/env python3
import base64

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
import ssh_wait

import depthai as dai
import consts.resource_paths
from depthai_calibration.calibration_utils import *

from depthai_helpers.pygame_checkbox import Checkbox, pygame_render_text
import pygame
from pygame.locals import *

from depthai_helpers import utils

import socket, pickle

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
    'RGB': dai.CameraBoardSocket.RGB,
    'LEFT': dai.CameraBoardSocket.LEFT,
    'RIGHT': dai.CameraBoardSocket.RIGHT,
    'CAM_A': dai.CameraBoardSocket.RGB,
    'CAM_B': dai.CameraBoardSocket.LEFT,
    'CAM_C': dai.CameraBoardSocket.RIGHT,
}


class SocketWorker:
    def __init__(self):
        self.PORT = 51014
        self.connection()
        self.buffer = 4096
        self.FPS = 1 / 60

    def connection(self):
        HOST = 'luxonis.local'
        ssh_wait.ssh_wait(HOST, service='ssh', wait=True, wait_limit=600, log_fn=print )
        # HOST = "192.168.1.5"
        # var_path = Path(consts.resource_paths.boards_dir_path) / \
        #              Path('consts.resource_paths.txt')
        # if os.path.exists(var_path):
        #     f = open(var_path)
        #     self.DEPTHAI_ALLOW_FACTORY_FLASHING = f.read().splitlines()[0]
        # else:
        #     self.DEPTHAI_ALLOW_FACTORY_FLASHING = None
        # install production and set env
        # os.system(f'sshpass -p raspberry ssh pi@{HOST} unset HISTFILE')
        # os.system(f'sshpass -p raspberry scp ~/Factory-calibration-DepthAI/server.py pi@{HOST}:/home/pi')
        # os.system(f'sshpass -p raspberry scp -r ~/Factory-calibration-DepthAI/pip_packages pi@{HOST}:/home/pi')
        # os.system(f'sshpass -p raspberry ssh pi@{HOST} pip3 install /home/pi/pip_packages/*')
        # DEPTHAI_ALLOW_FACTORY_FLASHING = os.environ.get('DEPTHAI_ALLOW_FACTORY_FLASHING')

        # set pre-set env
        # TODO: Temp, remove after GPIO fix
        os.system(f" ssh-keygen -f '/home/luxnuc-a/.ssh/known_hosts' -R 'luxonis.local'")
        os.system(f'sshpass -p raspberry ssh pi@{HOST} killall python')
        os.system(f'sshpass -p raspberry scp -r ~/Factory-calibration-DepthAI/enable_cm4_oak.py ~/Factory-calibration-DepthAI/rc.local ~/Factory-calibration-DepthAI/calib_env/ ~/Factory-calibration-DepthAI/server.py pi@{HOST}:/home/pi/')
        os.system(f'sshpass -p raspberry ssh pi@luxonis.local "echo raspberry | sudo -S mv /home/pi/rc.local /etc/;python3 /home/pi/enable_cm4_oak.py"')
        os.system(f"\
            sshpass -p raspberry ssh pi@{HOST} '\
            unset HISTFILE; \
            source calib_env/bin/activate; \
            nohup python server.py > /home/pi/server_out 2>&1 &'")

        
        time.sleep(2)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, self.PORT))
        self.conn = s
        self.PORT += 1

    def send(self, msg):
        try:
            data_string = pickle.dumps(msg)
            size = len(data_string)
            print(f'size={size}')
            self.conn.send(pickle.dumps(size))
            time.sleep(self.FPS)
            self.join()
            self.conn.sendall(data_string)
            print(f'send_msg={msg}')
            self.join()
        except Exception as e:
            print(e)
            self.conn.shutdown(1)
            self.conn.close()
            self.connection()
            self.send(msg)

    def recv(self):
        try:
            data = []
            size = 0
            packet = self.conn.recv(self.buffer)
            expected_size = pickle.loads(packet)
            print(f'expected_size={expected_size}')
            time.sleep(self.FPS/2)
            self.ack()
            while size < expected_size:
                packet = self.conn.recv(self.buffer)
                if expected_size < self.buffer:
                    print(f'pickle.loads(packet)={pickle.loads(packet)}')
                size += len(packet)
                data.append(packet)
            msg = pickle.loads(b"".join(data))
            print(f'msg={msg}')
            time.sleep(self.FPS/2)
            self.ack()
            return msg
        except Exception as e:
            print(e)
            self.conn.shutdown(1)
            self.conn.close()
            time.sleep(1)
            self.connection()
            self.recv()

    def join(self):
        msg = pickle.loads(self.conn.recv(4096))
        if not (msg == 'ACK'):
            raise RuntimeError(f'client join error msg={msg}')
        print('join=ACK')

    def ack(self):
        self.conn.send(pickle.dumps('ACK'))

    def __del__(self):
        if hasattr(self, 'conn'):
            self.conn.close()
        HOST = 'luxonis.local'
        # HOST = "192.168.1.5"
        os.system(f"sshpass -p raspberry ssh pi@{HOST} 'killall -9 server.py; rm -rf calib_env server.py; history -c'")


class depthai_calibration_node:
    def __init__(self, depthai_args):
        self.package_path = depthai_args['package_path']
        self.args = depthai_args
        self.traceLevel = self.args["traceLevel"]
        self.outputScaleFactor = self.args["outputScaleFactor"]
        self.bridge = CvBridge()
        self.is_service_active = False

        self.disp = pygame.display
        self.screen = self.disp.set_mode((800, 600))
        self.screen.fill(white)
        self.disp.set_caption("Calibration - Device check ")

        self.focusSigmaThreshold = 30

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
            self.auto_checkbox_names.append(cam_info['name'] + '-Camera-connected')
            self.auto_checkbox_names.append(cam_info['name'] + '-Stream')
            self.auto_focus_checkbox_names.append(cam_info['name'] + '-Focus')
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
            y_axis = y + (40 * i)
            font_surf = font.render(self.auto_checkbox_names[i], True, green)
            self.screen.blit(font_surf, (x_axis, y_axis))
            self.auto_checkbox_dict[self.auto_checkbox_names[i]] = Checkbox(self.screen, x + 10, y_axis - 5,
                                                                            outline_color=green,
                                                                            check_color=green, check=False)
        # text = 'call rosservice of device_status_handler to update the device status'
        for i in range(len(self.auto_checkbox_names)):
            self.auto_checkbox_dict[self.auto_checkbox_names[i]].render_checkbox()

        y = y + (40 * len(self.auto_checkbox_names))
        self.auto_focus_checkbox_dict = {}

        for i in range(len(self.auto_focus_checkbox_names)):
            w, h = font.size(self.auto_focus_checkbox_names[i])
            x_axis = x - w
            y_axis = y + (40 * i)
            font_surf = font.render(self.auto_focus_checkbox_names[i], True, green)
            self.screen.blit(font_surf, (x_axis, y_axis))
            self.auto_focus_checkbox_dict[self.auto_focus_checkbox_names[i]] = Checkbox(self.screen, x + 10, y_axis - 5,
                                                                                        outline_color=green,
                                                                                        check_color=green, check=False)

        for i in range(len(self.auto_focus_checkbox_names)):
            self.auto_focus_checkbox_dict[self.auto_focus_checkbox_names[i]].render_checkbox()

        pygame.draw.rect(self.screen, red, no_button)
        pygame_render_text(self.screen, 'Exit', (500, 505))
        self.no_active = False
        self.click = False
        # start connexion with server

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
        print(f'self.board_config["cameras"]={self.board_config["cameras"]}')
        for cam_id in self.board_config['cameras']:
            print(f'cam_id={cam_id}')
            name = self.board_config['cameras'][cam_id]['name']
            print(f'name={name}')
            self.imgPublishers[name] = rospy.Publisher(name, Image, queue_size=10)
            print(f'self.imgPublishers[name]={self.imgPublishers[name]}')

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
            y_axis = y + (60 * i)
            font_surf = font.render(ccm_names[i], True, black)
            self.screen.blit(font_surf, (x_axis, y_axis))
            ccm_names_dict[ccm_names[i]] = []
            offset = 150
            offset_increment = 1
            for camera in self.board_config['cameras'].keys():
                ccm_names_dict[ccm_names[i]].append(
                    Checkbox(self.screen, x + (offset * offset_increment), y_axis - 5, outline_color=green,
                             check_color=green, check=False, disable_pass=True))
                ccm_names_dict[ccm_names[i]][-1].render_checkbox()
                offset_increment += 1

            fill_color = pygame.Rect(20, y_axis + 40, 750, 2)
            pygame.draw.rect(self.screen, black, fill_color)

        next_button = pygame.Rect(600, 430, 60, 35)
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
                            pygame_render_text(self.screen, "Select the type of module before clicking next ",
                                               (150, 480), red)
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
        retest_button = pygame.Rect(400, 430, 100, 35)
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
        try:
            del self.socket_worker
        except AttributeError:
            pass
        self.socket_worker = SocketWorker()

        self.is_service_active = True
        self.start_disp = True  # TODO(sachin): Change code to Use this for display

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

        self.socket_worker.send('start_camera')
        # self.socket_worker.join()
        self.socket_worker.send(self.board_config)
        self.board_config = self.board_config_backup
        # self.socket_worker.join()
        finished = False
        while not finished:
            if self.capture_exit():
                rospy.signal_shutdown("Stopping calibration")
            while self.socket_worker.recv() != 'device_connected':
                if self.capture_exit():
                    rospy.signal_shutdown("Stopping calibration")
                if self.socket_worker.recv() == 'is_found':
                    # cameraList = self.device.getConnectedCameras()
                    # cameraProperties = self.device.getConnectedCameraProperties()
                    fill_color_2 = pygame.Rect(390, 120, 500, 100)
                    pygame.draw.rect(self.screen, white, fill_color_2)

                    rospy.sleep(2)
                    # dev_info = self.device.getDeviceInfo()
                    text = self.socket_worker.recv()
                    pygame_render_text(self.screen, text, (400, 120), black, 30)
                    text = "Device Connected!!!"
                    pygame_render_text(self.screen, text, (400, 150), green, 30)

                    lost_camera = False
                    # self.socket_worker.ack()
                    while self.socket_worker.recv() != 'last_property':
                        for in_cam in self.board_config['cameras'].keys():
                            cam_info = self.board_config['cameras'][in_cam]
                            if self.socket_worker.recv() == 'checked':
                                self.auto_checkbox_dict[cam_info['name'] + '-Camera-connected'].check()
                                break

                    for config_cam in self.board_config['cameras'].keys():
                        cam_info = self.board_config['cameras'][config_cam]
                        if self.auto_checkbox_dict[cam_info['name'] + '-Camera-connected'].isUnattended():
                            self.auto_checkbox_dict[cam_info['name'] + '-Camera-connected'].uncheck()
                            lost_camera = True
                        self.auto_checkbox_dict[cam_info['name'] + '-Camera-connected'].render_checkbox()
                    # self.socket_worker.ack()
                    # self.socket_worker.join()
                    if self.args['usbMode']:
                        self.socket_worker.send('usb_mode')
                    else:
                        self.socket_worker.send('no_usb_mode')

                    if self.args['usbMode']:
                        msg = self.socket_worker.recv()
                        if msg == 'check':
                            self.auto_checkbox_dict["USB3"].check()
                        else:
                            lost_camera = True
                            self.auto_checkbox_dict["USB3"].uncheck()
                        self.auto_checkbox_dict["USB3"].render_checkbox()

                    if not lost_camera:
                        self.socket_worker.send('working_camera')
                        # self.socket_worker.join()
                    else:
                        self.socket_worker.send('lost_camera')
                        print("Closing Device...")

                        fill_color_2 = pygame.Rect(390, 150, 220, 35)
                        pygame.draw.rect(self.screen, white, fill_color_2)
                        text = "Device Disconnected!!!"
                        pygame_render_text(self.screen, text, (400, 150), red, 30)
                        text = "Click RETEST when device is ready!!!"
                        pygame_render_text(self.screen, text, (400, 180), red, 30)

                        # self.socket_worker.join()
                        self.retest()
                        print("Restarting Device...")

                        fill_color_2 = pygame.Rect(390, 430, 120, 35)
                        pygame.draw.rect(self.screen, white, fill_color_2)
            print(flush=True)
            # for _ in range(120):
            #     print(f'{_=}')
            #     # self.socket_worker.ack()
            #     # self.socket_worker.join()
            #     print(flush=True)
            #     msg = self.socket_worker.recv()
            #     while msg == 'next':
            #         print(f'next={msg}')
            #         msg = self.socket_worker.recv()
            #         print(f'good_frame={msg}')
            #         print(flush=True)
            #         if msg == 'good_frame':
            #             # self.socket_worker.ack()
            #             print(flush=True)
            #             frame = self.socket_worker.recv()
            #             print(flush=True)
            #             # self.socket_worker.ack()
            #             name = self.socket_worker.recv()
            #             self.imgPublishers[name].publish(self.bridge.cv2_to_imgmsg(frame, "passthrough"))
            #
            #         msg = self.socket_worker.recv()
            #     print(f'last_msg={msg}')
            #     print(flush=True)
            #     # self.socket_worker.join()
            #     print(flush=True)
            #     rospy.sleep(1)
            # wait = self.socket_worker.recv()
            # print(f'{wait=}')
            while self.socket_worker.recv() != 'finish_mipi':
                message = self.socket_worker.recv()
                if message[1] == 'check':
                    self.auto_checkbox_dict[message[0]].check()
                else:
                    self.auto_checkbox_dict[message[0]].uncheck()
                    self.auto_checkbox_dict[message[0]].check()
                    print(f'message[0]={message[0]}')
            rospy.sleep(1)

            for i in range(len(self.auto_checkbox_names)):
                self.auto_checkbox_dict[self.auto_checkbox_names[i]].render_checkbox()

            isAllPassed = True
            for key in self.auto_checkbox_dict.keys():
                # FIXME(sachin): is_checked is a function not a variable
                isAllPassed = isAllPassed and self.auto_checkbox_dict[key].is_checked()
            # self.socket_worker.ack() # TODO: remove black magic
            rospy.sleep(1)
            # self.socket_worker.send('finished')
            if isAllPassed:
                self.socket_worker.send('finished')
                # self.socket_worker.join()
                finished = True
            else:
                self.socket_worker.send('unfinished')
                # self.socket_worker.join()
                self.retest()

        dataset_path = Path(self.package_path + "/dataset")
        if 0 and dataset_path.exists():
            shutil.rmtree(str(dataset_path))
        # dev_info = self.device.getDeviceInfo()
        self.is_service_active = False
        print('Finished device status')
        return finished, self.socket_worker.recv()

    def camera_focus_adjuster(self, req):
        self.socket_worker.send('focus_adjuster')
        self.is_service_active = True
        a = 1
        cam_info = self.socket_worker.recv()
        # while self.socket_worker.recv() == 'next':
        #     print(f'1 -> {a}')
        #     a = a + 1
        # self.socket_worker.send()
        # currFrame = self.socket_worker.recv()

        # self.imgPublishers[cam_info['name']].publish(
        #             self.bridge.cv2_to_imgmsg(currFrame, "passthrough"))
        mx_id = self.socket_worker.recv()

        backupFocusPath = self.args['ds_backup_path'] + '/focus/' + mx_id
        if not os.path.exists(backupFocusPath):
            os.makedirs(backupFocusPath)

        while self.socket_worker.recv() == 'img':
            name = self.socket_worker.recv()
            image = self.socket_worker.recv()
            print('Backing up images {}'.format(name))
            cv2.imwrite(backupFocusPath + "/" + name + '.png', image)

        a = 1
        while self.socket_worker.recv() == 'focus':
            a = a + 1
            cam_name = self.socket_worker.recv()
            if self.socket_worker.recv() == 'check':
                self.auto_focus_checkbox_dict[cam_name + "-Focus"].check()
                self.auto_focus_checkbox_dict[cam_name + "-Focus"].render_checkbox()
            else:
                self.auto_focus_checkbox_dict[cam_name + "-Focus"].uncheck()
                self.auto_focus_checkbox_dict[cam_name + "-Focus"].render_checkbox()
        self.lensPosition = self.socket_worker.recv()
        self.focusSigma = self.socket_worker.recv()
        # self.socket_worker.send(self.auto_focus_checkbox_dict)
        for key in self.auto_focus_checkbox_dict.keys():
            if not self.auto_focus_checkbox_dict[key].is_checked():
                # self.socket_worker.send('close')
                self.is_service_active = False
                return (False, key + " is out of Focus")
            # else:
            #     print(key + " is in Focus")

        self.is_service_active = False
        return (True, "RGB in Focus")

    def capture_servive_handler(self, req):
        print("Capture image Service Started")
        self.is_service_active = True
        rospy.sleep(1)

        # TODO(Sachin): Add time synchronization here and get the most recent frame instead.
        frameCount = 0
        detection_failed = False
        # while not finished:
        # self.socket_worker.ack()
        self.socket_worker.send('capture_service')
        while True:
            key = self.socket_worker.recv()
            if key == 'next': break
            # self.imgPublishers[key].publish(self.bridge.cv2_to_imgmsg(self.socket_worker.recv(), "passthrough"))
            gray_frame = self.socket_worker.recv()
            is_board_found = self.is_markers_found(gray_frame)
            if is_board_found:
                self.parse_frame(gray_frame, key, req.name)
            else:
                self.parse_frame(gray_frame, key + '_not', req.name)
                detection_failed = True
        # TODO(sachin): Do I need to cross check lens position of autofocus camera's ?
        # self.socket_worker.ack()
        if detection_failed:
            self.socket_worker.send('close')
            self.is_service_active = False
            return False, "Calibration board not found"
        else:
            self.socket_worker.send('success')
            self.is_service_active = False
            return (True, "No Error")

    def calibration_servive_handler(self, req):
        global eepromDataJson
        print('Calibrate service')
        self.socket_worker.send('calibration_service')
        self.is_service_active = True
        print("calibration Service Started")
        # pygame.draw.rect(self.screen, white, no_button)

        mx_serial_id = self.socket_worker.recv()
        # dev_info = self.device.getDeviceInfo()
        # mx_serial_id = dev_info.getMxId()
        calib_dest_path = os.path.join(
            self.args['calib_path'], self.args["board"] + '_' + mx_serial_id + '.json')
        # print(self.package_path)
        stereo_calib = StereoCalibration(self.traceLevel, self.outputScaleFactor)
        status, result_config = stereo_calib.calibrate(
            self.board_config,
            self.package_path + "/dataset",
            self.args['square_size_cm'],
            self.args['marker_size_cm'],
            self.args['squares_x'],
            self.args['squares_y'],
            self.args['cameraModel'],
            False)  # Turn off enable disp rectify

        start_time = datetime.now()
        time_stmp = start_time.strftime("%m-%d-%Y %H:%M:%S")

        log_list = [time_stmp, mx_serial_id]
        # for key in self.ccm_selected.keys():
        #     log_list.append(self.ccm_selected[key])

        # self.socket_worker.ack()

        if status == -1:
            self.socket_worker.send('close')
            # self.close_device()
            self.is_service_active = False
            return result_config
        else:
            self.socket_worker.send('pass')

        vis_x = 400
        vis_y = 180
        error_text = []
        self.socket_worker.send(eepromDataJson)
        eepromDataJson['batchTime'] = int(time.time())
        # eepromDataJson = self.socket_worker.recv()
        calibration_handler = dai.CalibrationHandler.fromJson(eepromDataJson)
        for camera in result_config['cameras'].keys():
            cam_info = result_config['cameras'][camera]
            log_list.append(self.ccm_selected[cam_info['name']])

            color = green
            reprojection_error_threshold = 1.4  # TODO: Remove this check later
            if cam_info['size'][1] > 720:
                print(cam_info['size'][1])
                reprojection_error_threshold = reprojection_error_threshold * cam_info['size'][1] / 720

            if cam_info['name'] == 'rgb':
                reprojection_error_threshold = 6
            print('Reprojection error threshold -> {}'.format(reprojection_error_threshold))
            if cam_info['reprojection_error'] > reprojection_error_threshold:
                color = red
                error_text.append("high Reprojection Error")
            text = cam_info['name'] + ' Reprojection Error: ' + format(cam_info['reprojection_error'], '.6f')
            pygame_render_text(self.screen, text, (vis_x, vis_y), color, 30)

            calibration_handler.setDistortionCoefficients(stringToCam[camera], cam_info['dist_coeff'])
            calibration_handler.setCameraIntrinsics(stringToCam[camera], cam_info['intrinsics'], cam_info['size'][0],
                                                    cam_info['size'][1])
            calibration_handler.setFov(stringToCam[camera], cam_info['hfov'])
            try:
                if cam_info['hasAutofocus']:
                    calibration_handler.setLensPosition(stringToCam[camera], self.lensPosition[cam_info['name']])
            except:
                pass
            log_list.append(self.focusSigma[cam_info['name']])
            log_list.append(cam_info['reprojection_error'])

            vis_y += 30
            color = green

            if 'extrinsics' in cam_info:

                if 'to_cam' in cam_info['extrinsics']:
                    right_cam = result_config['cameras'][cam_info['extrinsics']['to_cam']]['name']
                    left_cam = cam_info['name']
                    epipolar_error_threshold = 0.7
                    if cam_info['name'] == 'rgb' or right_cam == 'rgb':  # TODO: Remove this check later
                        epipolar_error_threshold = 1

                    if cam_info['extrinsics']['epipolar_error'] > epipolar_error_threshold:
                        color = red
                        error_text.append("high epipolar error between " + left_cam + " and " + right_cam)
                    elif cam_info['extrinsics']['epipolar_error'] == -1:
                        color = red
                        error_text.append("Epiploar validation failed between " + left_cam + " and " + right_cam)

                    log_list.append(cam_info['extrinsics']['epipolar_error'])
                    text = left_cam + "-" + right_cam + ' Avg Epipolar error: ' + format(
                        cam_info['extrinsics']['epipolar_error'], '.6f')
                    pygame_render_text(self.screen, text, (vis_x, vis_y), color, 30)
                    vis_y += 30
                    specTranslation = np.array(
                        [cam_info['extrinsics']['specTranslation']['x'], cam_info['extrinsics']['specTranslation']['y'],
                         cam_info['extrinsics']['specTranslation']['z']], dtype=np.float32)

                    calibration_handler.setCameraExtrinsics(stringToCam[camera],
                                                            stringToCam[cam_info['extrinsics']['to_cam']],
                                                            cam_info['extrinsics']['rotation_matrix'],
                                                            cam_info['extrinsics']['translation'], specTranslation)
                    if result_config['stereo_config']['left_cam'] == camera and result_config['stereo_config'][
                        'right_cam'] == cam_info['extrinsics']['to_cam']:
                        calibration_handler.setStereoLeft(stringToCam[camera],
                                                          result_config['stereo_config']['rectification_left'])
                        calibration_handler.setStereoRight(stringToCam[cam_info['extrinsics']['to_cam']],
                                                           result_config['stereo_config']['rectification_right'])
            # else:
            # log_list.append("N/A")

        log_file = self.args['log_path'] + "/calibration_logs_" + self.args['board'] + ".csv"
        with open(log_file, mode='a') as log_fopen:
            # header =
            log_csv_writer = csv.writer(log_fopen, delimiter=',')
            log_csv_writer.writerow(log_list)
        # calibration_handler.setBoardInfo(self.board_config['name'], self.board_config['revision'])

        # self.socket_worker.ack()
        if len(error_text) == 0:
            print('Flashing Calibration data into ')
            print(calib_dest_path)
            calibration_handler.eepromToJsonFile(calib_dest_path)
            eepromDataJson = calibration_handler.eepromToJson()
            # DEPTHAI_ALLOW_FACTORY_FLASHING = os.environ.get('DEPTHAI_ALLOW_FACTORY_FLASHING')
            # self.socket_worker.send(self.socket_worker.DEPTHAI_ALLOW_FACTORY_FLASHING)
            self.socket_worker.send(eepromDataJson)
            if self.socket_worker.recv() == 'flashed':
                is_write_succesful = True
            else:
                is_write_succesful = False

            # self.close_device()
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

            # self.close_device()
            self.is_service_active = False
            return False, error_text[0]


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
    arg["traceLevel"] =  rospy.get_param("~traceLevel")
    arg["outputScaleFactor"] =  rospy.get_param("~outputScaleFactor")

    # Adding service names to arg
    arg["capture_service_name"] = rospy.get_param(
        '~capture_service_name')  # Add  capture_checkerboard to launch file
    arg["calibration_service_name"] = rospy.get_param(
        '~calibration_service_name')  # Add capture_checkerboard to launch file
    # arg['host'] = rospy.get_param('~host')

    if not os.path.exists(arg['calib_path']):
        os.makedirs(arg['calib_path'])

    if not os.path.exists(arg['log_path']):
        os.makedirs(arg['log_path'])

    if arg['board']:
        board_path = Path(arg['board'])
        # TODO(Daniel): remove this after updating testing for CM4
        global eepromDataJson
        if arg['board'].lower() == 'dm2097':
            print('Loading data for OAK-D-CM4-PoE')
            eepromDataJson = {"batchName": "", "batchTime": 0, "boardConf": "nIR-C00M00-00", "boardName": "DM2097",
                              "boardRev": "R1M1E1", "productName": "OAK-D-CM4-POE", "boardCustom": "",
                              "hardwareConf": "F0-FV00-BC000", "boardOptions": 0, "version": 7}
        if arg['board'].lower() == 'dm2097-c11':
            print('Loading data for OAK-D-CM4-PoE')
            eepromDataJson = {"batchName": "", "batchTime": 0, "boardConf": "nIR-C11M00-00", "boardName": "DM2097",
                              "boardRev": "R3M1E4", "productName": "OAK-D-CM4-POE-C11", "boardCustom": "",
                              "hardwareConf": "F1-FV00-BC000", "boardOptions": 0, "version": 7}
        if arg['board'].lower() == 'dm1097':
            print('Loading data for OAK-D-CM4')
            eepromDataJson = {"batchName": "Maxwell", "batchTime": 0, "boardConf": "nIR-C00M00-00",
                              "boardName": "DM1097", "boardRev": "R2M0E2", "productName": "OAK-D-CM4",
                              "boardCustom": "", "hardwareConf": "F0-FV00-BC000", "boardOptions": 0, "version": 7}

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
