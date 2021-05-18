#!/usr/bin/env python3

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
        self.focus_value = 82

        pipeline = self.create_pipeline()
        self.device = dai.Device(pipeline)
        # self.device.startPipeline()

        self.left_camera_queue  = self.device.getOutputQueue("left", 5, False)
        self.right_camera_queue = self.device.getOutputQueue("right", 5, False)
        self.rgb_camera_queue   = self.device.getOutputQueue("rgb", 5, False)
        print("board_path")

        # self.frame_count = 0
        self.init_time = time.time()
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

        self.aruco_dictionary = cv2.aruco.Dictionary_get(
            cv2.aruco.DICT_4X4_1000)

        # Connection checks ----------->
        self.disp = pygame.display
        self.screen = self.disp.set_mode((800, 600))
        self.screen.fill(white)
        self.disp.set_caption("Calibration - Device check ")
        title = "Device Status"
        pygame_render_text(self.screen, title, (350, 20), orange, 50)
        self.auto_checkbox_names = ["Left Stream", "RGB Stream"]
        header = ['time', 'Mx_serial_id',
                  'left_camera', 'rgb_camera', 'Epipolar error L-Rgb']

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
        # self.rgb_focus_srv = rospy.Service(
        #     "set_rgb_focus", Capture, self.rgb_focus_handler)

        self.args['cameraModel'] = 'perspective'
        self.image_pub_left = rospy.Publisher("left", Image, queue_size=10)
        self.image_pub_right = rospy.Publisher("right", Image, queue_size=10)
        self.image_pub_color = rospy.Publisher("color", Image, queue_size=10)

    def create_pipeline(self):
        pipeline = dai.Pipeline()

        rgb_cam  = pipeline.createColorCamera()
        cam_left = pipeline.createMonoCamera()
        cam_right = pipeline.createMonoCamera()

        xout_left     = pipeline.createXLinkOut()
        xout_right    = pipeline.createXLinkOut()
        xout_rgb_isp  = pipeline.createXLinkOut()

        rgb_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)
        rgb_cam.setInterleaved(False)
        rgb_cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        rgb_cam.setIspScale(1, 3)
        rgb_cam.initialControl.setManualFocus(self.focus_value)
        # rgb_cam.initialControl.setManualFocus(135)
        # rgb_cam.setImageOrientation(dai.CameraImageOrientation.ROTATE_180_DEG)

        cam_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        cam_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)

        cam_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        cam_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
        # cam_left.setImageOrientation(dai.CameraImageOrientation.ROTATE_180_DEG)

        xout_left.setStreamName("left")
        cam_left.out.link(xout_left.input)

        xout_right.setStreamName("right")
        cam_right.out.link(xout_right.input)

        xout_rgb_isp.setStreamName("rgb")
        rgb_cam.isp.link(xout_rgb_isp.input)
        # rgb_cam.isp.link(xout_rgb_isp.input)
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

            if not self.is_service_active and not self.device.isClosed():                
                left_frame = self.left_camera_queue.tryGet()
                if left_frame is not None:
                    self.image_pub_left.publish(
                        self.bridge.cv2_to_imgmsg(left_frame.getCvFrame(), "passthrough"))
                
                right_frame = self.right_camera_queue.tryGet()
                if right_frame is not None:
                    self.image_pub_left.publish(
                        self.bridge.cv2_to_imgmsg(right_frame.getCvFrame(), "passthrough"))
                
                rgb_frame = self.rgb_camera_queue.tryGet()
                if rgb_frame is not None:
                    frame = cv2.cvtColor(rgb_frame.getCvFrame(), cv2.COLOR_BGR2GRAY) 
                    # cv2.imwrite("/home/sachin/test.png", frame)
                    self.image_pub_color.publish(
                        self.bridge.cv2_to_imgmsg(frame, "passthrough"))
                    # cv2.imshow("self.", rgb_frame.getCvFrame())

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

        ## adding backup
        now_tim = datetime.now()
        self.backup_ds(stream_name, file_name, frame)    
        return True

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

    def device_status_handler(self, req):
        self.is_service_active = True
        self.start_disp = True
        # pygame.draw.rect(self.screen, white, no_button)

        # if req.name == "recalibrate":
        #     self.device.override_device_changed()
        finished = False
        while self.device.isClosed():
            print("device_closed")
            # TODO(Sachin): add a check for available devices..
            pipeline = self.create_pipeline()
            self.device = dai.Device(pipeline) 
            self.device.startPipeline()
            self.left_camera_queue = self.device.getOutputQueue("left", 5, False)
            self.right_camera_queue = self.device.getOutputQueue("right", 5, False)
            self.rgb_camera_queue  = self.device.getOutputQueue("rgb", 5, False)
            # rospy.sleep(4)

        # while not self.device.is_device_changed():
        #     text = "Waiting for device change"
        #     pygame_render_text(self.screen, text, (300, 400), orange, 40)

        # to remove waiting for device change
        fill_color = pygame.Rect(280, 400, 450, 55)
        pygame.draw.rect(self.screen, white, fill_color)

        # to remove previous date and stuff
        fill_color = pygame.Rect(400, 70, 500, 300)
        pygame.draw.rect(self.screen, white, fill_color)
        now_time = datetime.now()
        text = "date/time : " + now_time.strftime("%m-%d-%Y %H:%M:%S")
        pygame_render_text(self.screen, text, (400, 80), black, 30)

        dev_info = self.device.getDeviceInfo()
        text = "device Mx_id : " + dev_info.getMxId()
        pygame_render_text(self.screen, text, (400, 120), black, 30)
        rospy.sleep(1)
        fill_color_2 = pygame.Rect(50, 520, 400, 80)
        pygame.draw.rect(self.screen, white, fill_color_2)


        dataset_path = Path(self.package_path + "/dataset")
        if 0 and dataset_path.exists():
            shutil.rmtree(str(dataset_path))

        while not finished:
            # print(self.device.is_device_changed())
            if self.capture_exit():
                print("signaling...")
                rospy.signal_shutdown("Finished calibration")
            # is_usb3 = False
            left_mipi = False
            right_mipi = False
            rgb_mipi = False

            # is_usb3 = self.device.is_usb3()
            left_status = True
            # right_status = self.device.is_right_connected()
            rgb_status = True
            if self.capture_exit():
                rospy.signal_shutdown("Finished calibration")

            # else
            if left_status and rgb_status:
                # mipi check using 20 iterations
                # ["USB3", "Left camera connected", "Right camera connected", "left Stream", "right Stream"]
                for _ in range(90):
                    left_frame = self.left_camera_queue.get()
                    if left_frame is not None:
                        left_mipi = True
                        self.image_pub_left.publish(
                            self.bridge.cv2_to_imgmsg(left_frame.getCvFrame(), "passthrough"))
                        
                    rgb_frame = self.rgb_camera_queue.get()
                    if rgb_frame is not None:
                        rgb_mipi = True
                        frame = cv2.cvtColor(rgb_frame.getCvFrame(), cv2.COLOR_BGR2GRAY) 
                        self.image_pub_color.publish(
                            self.bridge.cv2_to_imgmsg(frame, "passthrough"))
                    if left_mipi and rgb_mipi:
                        finished = True

            if not left_mipi:
                self.auto_checkbox_dict["Left Stream"].uncheck()
            else:
                print("Setting left_mipi to true")
                self.auto_checkbox_dict["Left Stream"].check()

            if not rgb_mipi:
                self.auto_checkbox_dict["RGB Stream"].uncheck()
            else:
                print("Setting left_mipi to true")
                self.auto_checkbox_dict["RGB Stream"].check()

            for i in range(len(self.auto_checkbox_names)):
                self.auto_checkbox_dict[self.auto_checkbox_names[i]].render_checkbox()

        # self.set_focus()
        # rospy.sleep(2)
        self.is_service_active = False
        return (True, dev_info.getMxId())


    def capture_servive_handler(self, req):
        print("Capture image Service Started")
        recent_left = None
        recent_right = None
        recent_color = None
        finished = False
        self.is_service_active = True
        rospy.sleep(1)

        rgb_check_count = 0
        m_d2h_seq_focus = dict()
        # color_pkt_queue = deque()
        local_color_frame_count = 0
        # TODO(Sachin): Add time synchronization here and get the most recent frame instead.
        while not finished:
            # left_frame = self.left_camera_queue.get()
            left_frame = self.left_camera_queue.getAll()[-1]
            self.image_pub_left.publish(
                        self.bridge.cv2_to_imgmsg(left_frame.getCvFrame(), "passthrough"))

            # right_frame = self.right_camera_queue.get()
            right_frame = self.right_camera_queue.getAll()[-1]
            self.image_pub_right.publish(
                        self.bridge.cv2_to_imgmsg(right_frame.getCvFrame(), "passthrough"))
                
            # rgb_frame = self.rgb_camera_queue.get()
            rgb_frame = self.rgb_camera_queue.getAll()[-1]
            recent_color = cv2.cvtColor(rgb_frame.getCvFrame(), cv2.COLOR_BGR2GRAY)
            self.image_pub_color.publish(
                    self.bridge.cv2_to_imgmsg(recent_color, "passthrough"))

            recent_left = left_frame.getCvFrame()
            recent_right = right_frame.getCvFrame()

            if recent_left is not None and recent_right is not None and recent_color is not None:
                finished = True

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
        avg_epipolar_error_lr, avg_epipolar_error_r_rgb, calib_data = stereo_calib.calibrate(
            self.package_path + "/dataset",
            self.args['square_size_cm'],
            self.args['marker_size_cm'],
            self.args['squares_x'],
            self.args['squares_y'],
            self.args['cameraModel'],
            True, # turn on rgb calibration
            False) # Turn off enable disp rectify
 
        start_time = datetime.now()
        time_stmp = start_time.strftime("%m-%d-%Y %H:%M:%S")

        log_list = [time_stmp, mx_serial_id]

        log_list.append(True)
        log_list.append(True)

        log_list.append(avg_epipolar_error_lr)
        log_list.append(avg_epipolar_error_r_rgb)

        log_file = self.args['log_path'] + "/calibration_logs.csv"
        with open(log_file, mode='a') as log_fopen:
            # header =
            log_csv_writer = csv.writer(log_fopen, delimiter=',')
            log_csv_writer.writerow(log_list)

        def print_epipolar_error(color):
            text = "Avg Epipolar error L-R: " + \
                format(avg_epipolar_error_lr, '.6f')
            pygame_render_text(self.screen, text, (400, 160), color, 30)
            text = "Avg Epipolar error RGB-R: " + \
                format(avg_epipolar_error_r_rgb, '.6f')
            pygame_render_text(self.screen, text, (400, 190), color, 30)

        if avg_epipolar_error_lr > 0.5:
            text = "Failed due to high calibration error L-R"
            pygame_render_text(self.screen, text, (400, 230), red, 30)
            print_epipolar_error(red)
            return (False, text)

        if avg_epipolar_error_r_rgb > 0.7:
            text = "Failed due to high calibration error RGB-R"
            pygame_render_text(self.screen, text, (400, 270), red, 30)
            print_epipolar_error(red)
            return (False, text)

        calibration_handler = dai.CalibrationHandler()
        calibration_handler.setBoardInfo(self.board_config['board_config']['swap_left_and_right_cameras'], self.board_config['board_config']['name'], self.board_config['board_config']['revision'])
        
        calibration_handler.setStereoLeft(dai.CameraBoardSocket.LEFT, calib_data[0])
        calibration_handler.setStereoRight(dai.CameraBoardSocket.RGB, calib_data[1])
        
        calibration_handler.setCameraIntrinsics(dai.CameraBoardSocket.LEFT, calib_data[2], 1280, 800)
        calibration_handler.setCameraIntrinsics(dai.CameraBoardSocket.RIGHT, calib_data[3], 1280, 800)
        calibration_handler.setCameraIntrinsics(dai.CameraBoardSocket.RGB, calib_data[4], 1920, 1080)

        measuredTranslation = [self.board_config['board_config']['left_to_right_distance_cm'], 0.0, 0.0]
        calibration_handler.setCameraExtrinsics(dai.CameraBoardSocket.LEFT, dai.CameraBoardSocket.RIGHT, calib_data[5], calib_data[6], measuredTranslation)

        measuredTranslation = [self.board_config['board_config']['left_to_rgb_distance_cm'], 0.0, 0.0]
        calibration_handler.setCameraExtrinsics(dai.CameraBoardSocket.RIGHT, dai.CameraBoardSocket.RGB, calib_data[7], calib_data[8], measuredTranslation)


        calibration_handler.setDistortionCoefficients(dai.CameraBoardSocket.LEFT, calib_data[9])
        calibration_handler.setDistortionCoefficients(dai.CameraBoardSocket.RIGHT, calib_data[10])
        calibration_handler.setDistortionCoefficients(dai.CameraBoardSocket.RGB, calib_data[11])

        calibration_handler.setFov(dai.CameraBoardSocket.LEFT, self.board_config['board_config']['left_fov_deg'])
        calibration_handler.setFov(dai.CameraBoardSocket.RIGHT, self.board_config['board_config']['left_fov_deg'])
        calibration_handler.setFov(dai.CameraBoardSocket.RGB, self.board_config['board_config']['rgb_fov_deg'])

        calibration_handler.setLensPosition(dai.CameraBoardSocket.RGB, self.focus_value)

        calibration_handler.eepromToJsonFile(calib_dest_path)
        try:
            is_write_succesful = self.device.flashCalibration(calibration_handler)
        except:
            print("Writing in except...")
            is_write_succesful = self.device.flashCalibration(calibration_handler)
        
        # calib_src_path = os.path.join(arg['depthai_path'], "resources/depthai.calib")
        # shutil.copy(calib_src_path, calib_dest_path)
        print("finished writing to EEPROM with Epipolar error of")
        print(avg_epipolar_error_lr)
        print(avg_epipolar_error_r_rgb)
        
        self.is_service_active = False
        self.device.close()
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
    arg["swap_lr"] = rospy.get_param('~swap_lr')
    arg["field_of_view"] = rospy.get_param('~field_of_view')
    arg["baseline"] = rospy.get_param('~baseline')
    arg["package_path"] = rospy.get_param('~package_path')

    arg["square_size_cm"] = rospy.get_param('~square_size_cm')
    arg["marker_size_cm"] = rospy.get_param('~marker_size_cm')
    arg["squares_x"] = rospy.get_param('~squares_x')
    arg["squares_y"] = rospy.get_param('~squares_y')

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

    depthai_dev = depthai_calibration_node(arg)
    depthai_dev.publisher()
    rospy.spin()
