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
on_embedded = platform.machine().startswith('arm') or platform.machine().startswith('aarch64')

def find_chessboard(frame):
    chessboard_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
    small_frame = cv2.resize(frame, (0, 0), fx=0.3, fy=0.3)
    return cv2.findChessboardCorners(small_frame, (9, 6), chessboard_flags)[0] and \
           cv2.findChessboardCorners(frame, (9, 6), chessboard_flags)[0]

white  = [255, 255, 255]
orange = [143, 122, 4]
red    = [230, 9, 9]
green  = [4, 143, 7]
black  = [0, 0, 0]
pygame.init()

class depthai_calibration_node:
    def __init__(self, depthai_args):
        self.package_path = depthai_args['package_path']
        self.args = depthai_args
        self.bridge = CvBridge()
        self.is_service_active = False
        self.config = {
            'streams':
                ['left', 'right', 'meta_d2h'] if not on_embedded else
                [{'name': 'left', "max_fps": 30.0}, {'name': 'right', "max_fps": 30.0}, {'name': 'meta_d2h', "max_fps": 30.0}],
            'depth':
                {
                    'calibration_file': consts.resource_paths.calib_fpath,
                    'padding_factor': 0.3
                },
            'ai':
                {
                    'blob_file': consts.resource_paths.blob_fpath,
                    'blob_file_config': consts.resource_paths.blob_config_fpath,
                    'shaves' : 7,
                    'cmx_slices' : 7,
                    'NN_engines' : 1,
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
                    'mono':
                    {
                        # 1280x720, 1280x800, 640x400 (binning enabled)
                        'resolution_h': 800,
                        'fps': 30.0,
                    },
                },
        }

        if arg['board']:
            board_path = Path(arg['board'])
            if not board_path.exists():
                board_path = Path(consts.resource_paths.boards_dir_path) / Path(arg['board'].upper()).with_suffix('.json')
                print(board_path)
                if not board_path.exists():
                    raise ValueError('Board config not found: {}'.format(board_path))
            with open(board_path) as fp:
                self.board_config = json.load(fp)
        utils.merge(self.board_config, self.config)

        self.aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        # self.charuco_board = aruco.CharucoBoard_create(
        #                                     22,16,
        #                                     self.args['square_size_cm'],
        #                                     self.args['marker_size_cm'],
        #                                     self.aruco_dictionary)
        self.start_device()
        self.capture_srv = rospy.Service(self.args["capture_service_name"], Capture, self.capture_servive_handler)
        self.calib_srv = rospy.Service(self.args["calibration_service_name"], Capture, self.calibration_servive_handler)
        self.dev_status_srv = rospy.Service("device_status", Capture, self.device_status_handler)
        self.image_pub_left = rospy.Publisher("left",Image, queue_size=10)
        self.image_pub_right = rospy.Publisher("right",Image, queue_size=10)

        self.disp = pygame.display
        self.screen = self.disp.set_mode((800, 600))
        self.screen.fill(white)
        self.disp.set_caption("Calibration - Device check ")
        title = "Device Status"
        pygame_render_text(self.screen, title, (350,20), orange, 50)
        self.auto_checkbox_names = ["USB3", "Left camera connected", "Right camera connected", 
                                    "Left Stream", "Right Stream"]
        
        y = 110
        x = 200
        self.start_disp = False
        font = pygame.font.Font(None, 20)
        self.auto_checkbox_dict = {}
        for i in range(len(self.auto_checkbox_names)):
            w, h = font.size(self.auto_checkbox_names[i])
            x_axis = x - w
            y_axis = y +  (40*i)
            font_surf = font.render(self.auto_checkbox_names[i], True, green)
            self.screen.blit(font_surf, (x_axis,y_axis))
            self.auto_checkbox_dict[self.auto_checkbox_names[i]] = Checkbox(self.screen, x + 10, y_axis-5, outline_color=green, 
                                                        check_color=green, check=False)

        # text = 'call rosservice of device_status_handler to update the device status'
        for i in range(len(self.auto_checkbox_names)):
            self.auto_checkbox_dict[self.auto_checkbox_names[i]].render_checkbox()
        # self.disp.update()

    def start_device(self):
        self.device = depthai.Device('', False)
        self.pipeline = self.device.create_pipeline(self.config)
        self.mx_id = self.device.get_mx_id()
        
    def publisher(self):
        while not rospy.is_shutdown():
            if self.start_disp:
                pygame.event.pump()
                self.disp.update()
            # print("updating dis-----")
            if not self.is_service_active:
                # print("SERVICE NOT ACTIVE")
                if not hasattr(self, "pipeline"):
                    self.start_device()
                    # print("restarting device---->")
                _, data_list = self.pipeline.get_available_nnet_and_data_packets(True)
                for packet in data_list:    
                    # print("found packets:")
                    # print(packet.stream_name)
                    if packet.stream_name == "left":
                        recent_left = packet.getData()
                        # print(recent_left.shape)
                        self.image_pub_left.publish(self.bridge.cv2_to_imgmsg(recent_left, "passthrough"))
                    elif packet.stream_name == "right":
                        recent_right = packet.getData()
                        self.image_pub_right.publish(self.bridge.cv2_to_imgmsg(recent_right, "passthrough"))

    def parse_frame(self, frame, stream_name, file_name):
        file_name += '.png'
        # filename = image_filename(stream_name, self.current_polygon, self.images_captured)
        print(self.package_path + "/dataset/{}/{}".format(stream_name, file_name))
        ds_path = self.package_path + "/dataset/{}".format(stream_name)
        print(ds_path)
        if not os.path.exists(ds_path):
            os.makedirs(ds_path)

        cv2.imwrite(self.package_path + "/dataset/{}/{}".format(stream_name, file_name), frame)
        print("py: Saved image as: " + str(file_name))
        return True

    def is_markers_found(self, frame):
        # print(frame.shape)
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, _, _ = cv2.aruco.detectMarkers(frame, self.aruco_dictionary)
        return not (len(marker_corners) == 0)

    def capture_servive_handler(self, req):
        print("Capture image Service Started")
        recent_left = None
        recent_right = None
        finished = False
        self.is_service_active = True
        # now = rospy.get_rostime()
        while not finished:
            _, data_list = self.pipeline.get_available_nnet_and_data_packets()
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

            if recent_left is not None and recent_right is not None:
                finished = True

        # print("looping")
        # is_board_found_l = find_chessboard(recent_left)
        # is_board_found_r = find_chessboard(recent_right)
        # is_board_found_l = True
        # is_board_found_r = True
        is_board_found_l = self.is_markers_found(recent_left)
        is_board_found_r = self.is_markers_found(recent_right)
        if is_board_found_l and is_board_found_r:
            print("Found------------------------->")
            self.parse_frame(recent_left, "left", req.name)
            self.parse_frame(recent_right, "right", req.name)
        else:
            print("Not found--------------------->")
            self.is_service_active = False
            self.parse_frame(recent_left, "left_not", req.name)
            self.parse_frame(recent_right, "right_not", req.name)
            return (False, "Calibration board not found")
        # elif is_board_found_l and not is_board_found_r: ## TODO: Add errors after srv is built
        print("Service ending")
        self.is_service_active = False
        return (True, "No Error")
    
    def device_status_handler(self, req):
        self.is_service_active = True
        self.start_disp = True
        while not self.device.is_device_changed():
            text = "Waiting for device change"
            pygame_render_text(self.screen, text, (250,400), orange, 40)
        
        # to remove waiting for device change
        fill_color =  pygame.Rect(200, 400, 400, 55)
        pygame.draw.rect(self.screen, white, fill_color)

        # to remove previous date and stuff
        fill_color =  pygame.Rect(400, 70, 400, 545)
        pygame.draw.rect(self.screen, white, fill_color)
        now_time = datetime.datetime.now()
        text = "date/time : " + now_time.strftime("%m-%d-%Y %H:%M:%S")
        pygame_render_text(self.screen, text, (400,80), black, 30)
        text = "device Mx_id : " + self.device.get_mx_id()
        pygame_render_text(self.screen, text, (400,120), black, 30)

        while self.device.is_device_changed():
            print(self.device.is_device_changed())
            is_usb3 = self.device.is_usb3()
            left_status = self.device.is_left_connected()
            right_status = self.device.is_right_connected()
            left_mipi = False
            right_mipi = False
            
            if left_status and right_status:
                # mipi check using 20 iterations
                # ["USB3", "Left camera connected", "Right camera connected", "left Stream", "right Stream"]
                # time.sleep(1) # this is needed to avoid iterating fastly over 
                for _ in range(60):
                    _, data_list = self.pipeline.get_available_nnet_and_data_packets(True)
                    print(len(data_list))
                    for packet in data_list:    
                        # print("found packets:")
                        # print(packet.stream_name)
                        if packet.stream_name == "left":
                            recent_left = packet.getData()
                            left_mipi = True
                            self.image_pub_left.publish(self.bridge.cv2_to_imgmsg(recent_left, "passthrough"))
                        elif packet.stream_name == "right":
                            recent_right = packet.getData()
                            right_mipi = True
                            self.image_pub_right.publish(self.bridge.cv2_to_imgmsg(recent_right, "passthrough"))
                    if left_mipi and right_mipi:
                        if is_usb3:
                            self.device.reset_device_changed()
                        # for i in range(len(self.auto_checkbox_names)):
                        #     self.auto_checkbox_dict[self.auto_checkbox_names[i]].render_checkbox()
                        # print("device reste")
                        # print(self.device.is_device_changed())
                        break

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

            for i in range(len(self.auto_checkbox_names)):
                self.auto_checkbox_dict[self.auto_checkbox_names[i]].render_checkbox()

        self.is_service_active = False
        return (True, self.device.get_mx_id())


    def calibration_servive_handler(self, req):
        self.is_service_active = True
        print("Capture image Service Started")

        mx_serial_id = self.device.get_mx_id()
        calib_dest_path = os.path.join(arg['calib_path'], arg["board"] + '_' + mx_serial_id + '.calib')

        flags = [self.config['board_config']['stereo_center_crop']]
        cal_data = StereoCalibration()
        avg_epipolar_error, calib_data = cal_data.calibrate(
                            self.package_path + "/dataset",
                            self.args['square_size_cm'],
                            calib_dest_path, 
                            flags, 
                            req.name, 
                            self.args['marker_size_cm'])

        text = "Avg Epipolar error : " + format(avg_epipolar_error, '.6f')
        pygame_render_text(self.screen, text, (400,160), green, 30)
        if avg_epipolar_error > 0.5:
            text = "Failed use to high calibration error"
            pygame_render_text(self.screen, text, (400,200), red, 30)
            return (False, "Failed use to high calibration error")
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
        dev_config["_board"]['mesh_left'] =  [0.0]        

        self.device.write_eeprom_data(dev_config)

        # calib_src_path = os.path.join(arg['depthai_path'], "resources/depthai.calib")
        # shutil.copy(calib_src_path, calib_dest_path)
        print("finished writing to EEPROM with Epipolar error of")

        print(avg_epipolar_error)
        print('Validating...')
        is_write_succesful = False
        run_thread = True
        while run_thread:
            _, data_packets = self.pipeline.get_available_nnet_and_data_packets(blocking=True)
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
                                    pygame_render_text(self.screen, text, (400,200), green, 30)
                                    is_write_succesful = True
                                    run_thread = False
                                elif 'FAILED' in log:
                                    text = "EEPROM write Failed"
                                    pygame_render_text(self.screen, text, (400,200), red, 30)
                                    is_write_succesful = False
                                    run_thread = False
        self.is_service_active = False
        if is_write_succesful:
            return (True, "EEPROM written succesfully")
        else:
            return (False, "EEPROM write Failed!!")

    def rundepthai(self):
        test_cmd = "python3 " + self.args['depthai_path'] + "/depthai_demo.py -e -brd " + self.args['brd']
        print(test_cmd)
        # test_cmd = """python3 depthai.py -brd bw1098obc -e""" ## TODO(sachin): Parameterize the board name
        self.p = subprocess.Popen(test_cmd, shell=True, preexec_fn=os.setsid)
        time.sleep(4)
        os.killpg(os.getpgid(self.p.pid), signal.SIGKILL)

        time.sleep(1)    
        
    
if __name__ == "__main__":
    
    rospy.init_node('depthai_calibration', anonymous=True)
    arg = {}
    arg["swap_lr"] = rospy.get_param('~swap_lr')
    arg["field_of_view"] = rospy.get_param('~field_of_view')
    arg["baseline"] = rospy.get_param('~baseline')
    arg["package_path"] = rospy.get_param('~package_path')
    arg["square_size_cm"] = rospy.get_param('~square_size_cm')
    arg["marker_size_cm"] = rospy.get_param('~marker_size_cm')

    arg["depthai_path"] = rospy.get_param('~depthai_path') ## Add  capture_checkerboard to launch file
    arg["board"] = rospy.get_param('~brd') ## Add  capture_checkerboard to launch file
    arg["capture_service_name"] = rospy.get_param('~capture_service_name') ## Add  capture_checkerboard to launch file
    arg["calibration_service_name"] = rospy.get_param('~calibration_service_name') ## Add  capture_checkerboard to launch file
    arg["depthai_path"] = rospy.get_param('~depthai_path') ## Path of depthai repo
 
    arg["calib_path"] = rospy.get_param('~calib_path') ## local path to store calib files with using mx device id.

    if not os.path.exists(arg['calib_path']):
        os.makedirs(arg['calib_path'])
    
    if arg['board']:
        board_path = Path(arg['board'])
        if not board_path.exists():
            board_path = Path(consts.resource_paths.boards_dir_path) / Path(arg['board'].upper()).with_suffix('.json')
            print(board_path)
            if not board_path.exists():
                raise ValueError('Board config not found: {}'.format(board_path))
        with open(board_path) as fp:
            board_config = json.load(fp)
    assert os.path.exists(arg['depthai_path']), (arg['depthai_path'] +" Doesn't exist. \
        Please add the correct path using depthai_path:=[path] while executing launchfile")

    depthai_dev = depthai_calibration_node(arg)
    depthai_dev.publisher()
    rospy.spin()
