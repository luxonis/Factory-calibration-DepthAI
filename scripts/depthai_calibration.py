#!/usr/bin/env python

import cv2
import sys
import copy
import rospy
import geometry_msgs.msg
from std_msgs.msg import String
import depthai
import platform

from calibration.srv import Capture
from depthai_helpers.calibration_utils import *
from depthai_helpers import utils
import time
import numpy as np
import os
from pathlib import Path
import shutil
import consts.resource_paths
import json


on_embedded = platform.machine().startswith('arm') or platform.machine().startswith('aarch64')

def find_chessboard(frame):
    chessboard_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
    small_frame = cv2.resize(frame, (0, 0), fx=0.3, fy=0.3)
    return cv2.findChessboardCorners(small_frame, (9, 6), chessboard_flags)[0] and \
           cv2.findChessboardCorners(frame, (9, 6), chessboard_flags)[0]


class depthai_calibration_node:
    def __init__(self, depthai_args, service_name):
        self.abs_path = depthai_args['path']
        self.args = depthai_args
        self.config = {
            'streams':
                ['left', 'right'] if not on_embedded else
                [{'name': 'left', "max_fps": 10.0}, {'name': 'right', "max_fps": 10.0}],
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
                    'override_eeprom': True,
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

        self.device = depthai.Device('', False)
        self.pipeline = self.device.create_pipeline(self.config)
        self.capture_srv = rospy.Service(service_name, Capture, self.capture_servive)


    def parse_frame(self, frame, stream_name, file_name):
        # if not find_chessboard(frame):
        #     return False
        file_name += '.png'
        # filename = image_filename(stream_name, self.current_polygon, self.images_captured)
        print(self.abs_path + "/dataset/{}/{}".format(stream_name, file_name))
        cv2.imwrite(self.abs_path + "/dataset/{}/{}".format(stream_name, file_name), frame)
        print("py: Saved image as: " + str(file_name))
        return True

    def capture_servive(self, req):
        print("Service Started")
        recent_left = None
        recent_right = None
        finished = False
        while not finished:
            _, data_list = self.pipeline.get_available_nnet_and_data_packets()

            for packet in data_list:
                if packet.stream_name == "left":
                    recent_left = packet.getData()
                elif packet.stream_name == "right":
                    recent_right = packet.getData()
            
            if recent_left is not None and recent_right is not None:
                finished = True

        # is_board_found_l = find_chessboard(recent_left)
        # is_board_found_r = find_chessboard(recent_right)
        is_board_found_l = True
        is_board_found_r = True

        if is_board_found_l and is_board_found_r:
            self.parse_frame(recent_left, "left", req.name)
            self.parse_frame(recent_right, "right", req.name)
        # elif is_board_found_l and not is_board_found_r: ## TODO: Add errors after srv is built
        print("Service ending")
        return (True, "No Error")
            

        

                

if __name__ == "__main__":
    
    rospy.init_node('depthai_calibration', anonymous=True)
    arg = {}
    arg["swap_lr"] = rospy.get_param('~swap_lr')
    arg["field_of_view"] = rospy.get_param('~field_of_view')
    arg["baseline"] = rospy.get_param('~baseline')
    arg["path"] = rospy.get_param('~path')

    depthai = depthai_calibration_node(arg, "capture_checkerboard_srv")

    rospy.spin()