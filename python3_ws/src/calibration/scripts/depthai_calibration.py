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

from depthai_helpers import utils
on_embedded = platform.machine().startswith('arm') or platform.machine().startswith('aarch64')

def find_chessboard(frame):
    chessboard_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
    small_frame = cv2.resize(frame, (0, 0), fx=0.3, fy=0.3)
    return cv2.findChessboardCorners(small_frame, (9, 6), chessboard_flags)[0] and \
           cv2.findChessboardCorners(frame, (9, 6), chessboard_flags)[0]


class depthai_calibration_node:
    def __init__(self, depthai_args):
        self.package_path = depthai_args['package_path']
        self.args = depthai_args
        self.bridge = CvBridge()
        self.is_service_active = False
        self.config = {
            'streams':
                ['left', 'right'] if not on_embedded else
                [{'name': 'left', "max_fps": 30.0}, {'name': 'right', "max_fps": 30.0}],
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
        
        self.start_device()
        self.capture_srv = rospy.Service(self.args["capture_service_name"], Capture, self.capture_servive_handler)
        self.calib_srv = rospy.Service(self.args["calibration_service_name"], Capture, self.calibration_servive_handler)
        self.image_pub_left = rospy.Publisher("left",Image, queue_size=10)
        self.image_pub_right = rospy.Publisher("right",Image, queue_size=10)

    def start_device(self):
        self.device = depthai.Device('', False)
        self.pipeline = self.device.create_pipeline(self.config)
        

    def publisher(self):
        while not rospy.is_shutdown():
            if not self.is_service_active:
                # print("SERVICE NOT ACTIVE")
                if not hasattr(self, "pipeline"):
                    self.start_device()
                    # print("restarting device---->")
                _, data_list = self.pipeline.get_available_nnet_and_data_packets()
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
        is_board_found_l = True
        is_board_found_r = True
        if is_board_found_l and is_board_found_r:
            print("Found------------------------->")
        else:
            print("Not found--------------------->")
        self.parse_frame(recent_left, "left", req.name)
        self.parse_frame(recent_right, "right", req.name)
        # elif is_board_found_l and not is_board_found_r: ## TODO: Add errors after srv is built
        print("Service ending")
        self.is_service_active = False
        return (True, "No Error")
            
    def calibration_servive_handler(self, req):
        self.is_service_active = True
        print("caalibrate image Service Started")
        # if hasattr(self, "pipeline"):
        #     print("Removing")
        #     try:
        #         del self.pipeline
        #         del self.device
        #     except:
        #         print("catching")
        print("starting calinratin-- ->")
        
        flags = [self.config['board_config']['stereo_center_crop']]
        cal_data = StereoCalibration()
        print("starting calinratin")
        avg_epipolar_error = cal_data.calibrate(
                            self.package_path + "/dataset",
                            self.args['square_size_cm'],
                            self.args['depthai_path'] + "/resources/depthai.calib", 
                            flags, 
                            req.name, 
                            self.args['marker_size_cm'])

        if avg_epipolar_error > 0.5:
            return (False, "Failed use to high calibration error")
        # self.rundepthai()
        mx_serial_id = self.device.get_mx_id()
        calib_src_path = os.path.join(arg['depthai_path'], "resources/depthai.calib")
        calib_dest_path = os.path.join(arg['calib_path'], "obc_" + mx_serial_id + ".calib")
        shutil.copy(calib_src_path, calib_dest_path)
        print("finished writing to EEPROM with Epipolar error of")
        print(avg_epipolar_error)
        self.is_service_active = False
        return (True, "EEPROM written succesfully")

    def rundepthai(self):
        test_cmd = "python3 " + self.args['depthai_path'] + "/depthai_demo.py -e -brd " + self.args['brd']
        print(test_cmd)
        # test_cmd = """python3 depthai.py -brd bw1098obc -e""" ## TODO(sachin): Parameterize the board name
        self.p = subprocess.Popen(test_cmd, shell=True, preexec_fn=os.setsid)
        time.sleep(4)
        os.killpg(os.getpgid(self.p.pid), signal.SIGTERM)

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

    arg["depthai_path"] = rospy.get_param('~depthai_path') ## Path of depthai repo
    arg["calib_path"] = rospy.get_param('~calib_path') ## local path to store calib files with using mx device id.

    arg["brd"] = rospy.get_param('~brd') ## board name (Mostly of no use in future)
    arg["capture_service_name"] = rospy.get_param('~capture_service_name') ## get service capture_checkerboard from launch file
    arg["calibration_service_name"] = rospy.get_param('~calibration_service_name') ## get calibration service from launch file

    assert os.path.exists(arg['depthai_path']), (arg['depthai_path'] +" Doesn't exist. \
        Please add the correct path using depthai_path:=[path] while executing launchfile")

    depthai_dev = depthai_calibration_node(arg)
    depthai_dev.publisher()
    rospy.spin()
