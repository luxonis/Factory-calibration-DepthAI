# !/usr/bin/env python3

import pickle
import platform
import cv2
from pathlib import Path
import json
import socket
import struct
import os
import time
import depthai as dai
import numpy as np

# ip = '127.0.1.1'
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

device = None
focusSigmaThreshold = 25

def create_pipeline(board_config):
    pipeline = dai.Pipeline()

    for cam_id in board_config['cameras']:
        cam_info = board_config['cameras'][cam_id]
        if cam_info['type'] == 'mono':
            cam_node = pipeline.createMonoCamera()
            xout = pipeline.createXLinkOut()

            cam_node.setBoardSocket(stringToCam[cam_id])
            cam_node.setResolution(camToMonoRes[cam_info['sensorName']])
            cam_node.setFps(10)

            xout.setStreamName(cam_info['name'])
            cam_node.out.link(xout.input)
        else:
            cam_node = pipeline.createColorCamera()
            xout = pipeline.createXLinkOut()
            
            cam_node.setBoardSocket(stringToCam[cam_id])
            cam_node.setResolution(camToRgbRes[cam_info['sensorName']])
            cam_node.setFps(10)

            xout.setStreamName(cam_info['name'])
            cam_node.isp.link(xout.input)

            if cam_info['hasAutofocus']:
                controlIn = pipeline.createXLinkIn()
                controlIn.setStreamName(cam_info['name'] + '-control')
                controlIn.out.link(cam_node.inputControl)

    return pipeline


def check_ping(ip_test):
    response = os.system("ping -c 1 " + ip_test)
    # and then check the response...
    if response == 0:
        pingstatus = True
    else:
        pingstatus = False

    return pingstatus

if check_ping("10.42.0.1"):
    HOST = "10.42.0.1"
elif check_ping("10.42.0.2"):
    HOST = "10.42.0.2"

print("host is set to {}".format(HOST))

# HOST = ip  # The server's hostname or IP address
PORT = 51264        # The port used by the server
camera_queue = {}
control_queue = {}
aruco_dictionary = cv2.aruco.Dictionary_get(
                                cv2.aruco.DICT_4X4_1000)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))


device = dai.Device()

def cvt_bgr(packet):
    meta = packet.getMetadata()
    w = meta.getFrameWidth()
    h = meta.getFrameHeight()
    # print((h, w))
    packetData = packet.getData()
    yuv420p = packetData.reshape((h * 3 // 2, w))
    return cv2.cvtColor(yuv420p, cv2.COLOR_YUV2BGR_IYUV)


def capture_servive_handler():
    print("Capture image Service Started")
    # TODO(Sachin): Add time synchronization here and get the most recent frame instead.
    frameCount = 0
    detection_failed = False
    # while not finished:
    image_dict = {}

    for key in camera_queue.keys():
        frame = camera_queue[key].getAll()[-1]
        gray_frame = None
        if frame.getType() == dai.RawImgFrame.Type.RAW8:
            gray_frame = frame.getCvFrame()
        else:
            gray_frame = cv2.cvtColor(frame.getCvFrame(), cv2.COLOR_BGR2GRAY)

        image_dict[key] = gray_frame

    return image_dict


def write_eeprom(result_config):
    calibration_handler = dai.CalibrationHandler()
    for camera in result_config['cameras'].keys():
        cam_info = result_config['cameras'][camera]

        calibration_handler.setDistortionCoefficients(stringToCam[camera], cam_info['dist_coeff'])
        calibration_handler.setCameraIntrinsics(stringToCam[camera], cam_info['intrinsics'],  cam_info['size'][0], cam_info['size'][1])
        calibration_handler.setFov(stringToCam[camera], cam_info['hfov'])

        if cam_info['hasAutofocus']:
            calibration_handler.setLensPosition(stringToCam[camera], result_config['cameras'][camera]['lensPosition'])
        
        if 'extrinsics' in cam_info:
            
            if 'to_cam' in cam_info['extrinsics']:
                """right_cam = result_config['cameras'][cam_info['extrinsics']['to_cam']]['name']
                left_cam = cam_info['name']
                
                 if cam_info['extrinsics']['epipolar_error'] > 0.6:
                    color = red
                    error_text.append("high epipolar error between " + left_cam + " and " + right_cam)
                elif cam_info['extrinsics']['epipolar_error'] == -1:
                    color = red
                    error_text.append("Epiploar validation failed between " + left_cam + " and " + right_cam) """
                
                """ log_list.append(cam_info['extrinsics']['epipolar_error'])
                text = left_cam + "-" + right_cam + ' Avg Epipolar error: ' + format(cam_info['extrinsics']['epipolar_error'], '.6f')
                pygame_render_text(self.screen, text, (vis_x, vis_y), color, 30)
                vis_y += 30 """
                specTranslation = np.array([cam_info['extrinsics']['specTranslation']['x'], cam_info['extrinsics']['specTranslation']['y'], cam_info['extrinsics']['specTranslation']['z']], dtype=np.float32)

                calibration_handler.setCameraExtrinsics(stringToCam[camera], stringToCam[cam_info['extrinsics']['to_cam']], cam_info['extrinsics']['rotation_matrix'], cam_info['extrinsics']['translation'], specTranslation)
                if result_config['stereo_config']['left_cam'] == camera and result_config['stereo_config']['right_cam'] == cam_info['extrinsics']['to_cam']:
                    calibration_handler.setStereoLeft(stringToCam[camera], result_config['stereo_config']['rectification_left'])
                    calibration_handler.setStereoRight(stringToCam[cam_info['extrinsics']['to_cam']], result_config['stereo_config']['rectification_right'])

    calibration_handler.setBoardInfo(board_config['name'], board_config['revision'])
    try:
        is_write_succesful = device.flashCalibration(calibration_handler)
    except:
        print("Writing in except...")
        is_write_succesful = device.flashCalibration(calibration_handler)
    device.close()
    return is_write_succesful

def camera_focus_adjuster():
    maxCountFocus   = 50
    focusCount = {}
    isFocused = {}
    trigCount = {}
    capturedFrames = {}
    status = {}
    lensPosition = {}
    focusSigma = {}
    
    ctrl = dai.CameraControl()
    ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
    ctrl.setAutoFocusTrigger()

    for config_cam in board_config['cameras'].keys():
        cam_info = board_config['cameras'][config_cam]
        focusCount[cam_info['name']] = 0
        focusSigma[cam_info['name']] = 0
        lensPosition[cam_info['name']] = 0
        isFocused[config_cam] = False
        capturedFrames[cam_info['name']] = None

        if cam_info['hasAutofocus']:
            trigCount[cam_info['name']] = 0
            control_queue[cam_info['name']].send(ctrl)

    time.sleep(1)
    focusFailed  = False
    while True:
        for config_cam in board_config['cameras'].keys():
            cam_info = board_config['cameras'][config_cam]
            frame = camera_queue[cam_info['name']].getAll()[-1]
            currFrame = frame.getCvFrame()
            if frame.getType() != dai.RawImgFrame.Type.RAW8:
                currFrame = cv2.cvtColor(currFrame, cv2.COLOR_BGR2GRAY)
            print('Resolution: {}'.format(currFrame.shape))
            capturedFrames[cam_info['name']] = currFrame 

            if cam_info['hasAutofocus']:
                marker_corners, _, _ = cv2.aruco.detectMarkers(currFrame, aruco_dictionary)
                if len(marker_corners) < 30:
                    print("Board not detected. Waiting...!!!")
                    trigCount[cam_info['name']] += 1
                    focusCount[cam_info['name']] += 1
                    if trigCount[cam_info['name']] > 31:
                        trigCount[cam_info['name']] = 0
                        control_queue[cam_info['name']].send(ctrl)
                        time.sleep(1)
                    if focusCount[cam_info['name']] > maxCountFocus:
                        focusFailed = True
                        break
                    continue

            dst_laplace = cv2.Laplacian(currFrame, cv2.CV_64F)
            mu, sigma = cv2.meanStdDev(dst_laplace)

            print('Sigma of {} is {}'.format(cam_info['name'], sigma))
            localFocusThreshold = focusSigmaThreshold 
            if dst_laplace.shape[1] > 2000:
                localFocusThreshold = localFocusThreshold / 2

            if sigma > localFocusThreshold:
                isFocused[config_cam] = True
                focusSigma[cam_info['name']] = sigma
                if cam_info['hasAutofocus']:
                    lensPosition[cam_info['name']] = frame.getLensPosition()
            else:
                if cam_info['hasAutofocus']:
                    trigCount[cam_info['name']] += 1
                    if trigCount[cam_info['name']] > 31:
                        trigCount[cam_info['name']] = 0
                        control_queue[cam_info['name']].send(ctrl)
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
    status['focusSigma'] = focusSigma
    for key in isFocused.keys():
        cam_name = board_config['cameras'][key]['name']
        if isFocused[key]:
            if board_config['cameras'][key]['hasAutofocus']:
                ctrl = dai.CameraControl()
                ctrl.setManualFocus(lensPosition[cam_name])
                print("Sending manual focus Control to {}".format(cam_name))
                board_config['cameras'][key]['lensPosition'] = lensPosition[cam_name]
                status['lensPosition'][cam_name] = lensPosition[cam_name]
                control_queue[cam_name].send(ctrl)
            status[cam_name + "-Focus"] = True
            # self.auto_focus_checkbox_dict[cam_name + "-Focus"].check()
            # self.auto_focus_checkbox_dict[cam_name + "-Focus"].render_checkbox()
        else:
            status[cam_name + "-Focus"] = False

    return status

def device_status_handler(board_config):

    if device is not None:
        if not device.isClosed():
            device.close()
    
    dev_status = {}
    finished = False
    # while not finished:

    while device is None or device.isClosed():
        device = dai.Device() 
        # cameraList = self.device.getConnectedCameras()
        cameraProperties = device.getConnectedCameraProperties()
        time.sleep(2)
        # dev_info = self.device.getDeviceInfo()
        
        dev_status['mx_id'] = device.getMxId()
        lost_camera = False
        for properties in cameraProperties:
            for in_cam in board_config['cameras'].keys():
                cam_info = board_config['cameras'][in_cam]
                if properties.socket == stringToCam[in_cam]:
                    board_config['cameras'][in_cam]['sensorName'] = properties.sensorName
                    print('Cam: {} and focus: {}'.format(cam_info['name'], properties.hasAutofocus))
                    board_config['cameras'][in_cam]['hasAutofocus'] = properties.hasAutofocus
                    dev_status[cam_info['name']  + '-Camera-connected'] = True
                # else:
                #     lost_camera = True
                #     dev_status[cam_info['name']  + '-Camera-connected'] = False
                    
        for config_cam in board_config['cameras'].keys():
            cam_info = board_config['cameras'][config_cam]
            id = cam_info['name'] + '-Camera-connected'
            if not id in dev_status:
                dev_status[cam_info['name']  + '-Camera-connected'] = False
                lost_camera = True

        if not lost_camera:
            pipeline = create_pipeline(board_config)
            device.startPipeline(pipeline)
            camera_queue = {}
            control_queue = {}
            for config_cam in board_config['cameras']:
                cam = board_config['cameras'][config_cam]
                camera_queue[cam['name']] = device.getOutputQueue(cam['name'], 5, False)
                if cam['hasAutofocus']:
                    control_queue[cam['name']] = device.getInputQueue(cam['name'] + '-control', 5, False)
        else:
            print("Closing Device...")
            return dev_status

    mipi = {}
    for config_cam in board_config['cameras']:
        mipi[board_config['cameras'][config_cam]['name']] = False

    # left_mipi = False
    # right_mipi = False
    # rgb_mipi = False

    for _ in range(120):
        for config_cam in board_config['cameras']:
            name = board_config['cameras'][config_cam]['name']
            imageFrame = camera_queue[name].tryGet()
            
            if imageFrame is not None:
                mipi[name] = True
                """ frame = None

                if imageFrame.getType() == dai.RawImgFrame.Type.RAW8:
                    frame = imageFrame.getCvFrame()
                else:
                    frame = cv2.cvtColor(imageFrame.getCvFrame(), cv2.COLOR_BGR2GRAY) """
                # self.imgPublishers[name].publish(
                #     self.bridge.cv2_to_imgmsg(frame, "passthrough"))

        isMipiReady = True
        for config_cam in board_config['cameras']:
            name = board_config['cameras'][config_cam]['name']
            isMipiReady = isMipiReady and mipi[name] 
        if isMipiReady:
            break
        time.sleep(1)
    
    dev_status['mipi'] = mipi
    # for key in mipi.keys():
    #     dev_status[key + "-Stream"] = mipi[key]

    return dev_status
        
        
# dataset_path = Path(self.package_path + "/dataset")
# if 0 and dataset_path.exists():
#     shutil.rmtree(str(dataset_path))
# # dev_info = self.device.getDeviceInfo()
# self.is_service_active = False
# return (finished, self.device.getMxId())

board_config = None

while True:
    data = s.recv(1024)
    req = repr(data)
    if req == 'get_board_config':
        data = s.recv(1024)
        board_config = pickle.loads(recv_data)
    if req == 'status_handler':
        dev_status = device_status_handler(board_config)
        data = pickle.dumps(dev_status)
        s.sendall(data)
    elif req == 'check_focus_adjuster':
        focus_status = camera_focus_adjuster()
        data = pickle.dumps(focus_status)
        s.sendall(data)
    elif req == 'capture_req':
        img_data = capture_servive_handler()

        rec_img_data_bytes = pickle.dumps(img_data, 0)
        size = len(rec_img_data_bytes)
        print(size)
        s.sendall(struct.pack(">L", size) + rec_img_data_bytes)
        
    elif req == 'write_eeprom':
        recv_data = s.recv(1024)
        result_config = pickle.loads(recv_data)
        
        for camera in board_config['cameras'].keys():
            if board_config['cameras'][camera]['hasAutofocus']:
                result_config['cameras'][camera]['hasAutofocus'] = board_config['cameras'][camera]['hasAutofocus']
                result_config['cameras'][camera]['lensPosition'] = board_config['cameras'][camera]['lensPosition']
        eeprom_status = write_eeprom(result_config)
        data = pickle.dumps(eeprom_status)
        s.sendall(data)
