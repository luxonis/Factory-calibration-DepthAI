import socket, pickle
import depthai as dai
import cv2
from datetime import timedelta
import time

stringToCam = {
                'RGB'   : dai.CameraBoardSocket.RGB,
                'LEFT'  : dai.CameraBoardSocket.LEFT,
                'RIGHT' : dai.CameraBoardSocket.RIGHT,
                'CAM_A' : dai.CameraBoardSocket.RGB,
                'CAM_B' : dai.CameraBoardSocket.LEFT,
                'CAM_C' : dai.CameraBoardSocket.RIGHT,
                # 'CAM_D' : dai.CameraBoardSocket.CAM_D,
                # 'CAM_E' : dai.CameraBoardSocket.CAM_E,
                # 'CAM_F' : dai.CameraBoardSocket.CAM_F,
                # 'CAM_G' : dai.CameraBoardSocket.CAM_G,
                # 'CAM_H' : dai.CameraBoardSocket.CAM_H
}

camToRgbRes = {
                'IMX378' : dai.ColorCameraProperties.SensorResolution.THE_4_K,
                'IMX214' : dai.ColorCameraProperties.SensorResolution.THE_4_K,
                'OV9282' : dai.ColorCameraProperties.SensorResolution.THE_1080_P,
                }

camToMonoRes = {
                'OV7251' : dai.MonoCameraProperties.SensorResolution.THE_480_P,
                'OV9282' : dai.MonoCameraProperties.SensorResolution.THE_800_P,
                }


class SocketWorker:
    def __init__(self):
        # self.once = True
        self.port = 51010
        self.recv_connection()
        # self.send_connection()

    def recv_connection(self):
        HOST = "luxonis.local"
        # HOST = "192.168.1.5"
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # if self.once:
        #     s.allow_reuse_address = True
        #     s.bind()
        # else:
        s.bind((HOST, self.port))
        self.port+= 1
        print(f'self.PORT={self.port}')
        s.listen()
        conn, addr = s.accept()
        self.recv_conn = conn
        self.send_conn = self.recv_conn
        print(f'Connected by {addr}')
        # self.once = False

    # def send_connection(self):
        # CHOST = '192.168.1.3'
        # CPORT = 5101
        # cs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # cs.connect((CHOST, CPORT))
        # self.send_conn = self.recv_conn

    def send(self, msg):
        try:
            data_string = pickle.dumps(msg)
            size = len(data_string)
            print(f'size={size}')
            self.send_conn.send(pickle.dumps(size))
            time.sleep(0.055)
            self.join()
            self.send_conn.sendall(data_string)
            print(f'send_msg={msg}')
            self.join()
        except Exception as e:
            print(e)
            
            # self.send_conn.shutdown(1)
            self.send_conn.close()
            self.recv_connection()
            self.send(msg)

    def recv(self):
        try:
            data = self.recv_conn.recv(4096)
            msg = pickle.loads(data)
            time.sleep(0.1)
            self.ack()
            print(f'msg={msg}')
            return msg
        except Exception as e:
            print(e)
            # self.recv_conn.shutdown()
            self.recv_conn.close()
            self.recv_connection()
            self.recv()

    def join(self):
        msg = pickle.loads(self.recv_conn.recv(4096))
        if not (msg == 'ACK'):
            raise RuntimeError(f'ACK error msg={msg}')

    def ack(self):
        msg = pickle.dumps('ACK')
        # self.send_conn.send(len(msg))
        self.send_conn.send(msg)
    
    def __del__(self):
        if hasattr(self, 'recv_conn'):
            self.recv_conn.close()
        if hasattr(self, 'send_conn'):
            self.send_conn.close()


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


class DepthaiCamera:
    def __init__(self, board_config, socket_worker):
        self.board_config = board_config
        self.socket_worker = socket_worker
        self.device = None
        self.start_camera()

    def start_camera(self):
        if self.device is not None:
            if not self.device.isClosed():
                self.device.close()
        # self.socket_worker.ack()

        while self.device is None or self.device.isClosed():
            self.socket_worker.send('not_connected')
            searchTime = timedelta(seconds=80)
            isFound, deviceInfo = dai.Device.getAnyAvailableDevice(searchTime)
            if not isFound:
                self.socket_worker.send('not_found')
            else:
                self.socket_worker.send('is_found')
                self.device = dai.Device()
                cameraProperties = self.device.getCameraSensorNames()

                # rospy.sleep(2)
                text = "device Mx_id : " + self.device.getMxId()
                self.socket_worker.send(text)
                # self.socket_worker.join()

                # text = "Device Connected!!!"
                lost_camera = False
                for properties in cameraProperties:
                    self.socket_worker.send('new_property')
                    for in_cam in self.board_config['cameras'].keys():
                        cam_info = self.board_config['cameras'][in_cam]
                        print(f'stringToCam[in_cam]={stringToCam[in_cam]}')
                        if properties == stringToCam[in_cam]:
                            self.board_config['cameras'][in_cam]['sensorName'] = cameraProperties[properties]
                            # focus = False
                            # if in_cam == 'RGB':
                            #     focus = True
                            focus = True
                            print('Cam: {} and focus: {}'.format(cam_info['name'], focus))
                            self.board_config['cameras'][in_cam]['hasAutofocus'] = focus
                            self.socket_worker.send('checked')
                            # self.auto_checkbox_dict[cam_info['name'] + '-Camera-connected'].check()
                            break
                        self.socket_worker.send('uncheked')
                self.socket_worker.send('last_property')

                # self.socket_worker.join()
                # self.socket_worker.ack()
                if self.socket_worker.recv() == 'usb_mode':
                    if self.device.getUsbSpeed() == dai.UsbSpeed.SUPER:
                        self.socket_worker.send('check')
                    else:
                        self.socket_worker.send('uncheck')
                
                if not self.socket_worker.recv() == 'lost_camera':
                    pipeline = create_pipeline(self.board_config)
                    self.device.startPipeline(pipeline)
                    self.camera_queue = {}
                    self.control_queue = {}
                    for config_cam in self.board_config['cameras']:
                        cam = self.board_config['cameras'][config_cam]
                        self.camera_queue[cam['name']] = self.device.getOutputQueue(cam['name'], 5, False)
                        if cam['hasAutofocus']:
                            self.control_queue[cam['name']] = self.device.getInputQueue(cam['name'] + '-control', 5, False)
                    # self.socket_worker.ack()
                else:
                    print("Closing Device...")

                    self.close_device()
                    print("Restarting Device...")
                    # self.socket_worker.ack()
            self.socket_worker.send('device_connected')
        mipi = {}
        for config_cam in self.board_config['cameras']:
            mipi[self.board_config['cameras'][config_cam]['name']] = False

        for _ in range(120):
            # self.socket_worker.join()
            # self.socket_worker.ack()
            print((config_cam in self.board_config['cameras']))
            for config_cam in self.board_config['cameras']:
                # self.socket_worker.send('next')
                name = self.board_config['cameras'][config_cam]['name']
                imageFrame = self.camera_queue[name].tryGetAll()
                if name == 'rgb':
                    print(imageFrame)

                if imageFrame is not None:
                    if len(imageFrame) > 0:
                        # self.socket_worker.send('good_frame')
                        # self.socket_worker.join()
                        mipi[name] = True
                        frame = None

                        if imageFrame[0].getType()== dai.RawImgFrame.Type.RAW8:
                            frame = imageFrame[0].getCvFrame()
                        else:
                            frame = cv2.cvtColor(imageFrame[0].getCvFrame(), cv2.COLOR_BGR2GRAY)
                        # self.socket_worker.send(frame)
                        # self.socket_worker.join()
                        # self.socket_worker.send(name)
                    #else:
                        #self.socket_worker.send('bad_frame')
            #self.socket_worker.send('fin')

            # for m in mipi:
            #     print(f'{mipi[m]=} {m=}')

            isMipiReady = True
            for config_cam in self.board_config['cameras']:
                name = self.board_config['cameras'][config_cam]['name']
                isMipiReady = isMipiReady and mipi[name]
            if isMipiReady:
                break
            # self.socket_worker.ack()
        for key in mipi.keys():
            self.socket_worker.send('next_mipi')
            if not mipi[key]:
                self.socket_worker.send((key + "-Stream", 'uncheck'))
            else:
                self.socket_worker.send((key + "-Stream", 'check'))
        time.sleep(1)
        self.socket_worker.send('finish_mipi')
        # self.socket_worker.join()
        if self.socket_worker.recv() != 'finished':
            self.device.close()
        self.socket_worker.send(self.device.getMxId())
        print('EEEND')


    def capture_servive_handler(self):
        for key in self.camera_queue.keys():
            self.socket_worker.send(key)
            frame = self.camera_queue[key].getAll()[-1]
            if frame.getType() == dai.RawImgFrame.Type.RAW8:
                gray_frame = frame.getCvFrame()
            else:
                gray_frame = cv2.cvtColor(frame.getCvFrame(), cv2.COLOR_BGR2GRAY)
            self.socket_worker.send(gray_frame)
        self.socket_worker.send('next')

        # TODO(sachin): Do I need to cross check lens position of autofocus camera's ?

        if self.socket_worker.recv() == 'close':
            self.device.close()

    def calibration_servive_handler(self):
        self.socket_worker.send(self.device.getMxId())
        # self.socket_worker.join()

        if self.socket_worker.recv() == 'close':
            self.close_device()

        calibration_handler = dai.readCalibration2()
        self.socket_worker.send(calibration_handler)
        # self.socket_worker.join()

        if self.socket_worker.recv() == 'close':
            self.close_device()
            return

        calibration_handler = self.socket_worker.recv()
        try:
            self.device.flashCalibration2(calibration_handler)
            self.device.flashFactoryCalibration(calibration_handler)
        except:
            self.socket_worker.send('error')
        else:
            self.socket_worker.send('flashed')

    def camera_focus_adjuster(self):
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
        # self.socket_worker.ack()
        # rospy.sleep(1)
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
                self.socket_worker.send('next_frame')
                self.socket_worker.send((cam_info['name'], currFrame))

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
                            print("Failed to Focus...!!!")
                            self.socket_worker.send('failed_focus')
                            break
                        continue

                dst_laplace = cv2.Laplacian(currFrame, cv2.CV_64F)
                mu, sigma = cv2.meanStdDev(dst_laplace)

                print('Sigma of {} is {}'.format(cam_info['name'], sigma))
                localFocusThreshold = self.focusSigmaThreshold
                if dst_laplace.shape[1] > 2000:
                    localFocusThreshold = localFocusThreshold / 2

                if sigma > localFocusThreshold:
                    print('Setting focus true for {}'.format(cam_info['name']))
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
                    print("Sending manual focus Control to {} at position {}".format(cam_name, self.lensPosition[cam_name]))
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


def main():
    this_camera = None
    try:
        socket_worker = SocketWorker()
    except:
        del socket_worker
        print('Connexion error')
    else:
        try:
            while True:
                message = socket_worker.recv()
                if message == 'start_camera':
                    print(f'{message=}')
                    # socket_worker.ack()
                    this_camera = DepthaiCamera(socket_worker.recv(), socket_worker)
                elif message == 'capture_service' and this_camera is not None:
                    this_camera.capture_servive_handler()
                elif message == 'calibration_service' and this_camera is not None:
                    this_camera.calibration_servive_handler()
                elif message == 'focus_adjuster' and this_camera is not None:
                    this_camera.camera_focus_adjuster()
                elif message == 'stop_camera':
                    print(f'{message=}')
                    send_message(cs, 'ACK')
                elif message == 'stop_calibration':
                    print(f'{message=}')
                    send_message(cs, 'ACK')
                    break
        finally:
            del socket_worker
            del this_camera
    print('Calibration finished')


if __name__ == '__main__':
    main()
