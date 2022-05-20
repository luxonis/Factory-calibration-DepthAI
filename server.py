import socket, pickle
import depthai as dai
import cv2
from datetime import timedelta

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

camToRgbRes = {
                'IMX378' : dai.ColorCameraProperties.SensorResolution.THE_12_MP,
                'IMX214' : dai.ColorCameraProperties.SensorResolution.THE_12_MP,
                'OV9*82' : dai.ColorCameraProperties.SensorResolution.THE_800_P,
                }


class SocketWorker:
    def __init__(self):
        HOST = "192.168.1.6"
        PORT = 50007
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        print(f'Connected by {addr}')
        CHOST = '192.168.1.3'
        CPORT = 5008
        cs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cs.connect((CHOST, CPORT))
        self.recv_conn, self.send_conn = conn, cs

    def __del__(self):
        if hasattr(self, 'recv_conn'):
            self.recv_conn.close()
        if hasattr(self, 'send_conn'):
            self.send_conn.close()

    def send(self, msg):
        data_string = pickle.dumps(msg)
        self.send_conn.send(data_string)

    def recv(self):
        data = self.recv_conn.recv(4096)
        msg = pickle.loads(data)
        return msg

    def join(self):
        if not (self.recv() == 'ACK'):
            raise RuntimeError('Socket network error')

    def ack(self):
        self.send('ACK')


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
        self.socket_worker.ack()

        while self.device is None or self.device.isClosed():
            searchTime = timedelta(seconds=80)
            isFound, deviceInfo = dai.Device.getAnyAvailableDevice(searchTime)
            if isFound:
                self.socket_worker.send('is_found')
                self.device = dai.Device()
                cameraProperties = self.device.getConnectedCameraProperties()

                # fill_color_2 = pygame.Rect(390, 120, 500, 100)
                # pygame.draw.rect(self.screen, white, fill_color_2)

                # rospy.sleep(2)
                text = "device Mx_id : " + self.device.getMxId()
                self.socket_worker.send(text)
                self.socket_worker.join()

                # pygame_render_text(self.screen, text, (400, 120), black, 30)
                # text = "Device Connected!!!"
                # pygame_render_text(self.screen, text, (400, 150), green, 30)

                lost_camera = False
                for properties in cameraProperties:
                    self.socket_worker.send('new_property')
                    for in_cam in self.board_config['cameras'].keys():
                        cam_info = self.board_config['cameras'][in_cam]
                        if properties.socket == stringToCam[in_cam]:
                            self.board_config['cameras'][in_cam]['sensorName'] = properties.sensorName
                            print('Cam: {} and focus: {}'.format(cam_info['name'], properties.hasAutofocus))
                            self.board_config['cameras'][in_cam]['hasAutofocus'] = properties.hasAutofocus
                            self.socket_worker.send('checked')
                            # self.auto_checkbox_dict[cam_info['name'] + '-Camera-connected'].check()
                            break
                        self.socket_worker.send('uncheked')
                self.socket_worker.send('last_property')

                self.socket_worker.join()
                if self.socket_worker.recv() == 'usb_mode':
                    if self.device.getUsbSpeed() == dai.UsbSpeed.SUPER:
                        self.socket_worker.send('check')
                    else:
                        self.socket_worker.send('uncheck')

                if not self.socket_worker.recv == 'lost_camera':
                    pipeline = self.create_pipeline(self.board_config)
                    self.device.startPipeline(pipeline)
                    self.camera_queue = {}
                    self.control_queue = {}
                    for config_cam in self.board_config['cameras']:
                        cam = self.board_config['cameras'][config_cam]
                        self.camera_queue[cam['name']] = self.device.getOutputQueue(cam['name'], 5, False)
                        if cam['hasAutofocus']:
                            self.control_queue[cam['name']] = self.device.getInputQueue(cam['name'] + '-control', 5, False)
                    self.socket_worker.ack()
                else:
                    print("Closing Device...")

                    self.close_device()
                    print("Restarting Device...")
                    self.socket_worker.ack()
                mipi = {}
                for config_cam in self.board_config['cameras']:
                    mipi[self.board_config['cameras'][config_cam]['name']] = False

                for _ in range(120):
                    for config_cam in self.board_config['cameras']:
                        self.socket_worker.send('next')
                        name = self.board_config['cameras'][config_cam]['name']
                        imageFrame = self.camera_queue[name].tryGet()

                        if imageFrame is not None:
                            mipi[name] = True
                            frame = None

                            if imageFrame.getType() == dai.RawImgFrame.Type.RAW8:
                                frame = imageFrame.getCvFrame()
                            else:
                                frame = cv2.cvtColor(imageFrame.getCvFrame(), cv2.COLOR_BGR2GRAY)
                            self.socket_worker.send('good_frame')
                            self.socket_worker.send(frame)
                        else:
                            self.socket_worker.send('bad_frame')

                    isMipiReady = True
                    for config_cam in self.board_config['cameras']:
                        name = self.board_config['cameras'][config_cam]['name']
                        isMipiReady = isMipiReady and mipi[name]
                    if isMipiReady:
                        break
                    self.socket_worker.ack()
                for key in mipi.keys():
                    self.socket_worker.send('next_mipi')
                    if not mipi[key]:
                        self.socket_worker.send((key + "-Stream", 'uncheck'))
                    else:
                        self.socket_worker.send((key + "-Stream", 'check'))
                self.socket_worker.send('finish_mipi')
                self.socket_worker.join()
                if self.socket_worker.recv != 'finished':
                    self.device.close()
                self.socket_worker.ack()
                self.socket_worker.send(self.device.getMxId())

    def capture_servive_handler(self):
        self.socket_worker.join()

        for key in self.camera_queue.keys():
            frame = self.camera_queue[key].getAll()[-1]
            if frame.getType() == dai.RawImgFrame.Type.RAW8:
                gray_frame = frame.getCvFrame()
            else:
                gray_frame = cv2.cvtColor(frame.getCvFrame(), cv2.COLOR_BGR2GRAY)
            self.socket_worker.send(gray_frame)

        self.socket_worker.join()

        # TODO(sachin): Do I need to cross check lens position of autofocus camera's ?

        if self.socket_worker.recv('close'):
            self.device.close()

    def calibration_servive_handler(self):
        self.socket_worker.send(self.device.getMxId())
        self.socket_worker.join()

        if self.socket_worker.recv() == 'close':
            self.close_device()

        calibration_handler = dai.readCalibration2()
        self.socket_worker.send(calibration_handler)
        self.socket_worker.join()

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
        self.socket_worker.ack()
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
                    socket_worker.send('ACK')
                    this_camera = DepthaiCamera(socket_worker.recv(), socket_worker)
                    # send_message('ACK')
                elif message == 'capture_service' and this_camera is not None:
                    this_camera.capture_servive_handler()
                elif message == 'calibration_service' and this_camera is not None:
                    this_camera.calibration_servive_handler()
                elif message == 'focus_adjuster' and this_camera is not None:
                    this_camera.camera_focus_adjuster()
                elif message == 'stop_camera':
                    print(f'{message=}')
                    # if this_camera is not None:
                    #     this_camera.stop_camera()
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
