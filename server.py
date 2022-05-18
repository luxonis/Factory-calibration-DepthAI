import socket, pickle
import depthai as dai

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


def send_message(conn, message):
    data_string = pickle.dumps(message)
    conn.send(data_string)


def recv_message(conn):
    data = conn.recv(4096)
    message = pickle.loads(data)
    return message


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
    def __init__(self, board_config, recv_con, send_con):
        self.board_config = board_config
        self.recv_con = recv_con
        self.send_con = send_con
        start_camera()

    def start_camera(self):
        searchTime = timedelta(seconds=80)
        isFound, deviceInfo = dai.Device.getAnyAvailableDevice(searchTime)
        if isFound:
            self.device = dai.Device()
            cameraProperties = self.device.getConnectedCameraProperties()

            # fill_color_2 = pygame.Rect(390, 120, 500, 100)
            # pygame.draw.rect(self.screen, white, fill_color_2)

            # rospy.sleep(2)
            text = "device Mx_id : " + self.device.getMxId()
            send_message(self.send_con, text)
            # pygame_render_text(self.screen, text, (400, 120), black, 30)
            # text = "Device Connected!!!"
            # pygame_render_text(self.screen, text, (400, 150), green, 30)

            lost_camera = False
            for properties in cameraProperties:
                for in_cam in self.board_config['cameras'].keys():
                    cam_info = self.board_config['cameras'][in_cam]
                    if properties.socket == stringToCam[in_cam]:
                        self.board_config['cameras'][in_cam]['sensorName'] = properties.sensorName
                        print('Cam: {} and focus: {}'.format(cam_info['name'], properties.hasAutofocus))
                        self.board_config['cameras'][in_cam]['hasAutofocus'] = properties.hasAutofocus
                        send_message(self.send_con, 'ACK')
                        # self.auto_checkbox_dict[cam_info['name'] + '-Camera-connected'].check()
                        break

            # for config_cam in board_config['cameras'].keys():
            #     cam_info = board_config['cameras'][config_cam]
            #     if self.auto_checkbox_dict[cam_info['name'] + '-Camera-connected'].isUnattended():
            #         self.auto_checkbox_dict[cam_info['name'] + '-Camera-connected'].uncheck()
            #         lost_camera = True
            #     self.auto_checkbox_dict[cam_info['name'] + '-Camera-connected'].render_checkbox()
            #
            # if self.args['usbMode']:
            #     if self.device.getUsbSpeed() == dai.UsbSpeed.SUPER:
            #         self.auto_checkbox_dict["USB3"].check()
            #     else:
            #         lost_camera = True
            #         self.auto_checkbox_dict["USB3"].uncheck()
            #     self.auto_checkbox_dict["USB3"].render_checkbox()

            if not lost_camera:
                pipeline = self.create_pipeline(self.board_config)
                self.device.startPipeline(pipeline)
                self.camera_queue = {}
                self.control_queue = {}
                for config_cam in self.board_config['cameras']:
                    cam = self.board_config['cameras'][config_cam]
                    self.camera_queue[cam['name']] = self.device.getOutputQueue(cam['name'], 5, False)
                    if cam['hasAutofocus']:
                        self.control_queue[cam['name']] = self.device.getInputQueue(cam['name'] + '-control', 5, False)
            else:
                print("Closing Device...")

                # fill_color_2 = pygame.Rect(390, 150, 220, 35)
                # pygame.draw.rect(self.screen, white, fill_color_2)
                # text = "Device Disconnected!!!"
                # pygame_render_text(self.screen, text, (400, 150), red, 30)
                # text = "Click RETEST when device is ready!!!"
                # pygame_render_text(self.screen, text, (400, 180), red, 30)

                self.close_device()
                self.retest()
                print("Restarting Device...")

                fill_color_2 = pygame.Rect(390, 430, 120, 35)
                pygame.draw.rect(self.screen, white, fill_color_2)

    def stop_camera(self):
        self.device.close()

def main():

    HOST = "192.168.1.6"
    PORT = 50007
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    print(f'Connected by {addr}')
    this_camera = None
    CHOST = '192.168.1.3'
    CPORT = 5008
    cs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cs.connect((CHOST, CPORT))
    try:
        while True:
            message = recv_message(conn)
            if message == 'start_camera':
                print(f'{message=}')
                send_message(cs, 'ACK')
                message = recv_message()
                this_camera = DepthaiCamera(message, conn, cs)
                # send_message('ACK')
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
        conn.close()
    print('Calibration finished')

if __name__ == '__main__':
    main()
