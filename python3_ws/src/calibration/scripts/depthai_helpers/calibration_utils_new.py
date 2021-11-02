#!/usr/bin/env python3

import cv2
import glob
import os
import shutil
import numpy as np
import re
import time
import json
import cv2.aruco as aruco
from pathlib import Path
# Creates a set of 13 polygon coordinates


def setPolygonCoordinates(height, width):
    horizontal_shift = width//4
    vertical_shift = height//4

    margin = 60
    slope = 150

    p_coordinates = [
        [[margin, margin], [margin, height-margin],
            [width-margin, height-margin], [width-margin, margin]],

        [[margin, 0], [margin, height], [width//2, height-slope], [width//2, slope]],
        [[horizontal_shift, 0], [horizontal_shift, height], [
            width//2 + horizontal_shift, height-slope], [width//2 + horizontal_shift, slope]],
        [[horizontal_shift*2-margin, 0], [horizontal_shift*2-margin, height], [width//2 +
                                                                               horizontal_shift*2-margin, height-slope], [width//2 + horizontal_shift*2-margin, slope]],

        [[width-margin, 0], [width-margin, height], [width//2, height-slope], [width//2, slope]],
        [[width-horizontal_shift, 0], [width-horizontal_shift, height], [width //
                                                                         2-horizontal_shift, height-slope], [width//2-horizontal_shift, slope]],
        [[width-horizontal_shift*2+margin, 0], [width-horizontal_shift*2+margin, height], [width //
                                                                                           2-horizontal_shift*2+margin, height-slope], [width//2-horizontal_shift*2+margin, slope]],

        [[0, margin], [width, margin], [
            width-slope, height//2], [slope, height//2]],
        [[0, vertical_shift], [width, vertical_shift], [width-slope,
                                                        height//2+vertical_shift], [slope, height//2+vertical_shift]],
        [[0, vertical_shift*2-margin], [width, vertical_shift*2-margin], [width-slope,
                                                                          height//2+vertical_shift*2-margin], [slope, height//2+vertical_shift*2-margin]],

        [[0, height-margin], [width, height-margin],
         [width-slope, height//2], [slope, height//2]],
        [[0, height-vertical_shift], [width, height-vertical_shift], [width -
                                                                      slope, height//2-vertical_shift], [slope, height//2-vertical_shift]],
        [[0, height-vertical_shift*2+margin], [width, height-vertical_shift*2+margin], [width -
                                                                                        slope, height//2-vertical_shift*2+margin], [slope, height//2-vertical_shift*2+margin]]
    ]
    return p_coordinates


def getPolygonCoordinates(idx, p_coordinates):
    return p_coordinates[idx]


def getNumOfPolygons(p_coordinates):
    return len(p_coordinates)

# Filters polygons to just those at the given indexes.


def select_polygon_coords(p_coordinates, indexes):
    if indexes == None:
        # The default
        return p_coordinates
    else:
        print("Filtering polygons to those at indexes=", indexes)
        return [p_coordinates[i] for i in indexes]


def image_filename(stream_name, polygon_index, total_num_of_captured_images):
    return "{stream_name}_p{polygon_index}_{total_num_of_captured_images}.png".format(stream_name=stream_name, polygon_index=polygon_index, total_num_of_captured_images=total_num_of_captured_images)


def polygon_from_image_name(image_name):
    """Returns the polygon index from an image name (ex: "left_p10_0.png" => 10)"""
    return int(re.findall("p(\d+)", image_name)[0])


class StereoCalibration(object):
    """Class to Calculate Calibration and Rectify a Stereo Camera."""

    def __init__(self):
        """Class to Calculate Calibration and Rectify a Stereo Camera."""

    def calibrate(self, dataset, square_size, mrk_size, squaresX, squaresY, camera_model, calibrate_LR, calibrate_rgb, enable_disp_rectify):
        """Function to calculate calibration for stereo camera."""
        # start_time = time.time()
        # init object data
        self.calibrate_LR = calibrate_LR
        self.calibrate_rgb = calibrate_rgb
        self.enable_rectification_disp = enable_disp_rectify
        self.cameraModel = camera_model
        self.data_path = dataset
        self.aruco_dictionary = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        self.board = aruco.CharucoBoard_create(
            # 22, 16,
            squaresX, squaresY,
            square_size,
            mrk_size,
            self.aruco_dictionary)

        # parameters = aruco.DetectorParameters_create()
        assert mrk_size != None,  "ERROR: marker size not set"

        if self.calibrate_LR:
            self.calibrate_charuco3D(dataset)
        else:
            self.R1 = np.zeros((3, 3), dtype=np.float32)
            self.R2 = np.zeros((3, 3), dtype=np.float32)
            self.M1 = np.zeros((3, 3), dtype=np.float32)
            self.M2 = np.zeros((3, 3), dtype=np.float32)
            self.d1 = np.zeros((3, 3), dtype=np.float32)
            self.d2 = np.zeros((3, 3), dtype=np.float32)
            self.R = np.zeros((3, 3), dtype=np.float32)
            self.T = np.zeros((3, 3), dtype=np.float32)

        # rgb-right extrinsic calibration
        if self.calibrate_rgb:
            self.rgb_calibrate(dataset)
        else:
            self.M3 = np.zeros((3, 3), dtype=np.float32)
            self.R_rgb = np.zeros((3, 3), dtype=np.float32)
            self.T_rgb = np.zeros(3, dtype=np.float32)
            self.d3 = np.zeros(14, dtype=np.float32)

        # self.M3_scaled_write = np.copy(self.M3_scaled)
        # self.M3_scaled_write[1, 2] += 40

        R1_fp32 = self.R1.astype(np.float32)
        R2_fp32 = self.R2.astype(np.float32)
        M1_fp32 = self.M1.astype(np.float32)
        M2_fp32 = self.M2.astype(np.float32)
        M3_fp32 = self.M3.astype(np.float32)

        R_fp32 = self.R.astype(np.float32)  # L-R rotation
        T_fp32 = self.T.astype(np.float32)  # L-R translation
        R_rgb_fp32 = self.R_rgb.astype(np.float32)
        T_rgb_fp32 = self.T_rgb.astype(np.float32)

        d1_coeff_fp32 = self.d1.astype(np.float32)
        d2_coeff_fp32 = self.d2.astype(np.float32)
        d3_coeff_fp32 = self.d3.astype(np.float32)

        if self.calibrate_rgb and self.calibrate_LR:
            R_rgb_fp32 = np.linalg.inv(R_rgb_fp32)
            T_rgb_fp32[0] = -T_rgb_fp32[0]
            T_rgb_fp32[1] = -T_rgb_fp32[1]
            T_rgb_fp32[2] = -T_rgb_fp32[2]

        self.calib_data = [R1_fp32, R2_fp32, M1_fp32, M2_fp32, M3_fp32, R_fp32,
                           T_fp32, R_rgb_fp32, T_rgb_fp32, d1_coeff_fp32, d2_coeff_fp32, d3_coeff_fp32]

        if 1:  # Print matrices, to compare with device data
            np.set_printoptions(suppress=True, precision=6)
            print("\nR1 (left)")
            print(R1_fp32)
            print("\nR2 (right)")
            print(R2_fp32)
            print("\nM1 (left)")
            print(M1_fp32)
            print("\nM2 (right)")
            print(M2_fp32)
            print("\nR")
            print(R_fp32)
            print("\nT")
            print(T_fp32)
            print("\nM3 (rgb)")
            print(M3_fp32)
            print("\nR (rgb)")
            print(R_rgb_fp32)
            print("\nT (rgb)")
            print(T_rgb_fp32)

        if 0:  # Print computed homography, to compare with device data
            np.set_printoptions(suppress=True, precision=6)
            for res_height in [800, 720, 400]:
                m1 = np.copy(M1_fp32)
                m2 = np.copy(M2_fp32)
                if res_height == 720:
                    m1[1, 2] -= 40
                    m2[1, 2] -= 40
                if res_height == 400:
                    m_scale = [[0.5,   0, 0],
                               [0, 0.5, 0],
                               [0,   0, 1]]
                    m1 = np.matmul(m_scale, m1)
                    m2 = np.matmul(m_scale, m2)
                h1 = np.matmul(np.matmul(m2, R1_fp32), np.linalg.inv(m1))
                h2 = np.matmul(np.matmul(m2, R2_fp32), np.linalg.inv(m2))
                h1 = np.linalg.inv(h1)
                h2 = np.linalg.inv(h2)
                print('\nHomography H1, H2 for height =', res_height)
                print(h1)
                print()
                print(h2)

        print("\tTook %i seconds to run image processing." %
              (round(time.time() - start_time, 2)))

        # self.create_save_mesh()

        epipolar_RRgb = None
        epipolar_LR = None
        rgb_reprojectErrors = None
        if self.calibrate_rgb:
            rgb_reprojectErrors = self.ret_rgb_scaled
            if self.calibrate_LR:
                epipolar_RRgb = self.test_epipolar_charuco_rgbr(filepath)

        if self.calibrate_LR:
            epipolar_LR = self.test_epipolar_charuco_lr(filepath)

        return rgb_reprojectErrors, epipolar_LR, epipolar_RRgb, self.calib_data

    def analyze_charuco(self, images, scale_req=False, req_resolution=(800, 1280)):
        """
        Charuco base pose estimation.
        """
        # print("POSE ESTIMATION STARTS:")
        allCorners = []
        allIds = []
        all_marker_corners = []
        all_marker_ids = []
        all_recovered = []
        # decimator = 0
        # SUB PIXEL CORNER DETECTION CRITERION
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
        count = 0
        for im in images:
            print("=> Processing image {0}".format(im))
            frame = cv2.imread(im)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # gray = cv2.flip(gray, 0) # TODO(Sachin) : remove this later
            # width = scale[1]
            expected_height = gray.shape[0]*(req_resolution[1]/gray.shape[1])

            if scale_req and not (gray.shape[0] == req_resolution[0] and gray.shape[1] == req_resolution[1]):
                if int(expected_height) == req_resolution[0]:
                    # resizing to have both stereo and rgb to have same
                    # resolution to capture extrinsics of the rgb-right camera
                    gray = cv2.resize(gray, req_resolution[::-1],
                                      interpolation=cv2.INTER_CUBIC)
                else:
                    # resizing and cropping to have both stereo and rgb to have same resolution
                    # to calculate extrinsics of the rgb-right camera
                    scale_width = req_resolution[1]/gray.shape[1]
                    dest_res = (
                        int(gray.shape[1] * scale_width), int(gray.shape[0] * scale_width))
                    gray = cv2.resize(
                        gray, dest_res, interpolation=cv2.INTER_CUBIC)
                    if gray.shape[0] < req_resolution[0]:
                        raise RuntimeError("resizeed height of rgb is smaller than required. {0} < {1}".format(
                            gray.shape[0], req_resolution[0]))
                    # print(gray.shape[0] - req_resolution[0])
                    del_height = (gray.shape[0] - req_resolution[0]) // 2
                    # gray = gray[: req_resolution[0], :]
                    gray = gray[del_height: del_height + req_resolution[0], :]

                count += 1
            marker_corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
                gray, self.aruco_dictionary)
            marker_corners, ids, refusd, recoverd = cv2.aruco.refineDetectedMarkers(gray, self.board,
                                                                                    marker_corners, ids, rejectedCorners=rejectedImgPoints)
            print('{0} number of Markers corners detected in the above image'.format(
                len(marker_corners)))
            if len(marker_corners) > 0:
                res2 = cv2.aruco.interpolateCornersCharuco(
                    marker_corners, ids, gray, self.board)

                # if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
                if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3:

                    cv2.cornerSubPix(gray, res2[1],
                                     winSize=(5, 5),
                                     zeroZone=(-1, -1),
                                     criteria=criteria)
                    allCorners.append(res2[1])  # Charco chess corners
                    allIds.append(res2[2])  # charuco chess corner id's
                    all_marker_corners.append(marker_corners)
                    all_marker_ids.append(ids)
                    all_recovered.append(recoverd)
                else:
                    print("in else")
            else:
                print(im + " Not found")
            # decimator+=1

        imsize = gray.shape
        return allCorners, allIds, all_marker_corners, all_marker_ids, imsize, all_recovered

    def calibrate_intrinsics(self, image_files):
        image_files.sort()
        assert len(
            image_files) != 0, "ERROR: Images not read correctly, check directory"

        allCorners, allIds, _, _, imsize, _ = self.analyze_charuco(image_files)
        if self.camera_model == 'perspective':
            return self.calibrate_camera_charuco(allCorners, allIds, imsize[::-1])
        else:
            return self.calibrate_fisheye(allCorners, allIds, imsize[::-1])

    def calibrate_extrinsics(self, images_left, images_right, M_l, d_l, M_r, d_r, guess_translation, guess_rotation):
        self.objpoints = []  # 3d point in real world space
        self.imgpoints_l = []  # 2d points in image plane.
        self.imgpoints_r = []  # 2d points in image plane.

        images_left.sort()
        images_right.sort()

        assert len(
            images_left) != 0, "ERROR: Images not found, check directory"
        assert len(
            images_right) != 0, "ERROR: Images not found, check directory"

        scale = None
        scale_req = False
        frame_left_shape = cv2.imread(images_left[0]).shape
        frame_right_shape = cv2.imread(images_right[0]).shape
        scalable_res = frame_left_shape
        scaled_res = frame_right_shape
        if frame_right_shape[0] < frame_left_shape[0] and frame_right_shape[1] < frame_left_shape[1]:
            scale_req = True
            scale = frame_right_shape[1] / frame_left_shape[1]
        elif frame_right_shape[0] > frame_left_shape[0] and frame_right_shape[1] > frame_left_shape[1]:
            scale_req = True
            scale = frame_left_shape[1] / frame_right_shape[1]
            scalable_res = frame_right_shape
            scaled_res = frame_left_shape

        if scale_req:
            scaled_height = scale * scalable_res[0]
            diff = scaled_height - scaled_res[0]
            # if scaled_height <  smaller_res[0]:
            if diff < 0:
                scaled_res = (scaled_height, scaled_res[1])

        # scaled_res = (scaled_height, )
        M_lp = self.scale_intrinsics(M_l, frame_left_shape, scaled_res)
        M_rp = self.scale_intrinsics(M_r, frame_right_shape, scaled_res)
        
        print("~~~~~~~~~~~ POSE ESTIMATION LEFT CAMERA ~~~~~~~~~~~~~")
        allCorners_l, allIds_l, _, _, imsize_l, _ = self.analyze_charuco(
            images_left, scale_req, scaled_res)

        print("~~~~~~~~~~~ POSE ESTIMATION RIGHT CAMERA ~~~~~~~~~~~~~")
        allCorners_r, allIds_r, _, _, imsize_r, _ = self.analyze_charuco(
            images_right, scale_req, scaled_res)

        assert imsize_r != imsize_l, "Left and right resolution scaling is wrong"

        return self.calibrate_stereo(
            allCorners_l, allIds_l, allCorners_r, allIds_r, imsize_r, M_lp, d_l, M_rp, d_r)

    def scale_intrinsics(self, intrinsics, originalShape, destShape):
        scale = destShape[1] / originalShape[1]
        scale_mat = np.array([[scale, 0, 0], [0, scale, 0], [0, 0, 1]])
        scaled_intrinsics = np.matmul(scale_mat, intrinsics)
        print("Scaled height offset : {}".format((originalShape[0] * scale - destShape[0]) / 2))
        print("Scaled width offset : {}".format((originalShape[1] * scale - destShape[1]) / 2))
        scaled_intrinsics[1][2] -= (originalShape[0] * scale - destShape[0]) / 2;
        scaled_intrinsics[0][2] -= (originalShape[1] * scale - destShape[1]) / 2;
        return scaled_intrinsics

    def fisheye_undistort_visualizaation(self, img_list, K, D, img_size):
        for im in img_list:
            # print(im)
            img = cv2.imread(im)
            # h, w = img.shape[:2]
            if self.cameraModel == 'perspective':
                map1, map2 = cv2.initUndistortRectifyMap(
                    K, D, np.eye(3), K, img_size, cv2.CV_32FC1)
            else:
                map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                    K, D, np.eye(3), K, img_size, cv2.CV_32FC1)

            undistorted_img = cv2.remap(
                img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            cv2.imshow("undistorted", undistorted_img)
            cv2.waitKey(0)
            # cv2.destroyAllWindows()

    def calibrate_camera_charuco(self, allCorners, allIds, imsize, hfov):
        """
        Calibrates the camera using the dected corners.
        """
        f = imsize[1] / (2 * np.tan(np.deg2rad(hfov/2)))
        # TODO(sachin): Change the initialization to be initialized using the guess from fov
        print("CAMERA CALIBRATION")
        print(imsize)
        cameraMatrixInit = np.array([[f,    0.0,      imsize[1]/2],
                                     [0.0,     f,      imsize[0]/2],
                                     [0.0,   0.0,        1.0]])
        """ if imsize[1] < 700:
            cameraMatrixInit = np.array([[400.0,    0.0,      imsize[0]/2],
                                         [0.0,     400.0,  imsize[1]/2],
                                         [0.0,        0.0,        1.0]])
        elif imsize[1] < 1100:
            cameraMatrixInit = np.array([[857.1668,    0.0,      643.9126],
                                         [0.0,     856.0823,  387.56018],
                                         [0.0,        0.0,        1.0]])
        else:
            cameraMatrixInit = np.array([[3819.8801,    0.0,     1912.8375],
                                         [0.0,     3819.8801, 1135.3433],
                                         [0.0,        0.0,        1.]]) """

        print("Camera Matrix initialization.............")
        print(cameraMatrixInit)

        distCoeffsInit = np.zeros((5, 1))
        flags = (cv2.CALIB_USE_INTRINSIC_GUESS +
                 cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    #     flags = (cv2.CALIB_RATIONAL_MODEL)
        (ret, camera_matrix, distortion_coefficients,
         rotation_vectors, translation_vectors,
         stdDeviationsIntrinsics, stdDeviationsExtrinsics,
         perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
            charucoCorners=allCorners,
            charucoIds=allIds,
            board=self.board,
            imageSize=imsize,
            cameraMatrix=cameraMatrixInit,
            distCoeffs=distCoeffsInit,
            flags=flags,
            criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

        return ret, camera_matrix, distortion_coefficients, rotation_vectors, translation_vectors

    def calibrate_fisheye(self, allCorners, allIds, imsize):
        one_pts = self.board.chessboardCorners
        obj_points = []
        for i in range(len(allIds)):
            obj_pts_sub = []
            for j in range(len(allIds[i])):
                obj_pts_sub.append(one_pts[allIds[i][j]])
            obj_points.append(np.array(obj_pts_sub, dtype=np.float32))

        cameraMatrixInit = np.array([[500,    0.0,      643.9126],
                                    [0.0,     500,  387.56018],
                                    [0.0,        0.0,        1.0]])

        print("Camera Matrix initialization.............")
        print(cameraMatrixInit)
        flags = 0
        distCoeffsInit = np.zeros((4, 1))
        term_criteria = (cv2.TERM_CRITERIA_COUNT +
                         cv2.TERM_CRITERIA_EPS, 100, 1e-5)

        return cv2.fisheye.calibrate(obj_points, allCorners, imsize, cameraMatrixInit, distCoeffsInit, flags=flags, criteria=term_criteria)

    def calibrate_stereo(self, allCorners_l, allIds_l, allCorners_r, allIds_r, imsize, cameraMatrix_l, distCoeff_l, cameraMatrix_r, distCoeff_r, r_in, t_in):
        left_corners_sampled = []
        right_corners_sampled = []
        obj_pts = []
        one_pts = self.board.chessboardCorners
        print('allIds_l')
        print(len(allIds_l))
        print('allIds_r')
        print(len(allIds_r))
        print('allIds_l')
        # print(allIds_l)
        # print(allIds_r)

        for i in range(len(allIds_l)):
            left_sub_corners = []
            right_sub_corners = []
            obj_pts_sub = []
        #     if len(allIds_l[i]) < 70 or len(allIds_r[i]) < 70:
        #         continue
            for j in range(len(allIds_l[i])):
                idx = np.where(allIds_r[i] == allIds_l[i][j])
                if idx[0].size == 0:
                    continue
                left_sub_corners.append(allCorners_l[i][j])
                right_sub_corners.append(allCorners_r[i][idx])
                obj_pts_sub.append(one_pts[allIds_l[i][j]])
            if len(left_sub_corners) > 3 and len(right_sub_corners) > 3:
                obj_pts.append(np.array(obj_pts_sub, dtype=np.float32))
                left_corners_sampled.append(
                    np.array(left_sub_corners, dtype=np.float32))
                right_corners_sampled.append(
                    np.array(right_sub_corners, dtype=np.float32))
            else:
                return [-1, "Stereo Calib failed due to less common features"]
        stereocalib_criteria = (cv2.TERM_CRITERIA_COUNT +
                                cv2.TERM_CRITERIA_EPS, 100, 1e-5)

        if self.cameraModel == 'perspective':
            flags = 0
            flags |= cv2.CALIB_FIX_INTRINSIC
            flags |= cv2.CALIB_RATIONAL_MODEL
            flags |= cv2.CALIB_USE_EXTRINSIC_GUESS

            ret, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate(
                obj_pts, left_corners_sampled, right_corners_sampled,
                cameraMatrix_l, distCoeff_l, cameraMatrix_r, distCoeff_r, imsize,
                r_in, t_in, criteria=stereocalib_criteria, flags=flags)

            R_l, R_r, P_l, P_r, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
                cameraMatrix_l,
                distCoeff_l,
                cameraMatrix_r,
                distCoeff_r,
                imsize, R, T)
            return [ret, R, T, R_l, R_r]

        elif self.cameraModel == 'fisheye':
            # print(len(obj_pts))
            # print('obj_pts')
            # print(obj_pts)
            # print(len(left_corners_sampled))
            # print('left_corners_sampled')
            # print(left_corners_sampled)
            # print(len(right_corners_sampled))
            # print('right_corners_sampled')
            # print(right_corners_sampled)
            for i in range(len(obj_pts)):
                print('---------------------')
                print(i)
                print(len(obj_pts[i]))
                print(len(left_corners_sampled[i]))
                print(len(right_corners_sampled[i]))
            flags = 0
            # flags |= cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC # TODO(SACHIN): Try without intrinsic guess
            ret, M1, d1, M2, d2, R, T, E, F = cv2.fisheye.stereoCalibrate(
                obj_pts, left_corners_sampled, right_corners_sampled,
                cameraMatrix_l, distCoeff_l, cameraMatrix_r, distCoeff_r, imsize,
                flags=flags, criteria=stereocalib_criteria), None, None

            R_l, R_r, P_l, P_r, Q = cv2.fisheye.stereoRectify(
                cameraMatrix_l,
                distCoeff_l,
                cameraMatrix_r,
                distCoeff_r,
                imsize, R, T)
            return [ret, R, T, R_l, R_r]

    def display_rectification(self, image_data_pairs):
        print(
            "Displaying Stereo Pair for visual inspection. Press the [ESC] key to exit.")
        for image_data_pair in image_data_pairs:
            img_concat = cv2.hconcat([image_data_pair[0], image_data_pair[1]])
            img_concat = cv2.cvtColor(img_concat, cv2.COLOR_GRAY2RGB)

            # draw epipolar lines for debug purposes
            line_row = 0
            while line_row < img_concat.shape[0]:
                cv2.line(img_concat,
                         (0, line_row), (img_concat.shape[1], line_row),
                         (0, 255, 0), 1)
                line_row += 30

            # show image
            cv2.imshow('Stereo Pair', img_concat)
            k = cv2.waitKey(0)
            if k == 27:  # Esc key to stop
                break

                # os._exit(0)
                # raise SystemExit()

        cv2.destroyWindow('Stereo Pair')

    def display_rectification(self, image_data_pairs):
        print(
            "Displaying Stereo Pair for visual inspection. Press the [ESC] key to exit.")
        for image_data_pair in image_data_pairs:
            img_concat = cv2.hconcat([image_data_pair[0], image_data_pair[1]])
            img_concat = cv2.cvtColor(img_concat, cv2.COLOR_GRAY2RGB)

            # draw epipolar lines for debug purposes
            line_row = 0
            while line_row < img_concat.shape[0]:
                cv2.line(img_concat,
                         (0, line_row), (img_concat.shape[1], line_row),
                         (0, 255, 0), 1)
                line_row += 30

            # show image
            cv2.imshow('Stereo Pair', img_concat)
            k = cv2.waitKey(0)
            if k == 27:  # Esc key to stop
                break

                # os._exit(0)
                # raise SystemExit()

        cv2.destroyWindow('Stereo Pair')

    def create_save_mesh(self):  # , output_path):

        curr_path = Path(__file__).parent.resolve()
        print("Mesh path")
        print(curr_path)

        map_x_l, map_y_l = cv2.initUndistortRectifyMap(
            self.M1, self.d1, self.R1, self.M2, self.img_shape, cv2.CV_32FC1)
        map_x_r, map_y_r = cv2.initUndistortRectifyMap(
            self.M2, self.d2, self.R2, self.M2, self.img_shape, cv2.CV_32FC1)

        """ 
        map_x_l_fp32 = map_x_l.astype(np.float32)
        map_y_l_fp32 = map_y_l.astype(np.float32)
        map_x_r_fp32 = map_x_r.astype(np.float32)
        map_y_r_fp32 = map_y_r.astype(np.float32)
        
                
        print("shape of maps")
        print(map_x_l.shape)
        print(map_y_l.shape)
        print(map_x_r.shape)
        print(map_y_r.shape) """

        meshCellSize = 16
        mesh_left = []
        mesh_right = []

        for y in range(map_x_l.shape[0] + 1):
            if y % meshCellSize == 0:
                row_left = []
                row_right = []
                for x in range(map_x_l.shape[1] + 1):
                    if x % meshCellSize == 0:
                        if y == map_x_l.shape[0] and x == map_x_l.shape[1]:
                            row_left.append(map_y_l[y - 1, x - 1])
                            row_left.append(map_x_l[y - 1, x - 1])
                            row_right.append(map_y_r[y - 1, x - 1])
                            row_right.append(map_x_r[y - 1, x - 1])
                        elif y == map_x_l.shape[0]:
                            row_left.append(map_y_l[y - 1, x])
                            row_left.append(map_x_l[y - 1, x])
                            row_right.append(map_y_r[y - 1, x])
                            row_right.append(map_x_r[y - 1, x])
                        elif x == map_x_l.shape[1]:
                            row_left.append(map_y_l[y, x - 1])
                            row_left.append(map_x_l[y, x - 1])
                            row_right.append(map_y_r[y, x - 1])
                            row_right.append(map_x_r[y, x - 1])
                        else:
                            row_left.append(map_y_l[y, x])
                            row_left.append(map_x_l[y, x])
                            row_right.append(map_y_r[y, x])
                            row_right.append(map_x_r[y, x])
                if (map_x_l.shape[1] % meshCellSize) % 2 != 0:
                    row_left.append(0)
                    row_left.append(0)
                    row_right.append(0)
                    row_right.append(0)

                mesh_left.append(row_left)
                mesh_right.append(row_right)

        mesh_left = np.array(mesh_left)
        mesh_right = np.array(mesh_right)
        left_mesh_fpath = str(curr_path) + '/../resources/left_mesh.calib'
        right_mesh_fpath = str(curr_path) + '/../resources/right_mesh.calib'
        mesh_left.tofile(left_mesh_fpath)
        mesh_right.tofile(right_mesh_fpath)