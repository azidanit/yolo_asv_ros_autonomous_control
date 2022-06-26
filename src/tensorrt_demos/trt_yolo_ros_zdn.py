"""trt_yolo.py

This script demonstrates how to do real-time object detection with
TensorRT optimized YOLO engine.
"""


import enum
import os
import time
import argparse

import numpy as np

import cv2
import pycuda.autoinit  # This is needed for initializing CUDA driver

from utils.yolo_classes import get_cls_dict
from utils.camera import add_camera_args, Camera
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization
from utils.yolo_with_plugins import TrtYOLO

import rospy
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import BoundingBox2DArray, BoundingBox2D
from sensor_msgs.msg import CompressedImage as CompressedImg
from sensor_msgs.msg import Image

import threading
import time

camera_res_w = 640
camera_res_h = 480

img_global = np.zeros((camera_res_h, camera_res_w, 3), dtype=np.uint8)
img_raw_global = np.zeros((camera_res_h, camera_res_w, 3), dtype=np.uint8)

rospy.init_node('vision_trt')

# imgPublisher = rospy.Publisher('/vision/image_detection/raw', Image, queue_size=1)
imgCompressedPublisher = rospy.Publisher('/vision/image_detection/compressed', CompressedImg, queue_size=1)
imgCompressedRawPublisher = rospy.Publisher('/vision/image/compressed', CompressedImg, queue_size=1)
objPublisher = rospy.Publisher('/vision/objects',BoundingBox2DArray,queue_size=1)
objRawPublisher = rospy.Publisher('/vision/objects/raw',BoundingBox2DArray,queue_size=1)

bridge = CvBridge()

img_mtx = threading.Lock()


WINDOW_NAME = 'TrtYOLODemo'


def parse_args():
    """Parse input arguments."""
    desc = ('Capture and display live camera video, while doing '
            'real-time object detection with TensorRT optimized '
            'YOLO model on Jetson')
    parser = argparse.ArgumentParser(description=desc)
    parser = add_camera_args(parser)
    parser.add_argument(
        '-c', '--category_num', type=int, default=80,
        help='number of object categories [80]')
    parser.add_argument(
        '-t', '--conf_thresh', type=float, default=0.7,
        help='set the detection confidence threshold')
    parser.add_argument(
        '-m', '--model', type=str, required=False, default='yolov4_train3_last',
        help=('[yolov3-tiny|yolov3|yolov3-spp|yolov4-tiny|yolov4|'
              'yolov4-csp|yolov4x-mish|yolov4-p5]-[{dimension}], where '
              '{dimension} could be either a single number (e.g. '
              '288, 416, 608) or 2 numbers, WxH (e.g. 416x256)'))
    parser.add_argument(
        '-l', '--letter_box', action='store_true',
        help='inference with letterboxed image [False]')
    args = parser.parse_args()
    return args


def compress_thread():
    global img_global, img_raw_global, bridge, imgCompressedPublisher, imgCompressedRawPublisher
    while(not rospy.is_shutdown()):

        print("thread running")
        # time.sleep(1)
        imgMsgComp = bridge.cv2_to_compressed_imgmsg(img_global)
        # imgMsg = bridge.cv2_to_imgmsg(img_global, encoding="bgr8")
        # imgPublisher.publish(imgMsg)
        imgCompressedPublisher.publish(imgMsgComp)

        imgRawComp = bridge.cv2_to_compressed_imgmsg(img_raw_global)
        imgCompressedRawPublisher.publish(imgRawComp)

        time.sleep(0.01)
    pass

def loop_and_detect(cam, trt_yolo, conf_th, vis):
    global objPublisher, objRawPublisher
    global camera_res_h, camera_res_w
    global img_global, img_mtx, img_raw_global
    """Continuously capture images from camera and do object detection.

    # Arguments
      cam: the camera instance (video source).
      trt_yolo: the TRT YOLO object detector instance.
      conf_th: confidence/score threshold for object detection.
      vis: for visualization.
    """
    full_scrn = False
    fps = 0.0
    tic = time.time()
    while True:

        if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
            break
        img = cam.read()

        img_mtx.acquire()
        img_raw_global = img.copy()
        img_mtx.release()


        if img is None:
            break
        boxes, confs, clss = trt_yolo.detect(img, conf_th)
        
        # print(boxes[0][0],boxes[0][1], boxes[0][2], boxes[0][3], clss[0])
        # x_min, y 
        bb_msg = BoundingBox2DArray()
        bb_msg.header.stamp = rospy.Time.now()
        bb_msg_raw = BoundingBox2DArray()
        bb_msg_raw.header.stamp = rospy.Time.now()

        bbox_sorted = list(boxes)
        bbox_sorted.sort(key = lambda x : x[3], reverse=True)

        # bb_msg.header.stamp = rospy.rospy
        for i in bbox_sorted:
            x_min = min(max(0,i[0]),camera_res_w)
            x_max = min(max(0,i[2]),camera_res_w)
            y_min = min(max(0,i[1]),camera_res_h)
            y_max = min(max(0,i[3]),camera_res_h)
            bb_ins = BoundingBox2D()
            bb_ins.center.x = (x_max + x_min) / 2
            bb_ins.center.y = (y_max + y_min) / 2
            bb_ins.size_x = abs(x_max - x_min)
            bb_ins.size_y = abs(y_max - y_min)

            bb_msg_raw.boxes.append(bb_ins)

            bb_ins_raw = BoundingBox2D()
            bb_ins_raw.center.x = bb_ins.center.x / camera_res_w
            bb_ins_raw.center.y = bb_ins.center.y / camera_res_h
            bb_ins_raw.size_x   = bb_ins.size_x / camera_res_w
            bb_ins_raw.size_y   = bb_ins.size_y / camera_res_h
            bb_msg.boxes.append(bb_ins_raw)
            # print(bb_ins)

        objPublisher.publish(bb_msg)
        objRawPublisher.publish(bb_msg_raw)


        

        img = vis.draw_bboxes(img, boxes, confs, clss)
        img = show_fps(img, fps)
        cv2.imshow(WINDOW_NAME, img)
        toc = time.time()
        curr_fps = 1.0 / (toc - tic)
        # print(fps)

        img_mtx.acquire()
        img_global = img
        img_mtx.release()


        # calculate an exponentially decaying average of fps number
        #fps = curr_fps if fps == 0.0 else (fps*0.95 + curr_fps*0.05)
        fps = curr_fps
        tic = toc
        key = cv2.waitKey(1)
        if key == 27:  # ESC key: quit program
            break
        elif key == ord('F') or key == ord('f'):  # Toggle fullscreen
            full_scrn = not full_scrn
            set_display(WINDOW_NAME, full_scrn)


def main():
    args = parse_args()
    if args.category_num <= 0:
        raise SystemExit('ERROR: bad category_num (%d)!' % args.category_num)
    if not os.path.isfile('yolo/%s.trt' % args.model):
        raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % args.model)

    cam = Camera(args)
    if not cam.isOpened():
        raise SystemExit('ERROR: failed to open camera!')

    cls_dict = get_cls_dict(args.category_num)
    vis = BBoxVisualization(cls_dict)
    trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)

    publish_img_thread = threading.Thread(target=compress_thread, args=())
    publish_img_thread.start()

    open_window(
        WINDOW_NAME, 'Camera TensorRT YOLO Demo',
        cam.img_width, cam.img_height)
    loop_and_detect(cam, trt_yolo, args.conf_thresh, vis=vis)

    cam.release()
    cv2.destroyAllWindows()



main()
