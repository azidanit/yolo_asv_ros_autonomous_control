"""trt_yolo.py

This script demonstrates how to do real-time object detection with
TensorRT optimized YOLO engine.
"""


import os
import time
import argparse

import cv2
import pycuda.autoinit  # This is needed for initializing CUDA driver

from utils.yolo_classes import get_cls_dict
from utils.camera import add_camera_args, Camera
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization
from utils.yolo_with_plugins import TrtYOLO


import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as Img
from sensor_msgs.msg import CompressedImage as CompressedImg
from rospy.numpy_msg import numpy_msg
from object_detection.msg import Objects, BoundingBox, BoundingBoxes



WINDOW_NAME = 'TrtYOLODemo'

SCALE_X = 640.0 / 416.0
SCALE_Y = 480.0 / 416.0


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
        '-m', '--model', type=str, required=True,
        help=('[yolov3-tiny|yolov3|yolov3-spp|yolov4-tiny|yolov4|'
              'yolov4-csp|yolov4x-mish]-[{dimension}], where '
              '{dimension} could be either a single number (e.g. '
              '288, 416, 608) or 2 numbers, WxH (e.g. 416x256)'))
    parser.add_argument(
        '-l', '--letter_box', action='store_true',
        help='inference with letterboxed image [False]')
    args = parser.parse_args()
    return args

bridge = CvBridge()
imgPublisher = rospy.Publisher('object_image/image_raw/compressed', CompressedImg, queue_size=1)
objPublisher = rospy.Publisher('objects', Objects, queue_size=1)
# dataStmSubs = rospy.Subscriber('data_to_stm',drive_system,servo_val)

def loop_and_detect(cam, trt_yolo, conf_th, vis):
    global bridge, imgPublisher, objPublisher
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
    while not rospy.is_shutdown():
        if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
            # break
            pass
        img = cam.read()
        
        img_hsv = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
        if img is None:
            # break
            print("IMAGE NOT FOUND")
            continue
        boxes, confs, clss = trt_yolo.detect(img, conf_th)
        # boxes coordinates automaically transformed to original resolution
        iob = list(enumerate(boxes))
        iob = sorted(iob, key=lambda x:x[1][3])
        bbox_msg_data = [[],[]]
        for i,bb in reversed(iob):
            x_min, y_min, x_max, y_max = bb[0], bb[1], bb[2], bb[3]
            h = abs(y_max-y_min)
            l = abs(x_max-x_min)
            box = BoundingBox(ymin=y_min,xmin=x_min,xmax=x_max,ymax=y_max)
            mask = np.zeros((h,l))
            # mask = cv2.circle(mask,(int(h//2),int(l//2)),min(h,l)/2,1,-1) 
            # avg = np.sum(img_hsv[y_min:y_max,x_min:x_max,0]*mask)/np.sum(mask)
            avg = np.mean(img_hsv[y_min:y_max,x_min:x_max,0])
            distance_to_red = avg
            distance_to_green = avg-60
            if distance_to_red > 90:
                distance_to_red = 180 - distance_to_red
            if distance_to_green > 90:
                distance_to_green = 180 - distance_to_green
            if distance_to_green < 0:
                distance_to_green = 180 + distance_to_green

            color = distance_to_red <= distance_to_green
            bbox_msg_data[color].append(box)
        objMsg = Objects()
        objMsg.objects[0].boxes = bbox_msg_data[0]
        objMsg.objects[1].boxes = bbox_msg_data[1]
        objPublisher.publish(objMsg)
        img_to_send = np.copy(img)

        img = vis.draw_bboxes(img, boxes, confs, clss)
        img = show_fps(img, fps)
        img_to_send = show_fps(img_to_send,fps)
        imgMsg = bridge.cv2_to_compressed_imgmsg(img_to_send, dst_format = 'jpg')
        imgPublisher.publish(imgMsg)
        cv2.imshow(WINDOW_NAME, img)
        toc = time.time()
        curr_fps = 1.0 / (toc - tic)
        # calculate an exponentially decaying average of fps number
        fps = curr_fps if fps == 0.0 else (fps*0.95 + curr_fps*0.05)
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

    rospy.init_node("Camera")

    cam = Camera(args)
    if not cam.isOpened():
        raise SystemExit('ERROR: failed to open camera!')

    cls_dict = get_cls_dict(args.category_num)
    vis = BBoxVisualization(cls_dict)
    trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)

    open_window(
        WINDOW_NAME, 'Camera TensorRT YOLO Demo',
        cam.img_width, cam.img_height)
    loop_and_detect(cam, trt_yolo, conf_th=0.3, vis=vis)

    cam.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
