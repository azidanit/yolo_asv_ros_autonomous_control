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
from sensor_msgs.msg import Image as Imgmsg


import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as Img
from sensor_msgs.msg import CompressedImage as CompressedImg
from rospy.numpy_msg import numpy_msg
from object_detection.msg import BoundingBox, BoundingBoxes
from mission_control.srv import StringService, StringServiceRequest, StringServiceResponse
from mission_control.srv import FloatService, FloatServiceRequest, FloatServiceResponse
from std_msgs.msg import Float32, Float32MultiArray
import zmq
import imagezmq
from utils.image_sender import ImageSender
import base64
import socket


from PIL import Image as pilimage
from PIL import ImageEnhance as pilimageenhance



WINDOW_NAME = 'TrtYOLODemo'

SCALE_X = 640.0 / 608.0
SCALE_Y = 480.0 / 608.0
#SCALE_X = 1920.0 / 608.0
#SCALE_Y = 1080.0 / 608.0


cls_names = {0:"balls", 1:"port_buoy", 2:"starboard_buoy"}


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
#    parser.add_argument(
#        '-m', '--model', type=str, required=True,default = 'kkcyolov4bad'
#        help=('[yolov3-tiny|yolov3|yolov3-spp|yolov4-tiny|yolov4|kkcyolov4bad|'
#              'yolov4-csp|yolov4x-mish]-[{dimension}], where '
#              '{dimension} could be either a single number (e.g. '
#              '288, 416, 608) or 2 numbers, WxH (e.g. 416x256)'))
    parser.add_argument(
           '-m', '--model', type=str, default = 'yolov4-tiny-3l'
                 )

    parser.add_argument(
        '-l', '--letter_box', action='store_true',
        help='inference with letterboxed image [False]')
    
    parser.add_argument(
        '--show', action='store_true'
    )
    parser.add_argument(
        '--no_zmq', action='store_true'
    )
    parser.add_argument(
        '--infer', action='store_true'
    )
    args = parser.parse_args()
    return args

bridge = CvBridge()
infer = False
def infer_service_cb(req : StringServiceRequest):
    global infer
    if req.data == "start":
        infer = True
    elif req.data == "stop":
        infer = False
    ret = StringServiceResponse()
    ret.status = "ok"
    return ret

inferService = rospy.Service('/vision/infer',StringService,infer_service_cb)

imgPublisher = rospy.Publisher('/vision/image_raw/compressed', CompressedImg, queue_size=1)
#objPublisher = rospy.Publisher('objects', Objects, queue_size=1)
objPublisher = rospy.Publisher('/vision/objects',BoundingBoxes,queue_size=1)
# dataStmSubs = rospy.Subscriber('data_to_stm',drive_system,servo_val)
gamma_table = np.array([i for i in range(256)])
def gamma_adjust(gamma):
    global gamma_table
    if gamma <= 0:
        gamma = 1
    invgamma = 1.0/gamma
    gamma_table = np.array([((i/255.0)**invgamma)*255 for i in range(256)]).astype("uint8")

brightness = 0
def lab_distance(lab1, lab2):
    lab1 = np.array(lab1)
    lab2 = np.array(lab2)
    return np.sqrt(np.sum((lab1-lab2)**2))

boatsidex = 0.0
boatsidey = 0.0
critline = 0.0
def cb_boatside(msg : Float32MultiArray):
    global boatsidex, boatsidey
    boatsidex = msg.data[0]
    boatsidey = msg.data[1]
def cb_critline(msg : Float32):
    global critline
    critline = msg.data
image_from_sim = None
def cb_imgsim(msg : CompressedImg):
    global image_from_sim
    print('ros image received')
    image_from_sim = bridge.compressed_imgmsg_to_cv2(msg)
    pass
controlpoint = [0,0,0,0]
def cb_controlpoint(msg : Float32MultiArray):
    global controlpoint
    controlpoint[0] = msg.data[0]
    controlpoint[1] = msg.data[1]
    controlpoint[2] = msg.data[2]
    controlpoint[3] = msg.data[3]
sub_crit = rospy.Subscriber('/debug_critline',Float32,cb_critline,queue_size=1)
sub_boatside = rospy.Subscriber('/debug_boatside',Float32MultiArray,cb_boatside,queue_size=1)
sub_img = rospy.Subscriber('/wamv/sensors/cameras/front_right_camera/image_raw/compressed',CompressedImg,cb_imgsim,queue_size=1)
sub_controlpoint = rospy.Subscriber('/debug_controlpoint',Float32MultiArray,cb_controlpoint,queue_size=1)



def loop_and_detect(cam, trt_yolo, conf_th, vis, args):
    global bridge, imgPublisher, objPublisher, infer, brightness, gamma_table
    global boatsidex, boatsidey, critline, image_from_sim
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
    color_m = [(0,0,255),(0,255,0)]
    greenshift = 0
    rrate = rospy.Rate(30)
    #sender = imagezmq.ImageSender(connect_to='tcp://192.168.50.181:5555',REQ_REP=True)
    #sender = ImageSender("192.168.50.181")
    sender = ImageSender("*")

    #sender.zmq_socket.setsockopt(zmq.RCVTIMEO, 5000)
    #ctx = zmq.Context()
    #img_socket = ctx.socket(zmq.PUB)
    #img_socket.connect('tcp://192.168.50.181:5555')
    #f = cv2.VideoWriter('outt.mp4',cv2.VideoWriter_fourcc('M','J','P','G'), 25, (640,480))
    while not rospy.is_shutdown():
        if args.show:
            if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
                # break
                pass
        img = image_from_sim#cam.read()

        if img is None:
            # break
            print("IMAGE NOT FOUND")
            continue
        # img_hsv[:,:,2] += 10

        #img = cv2.LUT(img,gamma_table)
        img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        img_lab = cv2.cvtColor(img,cv2.COLOR_BGR2LAB)

        # img_hsv[:,:,2] = np.clip(1.0*img_hsv[:,:,2]+brightness,a_min = 0, a_max = 254)
        # img = cv2.cvtColor(img_hsv,cv2.COLOR_HSV2BGR)

        img_to_send = np.copy(img)
        if infer:
            boxes, confs, clss = trt_yolo.detect(img, conf_th)
            # boxes coordinates automaically transformed to original resolution
            iob = list(enumerate(boxes))
            iob = sorted(iob, key=lambda x:x[1][3])
            bbox_msg_data = [[],[]]
            bboxclrlist = [False]*len(iob)
            bbxs_to_send = BoundingBoxes()

            reference_red = cv2.cvtColor(np.array([[ [0,0,255] ]]).astype(np.uint8),cv2.COLOR_BGR2LAB)[0][0]
            reference_green = cv2.cvtColor(np.array([[ [0,100,0] ]]).astype(np.uint8),cv2.COLOR_BGR2LAB)[0][0]
            reference_yellow = cv2.cvtColor(np.array([[ [0,100,100] ]]).astype(np.uint8),cv2.COLOR_BGR2LAB)[0][0]
            reference_blue = cv2.cvtColor(np.array([[ [200,0,0] ]]).astype(np.uint8),cv2.COLOR_BGR2LAB)[0][0]
            reference_black = cv2.cvtColor(np.array([[ [0,0,0] ]]).astype(np.uint8),cv2.COLOR_BGR2LAB)[0][0]

            for i,bb in reversed(iob):
                x_min, y_min, x_max, y_max = bb[0], bb[1], bb[2], bb[3]
                h = abs(y_max-y_min)
                l = abs(x_max-x_min)
                center_x = np.mean([x_min,x_max])
                center_y = np.mean([y_min,y_max])
                bb_msg = BoundingBox()
                cls_name = cls_names[clss[i]]
                color_idx = 4
                if cls_name == "balls" and False:

                    mask = np.zeros((h,l))
                    mask = cv2.circle(mask,(int(h//2),int(l//2)),int(min(h,l)//2),1,-1) 
                    mask = mask.astype(int).astype(bool)
                    labavg = np.mean(img_lab[y_min:y_max,x_min:x_max][mask],axis=0)
                    dist_red = lab_distance(labavg,reference_red)
                    dist_blue = lab_distance(labavg,reference_blue)
                    dist_green = lab_distance(labavg,reference_green)
                    dist_yellow = lab_distance(labavg,reference_yellow)
                    dist_black = lab_distance(labavg,reference_black)
                    lst = [dist_red,dist_blue,dist_green,dist_yellow,dist_black]
                    lstclr = ["red","blue","green","yellow","black"]
                    cls_name = "ball_"+lstclr[np.argmin(lst)]
                    pass
                if cls_name == "balls":
                    
                    mask = np.zeros((h,l))
                    mask = cv2.circle(mask,(int(h//2),int(l//2)),int(min(h,l)//2),1,-1) 
                    mask = mask.astype(int).astype(bool)

                    hues = img_hsv[y_min:y_max,x_min:x_max,0]
                    sats = img_hsv[y_min:y_max,x_min:x_max,0]
                    valid_hues = hues[mask]

                    to_vector_i = lambda hue : np.cos(hue*np.pi/90)
                    to_vector_j = lambda hue : np.sin(hue*np.pi/90)
                    try:
                        avg_hue_i = np.sum(to_vector_i(hues[mask])*sats[mask])/np.sum(sats[mask])
                        avg_hue_j = np.sum(to_vector_j(hues[mask])*sats[mask])/np.sum(sats[mask])
                    except:
                        print("something wrong with this boudningbox")
                        continue

                    avg_hue = np.arctan2(avg_hue_j,avg_hue_i)/2*180/np.pi
                    if avg_hue < 0:
                        avg_hue += 180
                    if i == 0: 
                        print("avg_hue_1", avg_hue)
                    
                    if avg_hue>=130 or avg_hue < 30:
                        cls_name="red ball"
                    elif avg_hue>=30 and avg_hue < 55:
                        cls_name="yellow ball"
                    elif avg_hue>=55 and avg_hue < 85:
                        cls_name="green ball"
                    elif avg_hue>=85 and avg_hue <130:
                        cls_name="blue_ball"
                    pass
                
                if cls_name == "balls" and False:
                    #box = BoundingBox(ymin=y_min,xmin=x_min,xmax=x_max,ymax=y_max)
                    mask = np.zeros((h,l))
                    mask = cv2.circle(mask,(int(h//2),int(l//2)),int(min(h,l)//2),1,-1) 
                    # avg = np.sum(img_hsv[y_min:y_max,x_min:x_max,0]*mask)/np.sum(mask)
                    avg_gr = np.sum(img[y_min:y_max,x_min:x_max,1]*mask)
                    avg_rd = np.sum(img[y_min:y_max,x_min:x_max,2]*mask)
                    to_vector_i = lambda hue : np.cos(hue*np.pi/90)
                    to_vector_j = lambda hue : np.sin(hue*np.pi/90)
                    mask = mask.astype(int).astype(bool)
                    hues = img_hsv[y_min:y_max,x_min:x_max,0]

                    valid_hues = hues[mask]

                    avg_hue_i = np.mean(to_vector_i(hues[mask]))
                    avg_hue_j = np.mean(to_vector_j(hues[mask]))

                    avg_val = np.mean(img_hsv[y_min:y_max,x_min:x_max,2][mask])
                    green_bag = 0
                    red_bag = 0
                    yellow_bag = 0
                    blue_bag = 0
                    for hh in valid_hues:
                        if hh>180-15 -15 or hh < 15:
                            red_bag += 1
                        if hh>=15 and hh < 45:
                            yellow_bag += 1
                        if hh>=45 and hh < 75+15:
                            green_bag += 1
                        #if hh >= 75 and hh <180-15-15:
                        #    blue_bag += 1

                    tmp = [green_bag, red_bag, yellow_bag, blue_bag]
                    tmpp= ['green'  ,'red'   ,'yellow'   ,'blue']
                    tmppp=[(0,255,0),(0,0,255),(0,255,255),(255,0,0),(0,0,0)]
                    color_idx = np.argmax(tmp)
                    cls_name = tmpp[color_idx]+'_ball'

                    if avg_val < 20/100*255:
                        color_idx = 4
                        cls_name = "black_ball"
                    #distance_to_red = avg
                    #distance_to_green = avg-60
                    #if distance_to_red > 90:
                    #   distance_to_red = 180 - distance_to_red
                    #if distance_to_green > 90:
                    #   distance_to_green = 180 - distance_to_green
                    #if distance_to_green < 0:
                    #   distance_to_green = 180 + distance_to_green

                    #bgravg = np.mean(np.mean(img[y_min:y_max,x_min:x_max,:],axis=0),axis=0)
                    #green_avg = np.mean(img[x_min:x_max,y_min:y_max,1])
                    #red_avg = np.mean(img[x_min:x_max,y_min:y_max,2])

                    #green_pseprob = distance_to_green/90
                    #green_pseprob = green_pseprob*bgravg[1]/255

                    #red_pseprob = distance_to_red/90
                    #red_pseprob = red_pseprob*bgravg[2]/255
                    #red_counter = 0
                    #green_counter = 0
                    #for pix in img[y_min:y_max,x_min:x_max,:]:
                    #    for pixx in pix:
                    #        red_counter += int(pixx[2]>pixx[1])
                    #green_counter = (y_max-y_min)*(x_max-x_min) - red_counter



                    #color = red_pseprob >= green_pseprob
                    #color = bgravg[2] >= bgravg[1]
                    #color = bgravg[2] <= (bgravg[1]+greenshift)#red<green
                    # color = avg_rd <= avg_gr
                    # color = distance_to_red >= distance_to_green
                    # if avg<30 or avg>165:
                    #     color = False
                    # else:
                    #     color = True
                    #color = red_avg <= green_avg
                    #color = red_counter <= green_counter
                #bbox_msg_data[color].append(box)
                #bboxclrlist.append(color)
                #bboxclrlist[i] = color
                bb_msg.bounding_box.center.position.x = center_x
                bb_msg.bounding_box.center.position.y = center_y
                bb_msg.bounding_box.size.x = l
                bb_msg.bounding_box.size.y = h
                bb_msg.Class = cls_name
                bbxs_to_send.bounding_boxes.append(bb_msg)
                #cv2.rectangle(img_to_send,(x_min,y_min),(x_max,y_max),tmppp[color_idx],2)
                cv2.rectangle(img_to_send,(x_min,y_min),(x_max,y_max),(0,0,0),2)
                cv2.putText(img_to_send,cls_name+'_'+str(i),(x_min,y_min-10),cv2.FONT_HERSHEY_PLAIN,1,(0,255,0),2)

                #if color:
                #    cv2.rectangle(img_to_send,(x_min,y_min),(x_max,y_max),(0,255,0),2)
                #else:
                #    cv2.rectangle(img_to_send,(x_min,y_min),(x_max,y_max),(0,0,255),2)
            bbxs_to_send.header.stamp = rospy.Time.now()
            bbxs_to_send.header.frame_id = "camera"
            objPublisher.publish(bbxs_to_send)
            #objMsg = Objects()
            #objMsg.objects[0].boxes = bbox_msg_data[0]
            #objMsg.objects[1].boxes = bbox_msg_data[1]
            #objPublisher.publish(objMsg)

            #img = vis.draw_bboxes(img, boxes, confs, clss)
            #img = vis.draw_bboxes_withcolor(img,boxes,confs,clss,bboxclrlist)

        img = show_fps(img, fps)
        img_to_send = show_fps(img_to_send,fps)
        cv2.line(img_to_send,(int(320+boatsidex*640),int(boatsidey*480)),(640,480),(255,0,0),3)
        cv2.line(img_to_send,(int(320-boatsidex*640),int(boatsidey*480)),(0,480),(255,0,0),3)
        cv2.line(img_to_send,(int(0),int(critline*480)),(640,int(critline*480)),(255,0,0),3)

        # controlpoint
        cv2.line(img_to_send,(int(controlpoint[1]),int(controlpoint[0])),(int(controlpoint[3]),int(controlpoint[2]) ),(0,0,255),2)

        encoded, send_buffer = cv2.imencode('.jpg',img_to_send,[int(cv2.IMWRITE_JPEG_QUALITY), 50])
        try:
            reply = sender.send(send_buffer)
        except zmq.error.ZMQError as err:
            print("error: ",err)
        print("reply: ", reply)
        #imgMsg = bridge.cv2_to_compressed_imgmsg(img_to_send, dst_format = 'jpg')

        #imgMsg = bridge.cv2_to_compressed_imgmsg(cv2.cvtColor(img_to_send,cv2.COLOR_BGR2RGB), dst_format = 'jpg')
        imgMsg = bridge.cv2_to_compressed_imgmsg(img_to_send, dst_format = 'jpg')
        imgPublisher.publish(imgMsg)
        #f.write(img_to_send)
        ###for i,bb in reversed(iob):
        ###        x_min, y_min, x_max, y_max = bb[0], bb[1], bb[2], bb[3]
        ###        bgravg = np.mean(np.mean(img_to_send[y_min:y_max,x_min:x_max,:],axis=0),axis=0)
        ###        color = bgravg[2] <= bgravg[1] #red<green
        ###        if color:
        ###            cv2.rectangle(img_to_send,(x_min,y_min),(x_max,y_max),(0,255,0),2)
        ###        else:
        ###            cv2.rectangle(img_to_send,(x_min,y_min),(x_max,y_max),(0,0,255),2)
        ###        #cv2.imwrite('img'+str(image_counter)+'_'+str(i)+'.jpg',img_to_send[y_min:y_max,x_min:x_max,:])

        toc = time.time()
        curr_fps = 1.0 / (toc - tic)
        # calculate an exponentially decaying average of fps number
        fps = curr_fps if fps == 0.0 else (fps*0.95 + curr_fps*0.05)
        tic = toc

        if args.show:
            cv2.imshow(WINDOW_NAME, img_to_send)
            key = cv2.waitKey(1)
            if key == 27:  # ESC key: quit program
                break
            elif key == ord('F') or key == ord('f'):  # Toggle fullscreen
                full_scrn = not full_scrn
                set_display(WINDOW_NAME, full_scrn)
            elif key == ord('I') or key == ord('i'):  # Toggle fullscreen
                infer = not(infer)
            elif key == ord('U') or key == ord('u'):
                brightness += 10
                gamma_adjust(brightness/50)
            elif key == ord('D') or key == ord('d'):
                brightness -= 10
                gamma_adjust(brightness/50)
            elif key == ord('G') or key == ord('g'):
                greenshift+=5
            elif key == ord('R') or key == ord('r'):
                greenshift-=5
        rrate.sleep()
        
    #f.close()

    #cam.release()

def main():
    global infer
    args = parse_args()
    if args.infer:
        infer=True
    if args.category_num <= 0:
        raise SystemExit('ERROR: bad category_num (%d)!' % args.category_num)
    if not os.path.isfile('yolo/%s.trt' % args.model):
        raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % args.model)

    rospy.init_node("Camera")

    #cam = Camera(args)
    #if not cam.isOpened():
    #    raise SystemExit('ERROR: failed to open camera!')

    cls_dict = get_cls_dict(args.category_num)
    vis = BBoxVisualization(cls_dict)
    trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)

    open_window(
        WINDOW_NAME, 'Camera TensorRT YOLO Demo',
        None, None)
    loop_and_detect(None, trt_yolo, conf_th=0.3, vis=vis, args = args)

    cam.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
