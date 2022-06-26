import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node("cam_record_py")

video_file = "edgar_7_2" + ".avi"
out = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30, (640, 480))

def camera_topic_callback(image_msg):
    global out
    print('subscribe_success')
    np_arr = np.fromstring(image_msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #     image_from_topic = image_msg
    # image_from_topic = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
    image_from_topic_received = True
    cv2.imshow("raw", image_np)
    out.write(image_np)
    cv2.waitKey(1)

    

    pass

rospy.Subscriber('/vision/image/compressed',CompressedImage,camera_topic_callback, queue_size=1)

rospy.spin()