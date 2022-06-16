import imagezmq
import socket
import zmq
import cv2

class ImageSender:

    def __init__(self, target_ip, compressed = True):
        self.host_name = socket.gethostname()
        print(self.host_name)
        #self.sender = imagezmq.ImageSender(connect_to='tcp://'+target_ip+':5555', REQ_REP=False)
        self.sender = imagezmq.ImageSender(connect_to='tcp://*:4949', REQ_REP=False)
        #self.sender.zmq_socket.setsockopt(zmq.RCVTIMEO,5000)
        self.compressed = compressed

    def send(self,image):
        ret = "no reply"
        try:
            if self.compressed:
                ret = self.sender.send_jpg(self.host_name, image)
            else:
                ret = self.sender.send_image(self.host_name, image)
        except:
            pass
        return ret


#img_sender = ImageSender('localhost')
#img_sender.compressed = False
#cap = cv2.VideoCapture(0)
#while True:
#    ret, frame = cap.read()
#    if ret:
#        cv2.imshow('to_send', frame)
#        try:
#            img_sender.send(frame) 
#        except zmq.error.Again:
#            print("timeout")
#        except zmq.error.ZMQError as err:
#            print("someothererror ", err)
#        keypress = cv2.waitKey(10)
#        if keypress == ord('q'):
#            break
