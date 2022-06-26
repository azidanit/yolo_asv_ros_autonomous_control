import csv
import rospy

from std_msgs.msg import Float64
from vision_msgs.msg import BoundingBox2DArray

# open the file in the write mode
f = open('edgar_5_2.csv', 'w')

# create the csv writer
writer = csv.writer(f)

rospy.init_node("recorde_py")

err_angle = 0
err_dist = 0
bb_msg = BoundingBox2DArray()

def cbMsgAngle(data):
    global err_angle
    err_angle = data.data
    pass

def cbMsgDist(data):
    global err_dist
    err_dist = data.data / 10
    pass

def cbMsgVision(data):
    global bb_msg
    bb_msg = data
    pass

rospy.Subscriber("/waypoint_control/error/angle", Float64, cbMsgAngle)
rospy.Subscriber("/waypoint_control/error/distance", Float64, cbMsgDist)
rospy.Subscriber("/vision/objects", BoundingBox2DArray, cbMsgVision)
# write a row to the csv file
rr = rospy.Rate(50)

while not rospy.is_shutdown():

    if len(bb_msg.boxes) > 0:
        center_x = bb_msg.boxes[0].center.x
        center_y = bb_msg.boxes[0].center.y
    else:
        center_x = 0
        center_y = 0

    data_row = [err_angle, err_dist, center_x, center_y]
    print(data_row)
    writer.writerow(data_row)
    err_angle = 0
    err_dist = 0
    rr.sleep()



# close the file
f.close()