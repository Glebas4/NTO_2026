import rospy
from clover import srv
from std_srvs.srv import Trigger
from clover import long_callback
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import numpy as np

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_altitude = rospy.ServiceProxy('set_altitude', srv.SetAltitude)
set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
land = rospy.ServiceProxy('land', Trigger)
bridge = CvBridge()
image_pub = rospy.Publisher('binary', Image, queue_size=1)

yellow_low = (78, 220, 220)
yellow_up = (86, 228, 228)
kernel_size = (5, 5) 
kernel = cv.getStructuringElement(cv.MORPH_RECT, kernel_size)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=1, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


@long_callback
def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8') 
    bin = cv.inRange(img, yellow_low, yellow_up)
    
    img_morph = cv.morphologyEx(bin, cv.MORPH_OPEN, kernel, iterations=3)

    #contours, _ = cv.findContours(img_morph, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    #largest_contour = max(contours, key=cv.contourArea)
    #x, y, w, h = cv.boundingRect(largest_contour)
    
    M = cv.moments(img_morph)

    if M["m00"] != 0:
        x = int(M["m10"] / M["m00"])
        y = int(M["m01"] / M["m00"])
    else:
        x, y = 0, 0

    error = (160 - x) // 3.5
    set_yaw(yaw=math.radians(error), frame_id='body')  
    set_velocity(vx=0, vy=0.5, vz=0, frame_id='body')  

    img = cv.circle(img, (x, y), 5, (0, 0, 255), 1)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))


def main():
    navigate_wait(0, 0, 1, frame_id="body", auto_arm=True)
    navigate_wait(1, 1, 1)



if __name__ == '__main__':
    main()
    image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)
    rospy.spin()
