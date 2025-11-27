import rospy                                                 # type: ignore
from clover import srv                                       # type: ignore
from std_srvs.srv import Trigger                             # type: ignore
from std_msgs.msg import Int16                               # type: ignore
from clover import long_callback                             # type: ignore
import cv2 as cv                                             # type: ignore 
from sensor_msgs.msg import Image, CameraInfo                # type: ignore
from cv_bridge import CvBridge                               # type: ignore
from geometry_msgs.msg import PointStamped, Point, PoseArray, Pose # type: ignore
from mavros_msgs.srv import CommandLong                      # type: ignore
import tf2_ros                                               # type: ignore
import tf2_geometry_msgs                                     # type: ignore
import image_geometry                                        # type: ignore
import math                                                  # type: ignore
import numpy as np                                           # type: ignore


rospy.init_node('flight')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)
set_altitude = rospy.ServiceProxy('set_altitude', srv.SetAltitude)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
cmd = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
land = rospy.ServiceProxy('land', Trigger)

image_pub = rospy.Publisher('result', Image, queue_size=1)
map_pub = rospy.Publisher("tubes_map", Image, queue_size=1)

points_pub = rospy.Publisher("tubes", PoseArray, queue_size=10)
msg = PoseArray()
msg.header.stamp = rospy.Time.now()
msg.header.frame_id = "aruco_map"

bridge = CvBridge()
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
camera_model = image_geometry.PinholeCameraModel()
camera_model.fromCameraInfo(rospy.wait_for_message('main_camera/camera_info', CameraInfo))

yellow_low = (78, 220, 220)
yellow_up = (86, 228, 228)
kernel_size = (6, 6) 
kernel = cv.getStructuringElement(cv.MORPH_RECT, kernel_size)
vrezki = []

aruco_map = cv.imread("/home/clover/aruco_map.png")
yellow = (0, 255, 255)
red = (0, 0, 255)

not_line_count = 0


def navigate_wait(x=0, y=0, z=0, yaw=math.radians(90), speed=1, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


def draw_map(x, y, col, img, radius):
    cx = 70 + 52 * x
    cy = 426 - 52 * y
    img = cv.circle(img, (round(cx), round(cy)), radius, col, -1)
    return img



def get_cords(xy, z, msg): # Image msg, xy point from cam
    xy_rect = camera_model.rectifyPoint(xy)
    ray = camera_model.projectPixelTo3dRay(xy_rect)
    pnt = Point(x=ray[0] * z, y=ray[1] * z, z=z)
    target = PointStamped(header=msg.header, point=pnt)
    pnt_aruco = tf_buffer.transform(target, 'aruco_map', timeout=rospy.Duration(0.2))
    return pnt_aruco



def follow_line(bin):
    M = cv.moments(bin) #line following
    
    if M["m00"] != 0:
        x = int(M["m10"] / M["m00"])
        y = int(M["m01"] / M["m00"])
        
        dx = x - 160
        dy = 120 - y
        angle_rad = np.arctan2(dy, dx)
        yaw_error = angle_rad - 1.58
        
        set_yaw(yaw=yaw_error, frame_id='body')
        set_velocity(vx=0.2, vy=0, vz=0, frame_id='body')

    else:
        x, y = 0, 0
    
    return x, y


@long_callback
def image_callback(msg):
    global vrezki, not_line_count, aruco_map

    img = bridge.imgmsg_to_cv2(msg, 'bgr8') [0:120, 0:320]
    bin = cv.inRange(img, yellow_low, yellow_up)

    if cv.countNonZero(bin) > 10:
        img_eroded = cv.erode(bin, kernel, iterations=2)#Сужаем все линии на картинке чтобы "Отлипить" врезки от трубы
        contours, _ = cv.findContours(img_eroded, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv.contourArea) #Находим контур основной трубы(самый большой)
            line_mask = np.zeros_like(bin)#Пустая маска
            cv.drawContours(line_mask, [largest_contour], -1, 255, -1)#Создаем основную маску,на которой только труба

            line_mask = cv.dilate(line_mask, kernel, iterations=2)#Возвращаем суженную трубу к прежним размерам
            contours, _ = cv.findContours(line_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) 
            img = cv.drawContours(img, contours, -1, 255, -1)#Визуализация нахождения основной трубы

            line_mask_inv = cv.bitwise_not(line_mask) #"НЕ" с маской основной трубы
            vrezki_mask = cv.bitwise_and(line_mask_inv, bin)# с помощью "И" находим совпадения врезок на 
            contours, _ = cv.findContours(vrezki_mask, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

            for c in contours:
                x, y, w, h = cv.boundingRect(c)
                area = w*h
                if area > 400 and w > 60:
                    vrezka = get_cords((x, y), 1.2, msg) 
                    cx = vrezka.point.x
                    cy = vrezka.point.y
                    point = np.array([cx, cy])
                    if all(np.linalg.norm(point - pnt) >= 0.75 for pnt in vrezki):
                        print(f"Vrezka at x={round(cx, 2)}; y={round(cy, 2)}")
                        vrezki.append(point)
                        data_pub(point)
                        aruco_map = draw_map(cx, cy, red, aruco_map, 1)
            
                    img = cv.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)


            x, y = follow_line(line_mask)
            telem = get_telemetry(frame_id='aruco_map')
            img = cv.line(img, (160, 120), (x, y), (0, 0, 255), 2)
            img = cv.circle(img, (x, y), 5, (0, 0, 255), -1)
            aruco_map = draw_map(telem.x, telem.y, yellow, aruco_map, 4)

    else:
        not_line_count +=1
        if not_line_count >= 20:
            image_sub.unregister()
            navigate_wait(0, 0, 1.2)
            land()
            rospy.on_shutdown()

    map_pub.publish(bridge.cv2_to_imgmsg(aruco_map, 'bgr8'))
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))


def data_pub(pnt):
    x = pnt[0]
    y = pnt[1]
    pnt = Pose()
    pnt.position.x = x
    pnt.position.y = y
    pnt.position.z = 0
    msg.poses.append(pnt)
    points_pub.publish(msg)



def main():
   navigate_wait(0, 0, 1, frame_id='body', auto_arm=True)
   navigate_wait(0.7, 0.7, 1.2)
   set_altitude(z=1.2, frame_id='terrain')


if __name__ == '__main__':
    main()
    image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)
    rospy.spin()
