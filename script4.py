import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from clover.srv import SetLEDEffect

rospy.init_node('flight')
bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_altitude = rospy.ServiceProxy('set_altitude', srv.SetAltitude)
set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)
set_yaw_rate = rospy.ServiceProxy('set_yaw_rate', srv.SetYawRate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)


def navigate_wait(x=0, y=0, z=1, yaw=float('nan'), speed=1, frame_id='aruco_map', auto_arm=True, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def detect_color():
    img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    yellow_mask = cv2.inRange(img_hsv, (60, 100, 100), (60, 100, 100))
    red_mask = cv2.inRange(img_hsv, (0, 100, 100), (0, 100, 100))
    green_mask = cv2.inRange(img_hsv, (120, 100, 50), (120, 100, 50))

    yellow_cnt = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
    red_cnt = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
    green_cnt = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
    print(yellow_cnt, red_cnt, green_cnt, sep='\n')

    if len(yellow_cnt) > 0:
        set_effect(r=255, g=255, b=0)
    if len(red_cnt) > 0:
        set_effect(r=255, g=0, b=0)
    if len(green_cnt) > 0:
        set_effect(r=0, g=255, b=0)


set_effect(effect='rainbow') 
navigate_wait(0, 0, 1, frame_id='body')
rospy.sleep(2)
navigate_wait(0, 0, 1)
detect_color()
land()