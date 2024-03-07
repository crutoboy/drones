import rospy
from clover import srv
from std_srvs.srv import Trigger
import numpy as np
import math
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

bridge = CvBridge()

lower_red = np.array([169,168,135])
upper_red = np.array([180,255,255])

lower_red1 = np.array([0,50,50])
upper_red1 = np.array([7,255,255])

lower_blue = np.array([96,130,76])
upper_blue = np.array([122,255,255])

lower_yellow = np.array(33,34,49])
upper_yellow = np.array([30,255,255])

lower_green = np.array([40,25,27])
upper_green = np.array([74,255,255])

def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    img = img[80:160, 100:180]
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask_blue = cv.inRange(hsv, lower_blue, upper_blue)
    mask_red1 = cv.inRange(hsv, lower_red1, upper_red1)
    mask_red = cv.inRange(hsv, lower_red, upper_red)
    mask_red = cv.bitwise_or(mask_red1, mask_red)
    mask_green = cv.bitwise_or(lower_green, upper_green)
    mask_yellow = cv.bitwise_or(lower_yellow, upper_yellow)
    M = cv.moments(mask_red)
    if M['m00'] >= 800:
        print('red')
        subscriber.unregister()
    M_b = cv.moments(mask_blue)
    if M_b['m00'] >= 800:
        print('blue')
        subscriber.unregister()
    M_g = cv.moments(mask_green)
    if M_g['m00'] >= 800:
        print('blue')
        subscriber.unregister()
    M_y = cv.moments(mask_yellow)
    if M_y['m00'] >= 800:
        print('blue')
        subscriber.unregister()
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)
navigate_wait(z=1.5, frame_id='body', auto_arm=True)

navigate_wait(x=0, y=0.5, z=1.5, frame_id='aruco_map')
navigate_wait(x=2, y=3, z=1.5, frame_id='aruco_map')

def flip():
    angle = math.pi / 1.8 #!!!!!!
    start = get_telemetry()  # memorize starting position

    set_rates(thrust=1)  # bump up
    rospy.sleep(0.3) #!!!!!!

    set_rates(pitch_rate=30, thrust=0.35)  # pitch flip #!!!!!
    # set_rates(roll_rate=30, thrust=0.2)  # roll flip

    while True:
        telem = get_telemetry()
        flipped = abs(telem.pitch) > angle or abs(telem.roll) > angle
        if flipped:
            break

    rospy.loginfo('finish flip')
    set_position(x=start.x, y=start.y, z=start.z, yaw=start.yaw)  # finish flip

navigate(z=2, speed=1, frame_id='body', auto_arm=True)  # take off
rospy.sleep(10)

rospy.loginfo('flip')
flip()

navigate_wait(x=0, y=0, z=0, frame_id='aruco_map')
land()


