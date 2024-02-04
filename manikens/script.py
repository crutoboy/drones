import rospy
from clover import srv
from std_srvs.srv import Trigger
from clover.srv import SetLEDEffect
import math
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


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


color_dict = {
    'red': (255, 0, 0),
    'orange': (255, 165, 0),
    'yellow_orange': (255, 255, 0),
    'yellow': (255, 255, 0),
    'yellow_green': (173, 255, 47),
    'green': (0, 255, 0),
    'blue_green': (0, 255, 127),
    'blue': (0, 0, 255),
    'blue_violet': (138, 43, 226),
    'violet': (148, 0, 211),
    'red_violet': (199, 21, 133),
}
filename = '/home/clover/Desktop/raport.txt'
# count_person = 0
persons = []

def navigate_wait(x=0, y=0, z=1, yaw=float('nan'), speed=1, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

def get_position():
    telem = get_telemetry()
    return telem.x, telem.y

def get_distance(pos1:tuple, pos2:tuple):
    return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[0] - pos2[1]) ** 2)

def check_repeate(x, y, color) -> bool:
    global persons
    for per in persons:
        if per[0] != color: continue
        if get_distance(per[1:], (x, y)) < 0.2: break
    else:
        return False
    return True

def image_colback_color():
    global persons
    img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    # img = bridge.imgmsg_to_cv2(data, 'bgr8') # OpenCV image
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #[118:119,158:159]

    # yellow_orange_low = (38, 110, 150)
    # yellow_orange_high= (52, 110, 150)

    # brown_low = (23, 50, 50)
    # brown_high= (37, 50, 50)

    # color = 'undefined'

    # if cv2.inRange(img_hsv, yellow_orange_low, yellow_orange_high)[119][159] == 255:
    #     color = 'yellow_orange'
    # elif cv2.inRange(img_hsv, brown_low, brown_high)[119][159] == 255:
    #     color = 'brown'

    color = 'undefined'
    
    # Red
    red_low1 = (0, 110, 150)
    red_high1 = (7, 255, 255)
    
    red_low2 = (172, 110, 150)
    red_high2 = (180, 255, 255)
    
    red_orange_low = (8, 110, 150)
    red_orange_high = (22, 110, 150)
    
    if cv2.inRange(img_hsv, red_low1, red_high1)[119][159] == 255 or cv2.inRange(img_hsv, red_low2, red_high2)[119][159] == 255 or cv2.inRange(img_hsv, red_orange_low, red_orange_high)[119][159] == 255:
        color = 'red'
    
    # Orange
    orange_low = (23, 110, 150)
    orange_high = (37, 110, 150)
    
    if cv2.inRange(img_hsv, orange_low, orange_high)[119][159] == 255:
        color = 'orange'
    
    # Yellow Orange
    yellow_orange_low = (38, 110, 150)
    yellow_orange_high = (52, 110, 150)
    
    if cv2.inRange(img_hsv, yellow_orange_low, yellow_orange_high)[119][159] == 255:
        color = 'yellow_orange'
    
    # Yellow
    yellow_low = (53, 150, 150)
    yellow_high = (67, 255, 255)
    
    if cv2.inRange(img_hsv, yellow_low, yellow_high)[119][159] == 255:
        color = 'green'
    
    # Yellow Green
    yellow_green_low = (68, 150, 150)
    yellow_green_high = (82, 255, 255)
    
    if cv2.inRange(img_hsv, yellow_green_low, yellow_green_high)[119][159] == 255:
        color = 'yellow_green'
    
    # Green
    green_low = (83, 150, 150)
    green_high = (97, 255, 255)
    
    if cv2.inRange(img_hsv, green_low, green_high)[119][159] == 255:
        color = 'green'
    
    # Blue Green
    blue_green_low = (98, 150, 150)
    blue_green_high = (113, 255, 255)
    
    if cv2.inRange(img_hsv, blue_green_low, blue_green_high)[119][159] == 255:
        color = 'blue_green'
    
    # Blue
    blue_low = (114, 150, 150)
    blue_high = (127, 255, 255)
    
    if cv2.inRange(img_hsv, blue_low, blue_high)[119][159] == 255:
        color = 'blue'
    
    # Blue Violet
    blue_violet_low = (128, 150, 150)
    blue_violet_high = (142, 255, 255)
    
    if cv2.inRange(img_hsv, blue_violet_low, blue_violet_high)[119][159] == 255:
        color = 'blue_violet'
    
    # Violet
    violet_low = (143, 150, 150)
    violet_high = (157, 255, 255)
    
    if cv2.inRange(img_hsv, violet_low, violet_high)[119][159] == 255:
        color = 'violet'
    
    # Red Violet
    red_violet_low = (158, 150, 150)
    red_violet_high = (171, 255, 255)
    
    if cv2.inRange(img_hsv, red_violet_low, red_violet_high)[119][159] == 255:
        color = 'red_violet'

    #detected color
    # print(color)
    # print(img[0][0])
    # print(img_hsv[0][0])
    if color != 'undefined':
        count_person = len(persons) + 1
        x, y = get_position()
        raport = f'person {count_person}: {color} {x} {y}'
        if check_repeate(x, y, color):
            print(raport + ' repeate')
            return None
        persons.append((color, x, y))
        # with open(filename, 'a') as file:
        #     file.write(raport + '\n')
        print(raport)
        color_rgb = color_dict[color]
        set_effect(r=color_rgb[0], g=color_rgb[1], b=color_rgb[2])

# image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_colback_color)

navigate_wait(z=1, frame_id='body', auto_arm=True)

for x in range(7*2-1):
    range_y = range(7*2-1)
    if x % 2 != 0:
        range_y = reversed(range_y)
    x *= 0.25
    x -= 0.5
    for y in range_y:
        y *= 0.25
        y -= 0.5
        navigate_wait(x=x, y=y, frame_id='aruco_map')
        rospy.sleep(1)
        image_colback_color()

# navigate_wait(x=0, y=1, frame_id='aruco_map')
# rospy.sleep(10)
navigate_wait(x=0, y=0, frame_id='aruco_map')
land()

with open(filename, 'w+') as file:
    for i, per in enumerate(persons):
        file.write(f'person {i+1}: {per[0]} {per[1]} {per[2]}\n')