import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
import numpy as np
from threading import Lock

rospy.init_node('flight')

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
bridge = CvBridge()
telem_lock = Lock()



def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def get_telemetry_locked(frame_id):
    with telem_lock:
        return get_telemetry(frame_id)

lower_blue = np.array([107,50,50])
upper_blue = np.array([130,255,255])

lower_red = np.array([0,50,50])
upper_red = np.array([5,255,255])

lower_red1 = np.array([172,50,50])
upper_red1 = np.array([180,255,255])

arr = []
size_flag = []
check_find = False

@long_callback
def image_callback(data):
    global arr

    img = bridge.imgmsg_to_cv2(data, 'bgr8') 
    img = img[80:160, 100:180]
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    mask_blue = cv.inRange(hsv, lower_blue, upper_blue)
    mask_red = cv.inRange(hsv, lower_red, upper_red)
    mask_red1 = cv.inRange(hsv, lower_red1, upper_red1)
    mask_red = cv.bitwise_or(mask_red, mask_red1)
    telemetry = get_telemetry_locked(frame_id='aruco_map')

    
    
    

    

    M = cv.moments(cv.bitwise_or(mask_red, mask_blue))
    if M['m00'] > 100: #we make a list of all coords when the drone sees flags
        arr.append([telemetry.x, telemetry.y])

    if check_find: #if we already have fixed coordinates of each flag
        ret_blue, thresh_blue = cv.threshold(mask_blue,127,255,0)
        contours_blue, hierarchy_blue = cv.findContours(thresh_blue, 1, 2)
        ret_red, thresh_red = cv.threshold(mask_red,127,255,0)
        contours_red, hierarchy_red = cv.findContours(thresh_red, 1, 2)

        cnt = contours_blue[0] #for blue
        x,y,w,h = cv.boundingRect(cnt)
        M = cv.moments(mask_blue)
        cx_blue = int(M['m10']/M['m00'])
        cy_blue = int(M['m01']/M['m00'])

        M1 = cv.moments(mask_red)
        cnt_red =  contours_red[0] #for red
        x,y,w,h = cv.boundingRect(cnt_red)
        cx_red = int(M1['m10']/M1['m00'])
        cy_red = int(M1['m01']/M1['m00'])

        if abs(math.sqrt((cx_red - cx_blue)**2 + (cy_red - cy_blue)**2) - min(w, h)) < min(w, h) // 2:
            print('russia')
        else:
            print('france')
    
    image_pub.publish(bridge.cv2_to_imgmsg(cv.bitwise_or(mask_blue, mask_red), 'mono8'))

image_pub = rospy.Publisher('~debug', Image)
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

navigate_wait(z=1, frame_id='body', auto_arm=True)


navigate_wait(z=1, frame_id='aruco_0')


#finding exact coordinates
final_cords = []
cou = 0
prx = arr[0][0]
pry = arr[0][1]
sum_x = 0
sum_y = 0
for x, y in arr:
    if abs(prx - x) >= 0.5 or abs(pry - y) >= 0.5:
        final_cords.append([sum_x / cou, sum_y / cou])
        prx = x
        pry = y
        sum_x = 0
        sum_y = 0
        cou = 0
    else:
        prx = x
        pry = y
        sum_x += x
        sum_y += y
        cou += 1
final_cords.append([sum_x / cou, sum_y / cou])
print(final_cords)
#check_find = True

for x, y in final_cords:
    print(x, y)
    navigate_wait(x=x, y=y, z=1)
    rospy.sleep(2)
    check_find = True
    rospy.sleep(4)
    check_find = False
navigate_wait(x=0, y=0, z=1)
land()