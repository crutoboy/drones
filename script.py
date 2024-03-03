import rospy
import math
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

rospy.init_node('flight')
bridge = CvBridge()
image_pub = rospy.Publisher('~red_obj', Image, queue_size=1)

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)



def navigate_wait(x=0, y=0, z=1, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=True, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)
# red (0, 255, 255), (0, 255, 255)
def check_clolor(img, img_hsv, range1, range2, color):
    try:
        mask = cv2.inRange(img_hsv, range1, range2)
        cnt = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        mom = cv2.moments(mask)
        center_x = int(mom["m10"] / mom["m00"])
        center_y = int(mom["m01"] / mom["m00"])

        cv2.circle(img, (center_x, center_y), 3, (255, 255, 255), -1)
        cv2.drawContours(img, cnt, -1, (0, 255, 0), 3)
        cv2.putText(img, color, (center_x - 25, center_y - 25),  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    except:
        ...
    return img

def publish():
    img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img = check_clolor(img, img_hsv, (0, 255, 255), (0, 255, 255), 'old_red') # old_red

    img = check_clolor(img, img_hsv, (80, 10, 140), (120, 50, 180), 'blue')
    img = check_clolor(img, img_hsv, (100, 100, 110), (140, 140, 150), 'fiolet')
    img = check_clolor(img, img_hsv, (140, 140, 130), (180, 190, 180), 'pink')
    img = check_clolor(img, img_hsv, (0, 130, 135), (0, 180, 190), 'red')
    img = check_clolor(img, img_hsv, (45, 220, 110), (100, 255, 155), 'green')


    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))


navigate_wait(frame_id='body')
publish()
navigate_wait(x=1, y=1)
publish()
navigate_wait(x=0, y=0)
publish()
land()