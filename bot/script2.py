import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
import telebot
from telebot import types

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


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
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


bot = telebot.TeleBot('5156482363:AAHNbeK4wP2BsB2Th3bAQepJbMmO-jXSuaI')

@bot.message_handler(commands=['start'])
def start(message):
    bot.send_message(message.from_user.id, '/takeoff - взлёт\n/land - посадка\n/flight x y - полёт по координатам, обязательно указать x и y')

@bot.message_handler(commands=['takeoff'])
def takeoff(message):
    bot.send_message(message.from_user.id, 'взлёт')
    navigate_wait(z=1, frame_id='body', auto_arm=True)
    bot.send_message(message.from_user.id, 'взлёт завершён')

@bot.message_handler(commands=['land'])
def land_t(message):
    bot.send_message(message.from_user.id, 'посадка')
    land_wait()
    bot.send_message(message.from_user.id, 'посадка окончена')

@bot.message_handler(commands=['flight'])
def flight2position(message):
    bot.send_message(message.from_user.id, message.text)
    try:
        _, x, y = message.text.split(' ')
        bot.send_message(message.from_user.id, f'летит на корды x: {x} y: {y}')
        navigate_wait(x=int(x), y=int(y), z=1, frame_id='aruco_map')
        bot.send_message(message.from_user.id, f'прилетел на корды x: {x}, y: {y}')
    except:
        bot.send_message(message.from_user.id, 'https://www.youtube.com/watch?v=dQw4w9WgXcQ')

bot.infinity_polling()