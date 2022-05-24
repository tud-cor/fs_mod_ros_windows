from ast import Global
import rospy
import pygame
from geometry_msgs.msg import Twist

# These are the global variables used in this code
# ACC_SPEED is How quick it accelerates
# ACC_SPEED=.1
# Max_Speed is the Max Speed for x velocity
# Manual Control is a boolean for if the controller is controlling
# PrevState is true if the switch was hit last loop so control dosen't switch (Needs to change to be numeric cause of debouncing)
# PubTwist is the message manual control sends
MAX_SPEED=1
ManualControl=False
PrevState=False
DebounceLoop=3

# This function returns true if it should change control 
# Atm it returns true if he flappy button pressed
def ChangeControl():
    return joysticks[0].get_button(4)

# this methods checks if swithc has been toggled
def SwitchCheck():
    global MAX_SPEED,ManualControl, PrevState, DebounceLoop
    if ChangeControl():
        if(PrevState>DebounceLoop):
            print("Switch")
            ManualControl= not ManualControl
            PrevState=0
    PrevState+=1

# This is the callback for the rospy timer
# It calls Switch Check to see if control has been toggled and 
# It publish a twist message if in manual control

def Manual_CallBack(event):
    pygame.event.get()
    global ManualControl
    PubTwist = Twist()
    SwitchCheck()
    if ManualControl:
        PubTwist.linear.x=0
        if joysticks[0].get_button(7) and joysticks[0].get_button(6):
            print("Stop")
        elif joysticks[0].get_button(6):
            PubTwist.linear.x=MAX_SPEED
        elif joysticks[0].get_button(7):
            PubTwist.linear.x=-MAX_SPEED
        PubTwist.angular.z=-joysticks[0].get_axis(0)
        pub.publish(PubTwist)

# This is a call back for cmd_vel and publish twist message if not in manual control
def  Nav_Callback(data):
    global ManualControl
    if ManualControl==False:
        pub.publish(data)

# Here the ros node is created as well as the ros timer and ros subscriber
def RosControl():
    rospy.init_node('cml_vel_subscriberGame', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, Nav_Callback)
    rospy.Timer(rospy.Duration(.8), Manual_CallBack)
    rospy.spin()
    
if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel_game',Twist,queue_size=5)
    pygame.init()
    joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
    for joy in joysticks:
        joy.init()
    RosControl()