#!/usr/bin/env python3

import numpy as np
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist , PoseStamped
import smach

import ros_numpy
from utils_evasion import *
import tf2_ros


########## Functions for takeshi states ##########
Meta = PoseStamped

def get_coords ():
    for i in range(10):   ###TF might be late, try 10 times
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            return trans
        except:
            print ('waiting for tf')
            trans=0
            

def move_base_vel(vx, vy, vw):
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = vw 
    base_vel_pub.publish(twist)

def move_base(x,y,yaw,timeout=5):
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < timeout:  
        move_base_vel(x, y, yaw)


def move_forward():
    move_base(0.15,0,0,3)
def move_backward():
    move_base(-0.15,0,0,1.5)
def turn_left():
    move_base(0.0, 0.0, 0.18*np.pi, 3)
def turn_right():
    move_base(0.0,0,-0.18*np.pi, 3)

def get_lectura_cuant():
    try:
        lectura=np.asarray(laser.get_data().ranges)
        lectura=np.where(lectura>20,20,lectura) #remove infinito

        right_scan=lectura[:300]
        left_scan=lectura[300:]
        ront_scan=lectura[300:360]

        sd, si, sf = 0, 0, 0
        if np.mean(left_scan) < 3: si = 1
        if np.mean(right_scan) < 3: sd = 1
        if np.mean(ront_scan) < 1: sf = 1

    except:
        sd, si, sf = 0, 0, 0

    return si, sd, sf

#Establece el estado
def setOutcome():
    si, sd, sf = get_lectura_cuant()
    outcomeCase = 'outcome'

    if(si == 0 and sd == 0):
        move_forward()
        outcomeCase = 'outcome1'

    if (si==0 and sd==1):
        move_forward()
        outcomeCase = 'outcome1'

    if (si==1 and sd==0):
        move_forward()
        outcomeCase = 'outcome1'
                    
    if (si==1 and sd==1):
        outcomeCase = 'outcome2'
    
    return outcomeCase, sf

##### Define state INITIAL #####

class Inicio (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ','fail']) #shor for success
        

    def execute(self,userdata):
    	# Aqui va lo que se desea ejecutar en el estado
        global Meta

        print('inicializando')

        Meta = rospy.wait_for_message('meta_competencia', PoseStamped)
        ########
        
        rospy.sleep(1)#### dar tiempo al arbol de tfs de publicarse
        
        punto_inicial = get_coords()
        print ( 'tiempo = '+ str(punto_inicial.header.stamp.to_sec()), punto_inicial.transform.translation)
        print("Meta: ", Meta.pose.position)

        turn_right()

        return 'succ'

#Estado para avanzar en Y
class S1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2', 'succ'])
        self.counter = 0
        

    def execute(self,userdata):
        global Meta
        
        puntoActual = get_coords()
        outcomeCase, sf = setOutcome()

        print('robot Estado S_1')
        print('Actual --> Y: ', np.abs(puntoActual.transform.translation.y))
        print('Meta ---> Y: ', np.abs(Meta.pose.position.y))

        #Si no detecta colicion avanza a la cordenada
        if(sf == 0):
            #Estableciendo un rango de movimiento
            if(np.abs(puntoActual.transform.translation.y) < np.abs(Meta.pose.position.y)):
                print('llendo al punto en Y')
                return outcomeCase
                    
            if(np.abs(puntoActual.transform.translation.y) >= np.abs(Meta.pose.position.y) and np.abs(puntoActual.transform.translation.x) < np.abs(Meta.pose.position.x)):
                print("Llendo al punto en X")
                turn_right()
                return 'outcome2'
            else:
                print('EL robot llego a su destino\nPunto Final: \n', puntoActual.transform.translation, 'Meta: \n', Meta.pose.position)
                return 'succ'
        else:
            if(np.abs(puntoActual.transform.translation.x) < np.abs(Meta.pose.position.x)):
                print("Colicion detectada, llendo al segundo punto en x")
                turn_right()
                return 'outcome2'
            else:
                print('EL robot llego a su destino\nPunto Final: \n', puntoActual.transform.translation, 'Meta: \n', Meta.pose.position)
                return 'succ'

#Estado para avanzar en X
class S2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2', 'succ'])

    def execute(self,userdata):
        global Meta

        puntoActual = get_coords()
        outcomeCase, sf = setOutcome()

        print('robot Estado S_2')
        print('Actual --> X: ', np.abs(puntoActual.transform.translation.x))
        print('Meta ---> X: ', np.abs(Meta.pose.position.x))

        if(sf == 0):
            if(np.abs(puntoActual.transform.translation.x) <= np.abs(Meta.pose.position.x)):
                print('Llendo al punto en X')
                return outcomeCase

            else:
                if(np.abs(puntoActual.transform.translation.y) >= np.abs(Meta.pose.position.y)):
                    print('EL robot llego a su destino\nPunto Final: \n', puntoActual.transform.translation, 'Meta: \n', Meta.pose.position)
                    return 'succ'
                else:
                    print("Llendo al punto en Y")
                    turn_left()
                    return 'outcome2'
        else:
            print("Llendo al punto en Y")
            turn_left()
            return 'outcome2'


def init(node_name):
    global laser, base_vel_pub
    rospy.init_node(node_name)
    base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
    laser = Laser()
      

#Entry point
if __name__== '__main__':

    print("STATE MACHINE...")
    init("smach_node")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    with sm:
        #State machine for evasion
        smach.StateMachine.add("INICIO",   Inicio(),  transitions = {'fail':'END', 'succ':'s_1'})
        smach.StateMachine.add("s_1",   S1(),  transitions = {'outcome1':'s_1','outcome2':'s_2','succ':'END'})
        smach.StateMachine.add("s_2",   S2(),  transitions = {'outcome1':'s_2','outcome2':'s_1','succ':'END'})
        

outcome = sm.execute()