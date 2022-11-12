#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped ,TwistStamped, Twist, Pose
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy as np
from math import sqrt
import matplotlib
import matplotlib.pyplot as plt
from function_1 import print_inf
import math

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from MPC import acados_settinngs, linear_quad_model, run_solver
current_state = State()
pos_1 = PoseStamped()
V_1 = TwistStamped()
IMU_data = Imu()

def state_cb(msg):
    global current_state
    current_state = msg



def position_cb(msg):
    global pos_1
    pos_1 = msg

def V_cb(msg):
    global V_1
    V_1 = msg

def Imu_cb(msg):
    global IMU_data
    IMU_data = msg

def refv_cb(msg):
    global refv
    refv = msg

def refp_cb(msg):
    global refp
    refp = msg


def euler_from_quaternion(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians






if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    ananana = rospy.Subscriber("mavros/imu/data", Imu, callback = Imu_cb)
    bbabbba = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = position_cb)
    uiuyuf = rospy.Subscriber("mavros/global_position/raw/gps_vel", TwistStamped, callback = V_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    setvel = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
    ffaa = rospy.Subscriber("reference", Twist, callback = refv_cb)
    wwss = rospy.Subscriber("reference2", Pose, callback = refp_cb)


    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    # set MPC 
    t_horizon = 5
    N = 20 # number of optimization nodes until time horizon
    model = linear_quad_model()
    acados_solver = acados_settinngs(t_horizon = t_horizon,N=N)
    

    # Setpoint publishing MUST be faster than 2Hz
    Hz = 20
    rate = rospy.Rate(Hz)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 2
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 3

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    k=0
    Z_last = 0
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()
        


        roll,pitch,yaw = euler_from_quaternion(pos_1.pose.orientation.x,
            pos_1.pose.orientation.y,pos_1.pose.orientation.z,pos_1.pose.orientation.w)
        Xc = pos_1.pose.position.x
        Yc = pos_1.pose.position.y
        Zc = pos_1.pose.position.z
        Vxc = V_1.twist.linear.x
        Vyc = V_1.twist.linear.y
        Vzc = (Zc - Z_last)*Hz
        Wxc = IMU_data.angular_velocity.x
        Wyc = IMU_data.angular_velocity.y
        Wzc = IMU_data.angular_velocity.z
        Z_last = Zc

        current_states = [Xc, Yc, Zc] + [ roll, pitch, yaw] + [Vxc, Vyc, Vzc] + [Wxc, Wyc, Wzc]
        if (k < 350):
            local_pos_pub.publish(pose)
            k=k+1
        else:
            
            
            x_target = [[1, 3, 1], [ 0, 0, 0], [0, 0, 0], [0, 0, 0]]
            
            vx,vy,vz,p,q,r = run_solver(N=N,model=model,
            acados_solver=acados_solver,initial_state = current_states,
            x_target=x_target)
            vel.linear.x = vx
            vel.linear.y = vy
            vel.linear.z = vz
            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = 0
            '''
            print('p is : ')
            print(p)
            print('q is : ')
            print(q)
            print('r is : ')
            print(r)
            '''
        

            setvel.publish(vel)
            
            print(refv)
            print(refp)
        rate.sleep()
        '''
        print('k is : ')
        print(k)
        #print_inf(pos_1,k,V_1,IMU_data)
        print('x is:')
        print(pos_1.pose.position.x)
        print('y is:')
        print(pos_1.pose.position.y)
        print('z is:')
        print(pos_1.pose.position.z)
        print('roll pitch yaw')
        print(roll)
        print(pitch)
        print(yaw)
        print('Vx Vy Vz')
        print(V_1.twist.linear.x)
        print(V_1.twist.linear.y)
        print(Vzc)
        print(IMU_data.angular_velocity.x)
        print(IMU_data.angular_velocity.y)
        print(IMU_data.angular_velocity.z)
        '''


