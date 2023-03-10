#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped ,TwistStamped, Twist, Pose
from mavros_msgs.msg import State ,HomePosition
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.srv import CommandHome, CommandHomeRequest, CommandHomeResponse
from mavros_msgs.srv import CommandHome
import numpy as np
from collections import deque
from math import sqrt
import math
import time
from MPC import acados_settinngs, linear_quad_model, run_solver,DT_gp_model, DT_linear_model, solve_DT_nextState, eight_trag, ellipse_trag
from scipy.spatial import distance
import threading
import os
from geometry_msgs.msg import Quaternion
source_dir = os.getcwd()
print('source_dir',source_dir)
os.chdir(source_dir +'/data')
os.system('rm -r *.out')
os.chdir(source_dir)

current_state = State()
pos_1 = PoseStamped()
V_1 = TwistStamped()
IMU_data = Imu()
home_pos = HomePosition()
def state_cb(msg):
    global current_state
    current_state = msg

def home_pos_cb(msg):
    global home_pos
    home_pos = msg

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

rospy.init_node("offb_node_py")
state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
ananana = rospy.Subscriber("mavros/imu/data", Imu, callback = Imu_cb)
bbabbba = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = position_cb)
uiuyuf = rospy.Subscriber("mavros/global_position/raw/gps_vel", TwistStamped, callback = V_cb)
local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
setvel = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
start_pub = rospy.Publisher("start_time", Bool, queue_size=10)
ffaa = rospy.Subscriber("reference", Twist, callback = refv_cb)
wwss = rospy.Subscriber("reference2", Pose, callback = refp_cb)
home_sub = rospy.Subscriber("mavros/home_position/home", HomePosition, callback = home_pos_cb)

rospy.wait_for_service("/mavros/cmd/arming")
arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
rospy.wait_for_service("/mavros/set_mode")
set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


set_home_service = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)


# Create a CommandHomeRequest object with the new home location
request = CommandHomeRequest()
request.latitude = 47.6199 # replace with your desired latitude
request.longitude = -122.3535 # replace with your desired longitude
request.altitude = 10.0 # replace with your desired altitude

# Wait for the mavros services to become available
rospy.wait_for_service('/mavros/cmd/set_home')



# init use dynamic model : 
solver_save= deque([0])
model_save = deque([0])
t_horizon = 1.2
N = 24

# Setpoint publishing MUST be faster than 2Hz
Hz = 20
rate = rospy.Rate(Hz)
# rate of change the model:
model_Hz = 1/20
discretization_dt = 0.05
v_average = 1.5 # 1.5 / 2 / 2.5 / 3
# set traj =================================================================================================================================
x_ref,y_ref,z_ref,t_ref,vx_ref,vy_ref,vz_ref,T_onecircle = eight_trag(speed = v_average,x_w = 4,y_w = 4,z_w = 0,H = 1.5,dT = discretization_dt ,sim_t = 60)
ref = np.zeros((x_ref.shape[0],12))
ref[::,0] = x_ref
ref[::,1] = y_ref
ref[::,2] = z_ref
ref[::,6] = vx_ref
ref[::,7] = vy_ref
ref[::,8] = vz_ref
# ==========================================================================================================================================

# set measurement , pridect , controls recording deque
# record x, y ,z, this is record for 3D plot
pos_record = deque([])
model_predict_record = deque([])
measurements_record = deque([])
controlinput_record = deque([])
pos_error_record = deque([])
ref_mes = deque([])

def main_control():
    # set MPC 
    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # call the CommandHome service and wait for the response
    response = set_home_service(request)

    # Check if the service call was successful
    while(response.success == 0):
        response = set_home_service(request)
    if response.success:
        print(';;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;')
        rospy.loginfo('New home location set successfully')
        print(';;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;')
        print(';;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;')
        print(';;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;')
    else:
        print('========================================')
        print('========================================')
        print('========================================')
        print('========================================')
        rospy.logerr('Failed to set new home location')

    # set publisher variables
    pose = PoseStamped()
    vel = Twist()
    flag = Bool()
    flag.data = 0
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0
    pose.pose.position.x = x_ref[0]
    pose.pose.position.y = y_ref[0]
    pose.pose.position.z = z_ref[0]

    # Send a few setpoints before starting
    for i in range(10):   
        if(rospy.is_shutdown()):
            break

        setvel.publish(vel)
        start_pub.publish(flag)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    last_req = rospy.Time.now()

    # set init variables
    k=0  # control flag
    X_last = 0
    Y_last = 0
    Z_last = 0
    # set move average window init 
    window_Vx = deque([0,0,0,0])
    window_Vy = deque([0,0,0,0])
    window_Vz = deque([0,0,0,0])

    # set init index and record array size
    iindex = 1
    control_input = np.zeros(4)
    current_states = np.zeros(12)


    # main loop 
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
        

        # measure states
        roll,pitch,yaw = euler_from_quaternion(pos_1.pose.orientation.x,
            pos_1.pose.orientation.y,pos_1.pose.orientation.z,pos_1.pose.orientation.w)
        import random
        Xc = pos_1.pose.position.x #+ random.randint(0,9) /100
        Yc = pos_1.pose.position.y
        Zc = pos_1.pose.position.z
        Vxc = (Xc - X_last)*Hz
        Vyc = (Yc - Y_last)*Hz
        Vzc = (Zc - Z_last)*Hz

        # FIFO V into buffer for low pass filter
        window_Vx.appendleft(Vxc)
        window_Vx.pop()
        window_Vy.appendleft(Vyc)
        window_Vy.pop()
        window_Vz.appendleft(Vzc)
        window_Vz.pop()

        Vx_mean = np.mean(window_Vx)
        Vy_mean = np.mean(window_Vy)
        Vz_mean = np.mean(window_Vz)

        Wxc = IMU_data.angular_velocity.x
        Wyc = IMU_data.angular_velocity.y
        Wzc = IMU_data.angular_velocity.z
        
        X_last = Xc
        Y_last = Yc
        Z_last = Zc
        if (k < 400 or k > 500):
            
            local_pos_pub.publish(pose)
            start_pub.publish(flag)
            k=k+1
        else:
            
            target = ref[iindex:iindex+N + 1]
            last_refPoint = ref[iindex-10]
            if target.shape[0] == N+1:

                current_states = np.array(
                [Xc, Yc, Zc] + [roll, pitch, yaw] + [Vx_mean, Vy_mean, Vz_mean] + [Wxc, Wyc, Wzc]
                )

                # record x, y ,z, this is record for 3D plot
                pos_record.append(np.array([Xc, Yc, Zc])) 
                measurements_record.append(current_states)
                pos_error_record.append(np.array([(last_refPoint[0]-Xc)**2 , (last_refPoint[1]-Yc)**2 , (last_refPoint[2]-Zc)**2]))
                ref_mes.append(np.array([last_refPoint[0],last_refPoint[1],last_refPoint[2],Xc,Yc,Zc]))

                pose.pose.position.x = x_ref[iindex+1]
                pose.pose.position.y = y_ref[iindex+1]
                pose.pose.position.z = z_ref[iindex+1]
                local_pos_pub.publish(pose)
                print('home position---')
                print(home_pos)
                iindex += 1
                
            else:
                

                print('==========================================================')
                print('===================finished ! =========================')
                print('==========================================================')
                np.savetxt(source_dir+'/data/pos_error_record_PID.out',np.array(pos_error_record).T,delimiter=',')
                np.savetxt(source_dir+'/data/ref_mes_PID.out',np.array(ref_mes).T,delimiter=',')
                k = 501

        rate.sleep()




if __name__ == "__main__":
    main_control()
    

