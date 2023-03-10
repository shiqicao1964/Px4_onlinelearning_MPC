#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped ,TwistStamped, Twist, Pose
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import numpy as np
from collections import deque
from math import sqrt
import math
import time
from MPC import acados_settinngs, linear_quad_model, run_solver,DT_gp_model, DT_linear_model, solve_DT_nextState, eight_trag, ellipse_trag
from scipy.spatial import distance
import threading
import os
source_dir = os.getcwd()
print('source_dir',source_dir)
os.chdir(source_dir +'/data')
os.system('rm -r *.out')
os.chdir(source_dir)

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


rospy.wait_for_service("/mavros/cmd/arming")
arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
rospy.wait_for_service("/mavros/set_mode")
set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


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
rate_model = rospy.Rate(model_Hz)
discretization_dt = 0.05
v_average = 1 # 1.5 / 2 / 2.5 / 3
radius = 8
# set traj =================================================================================================================================
x_ref,y_ref,z_ref,t_ref,vx_ref,vy_ref,vz_ref,T_onecircle = ellipse_trag(speed = v_average,x_w = 1.3,y_w = 1.3,z_w = 0,H = 1.5,dT = discretization_dt ,sim_t = 350)
ref = np.zeros((x_ref.shape[0],12))
ref[::,0] = x_ref
ref[::,1] = y_ref
ref[::,2] = z_ref
ref[::,6] = vx_ref
ref[::,7] = vy_ref
ref[::,8] = vz_ref
# ==========================================================================================================================================
# set online learning GP data buffer: here set the buffer to be one circle: radius * 2* pi/ average_seppd

T_buff = T_onecircle * 2
buff_Length = int(T_buff / discretization_dt)
GP_buff_predict = deque([])
DT_buff_predict = deque([])
GP_buff_measurement = deque([])
GP_buff_controls = deque([])
print('buff_Length', buff_Length)
print('GP_buff.length : ',len(GP_buff_predict))
print('GP_buff : ',GP_buff_predict)


predict = np.genfromtxt(source_dir+'/GP_points/predict_mismatch_16.out', delimiter=",")
measurement = np.genfromtxt(source_dir+'/GP_points/measurement_mismatch_16.out', delimiter=",")
controls = np.genfromtxt(source_dir+'/GP_points/controls_mismatch_16.out', delimiter=",")
model_init = DT_gp_model(t_horizon/N,f'GP_{0}',predict,measurement,controls,buff_Length)
#model_init = DT_linear_model(dT = t_horizon/N,name = f'DT_{0}')

acados_solver = acados_settinngs(model_init,t_horizon = t_horizon,N=N)


solver_save.append(acados_solver)
solver_save.popleft()
model_save.append(model_init)
model_save.popleft()

# set measurement , pridect , controls recording deque
# record x, y ,z, this is record for 3D plot
pos_record = deque([])
model_predict_record = deque([])
measurements_record = deque([])
controlinput_record = deque([])
pos_error_record = deque([])
ref_mes = deque([])

def build_new_model():
    t_comp_start = time.time()
    model_iter = 1
    saved_dyn = 0
    while(1):

        predict = np.array(DT_buff_predict).T
        measurement = np.array(GP_buff_measurement).T
        controls = np.array(GP_buff_controls).T
        gp_predict = np.array(GP_buff_predict).T

        if len(GP_buff_predict) < buff_Length :
            time.sleep(0.05)
            acados_solver = acados_settinngs(model_save[0],t_horizon = t_horizon,N=N)
        elif (time.time() - t_comp_start < 100):
            predict = np.genfromtxt(source_dir+'/GP_points/predict_mismatch_16.out', delimiter=",")
            measurement = np.genfromtxt(source_dir+'/GP_points/measurement_mismatch_16.out', delimiter=",")
            controls = np.genfromtxt(source_dir+'/GP_points/controls_mismatch_16.out', delimiter=",")
            model_offlineGP = DT_gp_model(t_horizon/N,f'offlineGP_{0}',predict,measurement,controls,buff_Length)
            acados_solver = acados_settinngs(model_offlineGP,t_horizon = t_horizon,N=N)

        else:

            if saved_dyn ==0:
                # save dynamic model data
                np.savetxt(source_dir+'/data/posrecord_dyn.out',np.array(pos_record).T,delimiter=',')
                np.savetxt(source_dir+'/data/pos_error_record_dyn.out',np.array(pos_error_record).T,delimiter=',')
                np.savetxt(source_dir+'/data/ref_mes_dyn.out',np.array(ref_mes).T,delimiter=',')
                print('DYN record size:',len(pos_record))
                pos_record.clear()
                model_predict_record.clear()
                measurements_record.clear()
                controlinput_record.clear()
                pos_error_record.clear()
                print(' DYN ==========================================================')
                print(' DYN ===================data dyn saved ! =========================')
                print(' DYN ==========================================================')
                saved_dyn = 1
            time_new_solver = time.time()
            
            # save the latest GP data points
            np.savetxt(source_dir+'/GP_points/predict.out',predict,delimiter=',')
            np.savetxt(source_dir+'/GP_points/measurement.out',measurement,delimiter=',')
            np.savetxt(source_dir+'/GP_points/controls.out',controls,delimiter=',')
            np.savetxt(source_dir+'/GP_points/gp_predict.out',gp_predict,delimiter=',')
            
            creat_GP_offline = 0
            if creat_GP_offline == 1:
                predict = np.genfromtxt(source_dir+'/GP_points/predict_mismatch_16.out', delimiter=",")
                measurement = np.genfromtxt(source_dir+'/GP_points/measurement_mismatch_16.out', delimiter=",")
                controls = np.genfromtxt(source_dir+'/GP_points/controls_mismatch_16.out', delimiter=",")
            
            model = DT_gp_model(t_horizon/N,f'GP_{model_iter}',predict,measurement,controls,buff_Length)
            model_save.append(model)
            model_save.popleft()

            acados_solver = acados_settinngs(model_save[0],t_horizon = t_horizon,N=N)
            solver_save.append(acados_solver)
            solver_save.popleft()

            print('=======================================================================')
            print('===============                                        ================')
            print('===============                                        ================')
            print('===============       ====               ====          ================')
            print('===============                                        ================')
            print('===============                                        ================')
            print('===============                                        ================')
            print('===============                                        ================')
            print('===============                                        ================')
            print('===============                                        ================')
            print('===============       ==                     ==        ================')
            print('===============         ==                 ==          ================')
            print('===============           == == == == == ==            ================')
            print('===============                                        ================')
            print('===============                                        ================')
            print('===============                                        ================')
            print('===============                                        ================')
            print('===============                                        ================')
            print('=======================================================================', time.time() - time_new_solver)

            np.savetxt(f'/home/shiqi/.ros/data/posrecord_{model_iter}.out',np.array(pos_record).T,delimiter=',')
            np.savetxt(f'/home/shiqi/.ros/data/pos_error_record_{model_iter}.out',np.array(pos_error_record).T,delimiter=',')
            np.savetxt(f'/home/shiqi/.ros/data/ref_mes_{model_iter}.out',np.array(ref_mes).T,delimiter=',')
            pos_record.clear()
            model_predict_record.clear()
            measurements_record.clear()
            controlinput_record.clear()
            pos_error_record.clear()
            ref_mes.clear()
            model_iter += 1
        rate_model.sleep()
        
            


def main_control():
    # set MPC 
    model_DT = DT_linear_model(dT = t_horizon/N,name = f'DT_{0}')
    acados_solver = solver_save[0]
    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
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
            last_refPoint = ref[iindex+1]
            if target.shape[0] == N+1:
                t_s = time.time()
                # use last measurement and control inputs to predict current using DT model   
                DT_result = solve_DT_nextState(model_DT,control_input ,current_states)
                # use last measurement and control inputs to predict current using GP + DT model   
                acados_solver = solver_save[0]
                model = model_save[0]
                GP_result = solve_DT_nextState(model,control_input ,current_states)
                # get measurement as current states 
                current_states = np.array(
                [Xc, Yc, Zc] + [roll, pitch, yaw] + [Vx_mean, Vy_mean, Vz_mean] + [Wxc, Wyc, Wzc]
                )

                # solve MPC and get control signal
                vx,vy,vz,p,q,r,control_input= run_solver(N=N,model=None,
                acados_solver=acados_solver,initial_state = current_states,
                ref=target)

                # record x, y ,z, this is record for 3D plot
                pos_record.append(np.array([Xc, Yc, Zc])) 
                model_predict_record.append(np.array(DT_result).T.squeeze(0))
                measurements_record.append(current_states)
                controlinput_record.append(control_input)
                pos_error_record.append(np.array([(last_refPoint[0]-Xc)**2 , (last_refPoint[1]-Yc)**2 , (last_refPoint[2]-Zc)**2]))
                ref_mes.append(np.array([last_refPoint[0],last_refPoint[1],last_refPoint[2],Xc,Yc,Zc]))
                # record GP data 
                if len(GP_buff_predict) < buff_Length : 
                    DT_buff_predict.append(np.array(DT_result).T.squeeze(0))
                    GP_buff_predict.append(np.array(GP_result).T.squeeze(0))
                    GP_buff_measurement.append(current_states)
                    GP_buff_controls.append(control_input)
                else:
                    GP_buff_predict.append(np.array(GP_result).T.squeeze(0))
                    GP_buff_predict.popleft()
                    DT_buff_predict.append(np.array(DT_result).T.squeeze(0))
                    DT_buff_predict.popleft()
                    GP_buff_measurement.append(current_states)
                    GP_buff_measurement.popleft()
                    GP_buff_controls.append(control_input)
                    GP_buff_controls.popleft()
                

                # set message, sent vel vx, vy ,vz
                vel.linear.x = vx 
                vel.linear.y = vy 
                vel.linear.z = vz 
                vel.angular.x = 0
                vel.angular.y = 0
                vel.angular.z = 0
                setvel.publish(vel)
                t_end = time.time()
                
                iindex += 1
                #print('time spend for one control (ms)', (t_end - t_s)*1000, 'height', Zc)
                control_print_flag = 0
                if control_print_flag == 1:
                    print('time spend for one control', t_end - t_s)
                    print('input to PX4',vx,vy,vz)
                    print('control inupt u(4)',control_input)
                    print('measurement',current_states[:3])
                    print('predict',np.array(DT_result).T.squeeze(0)[:3])
                    print('trasint velocity',math.sqrt(Vx_mean**2 + Vy_mean**2 + Vz_mean**2))
                    print('error distance', distance.euclidean(current_states[:3],np.array(DT_result).T.squeeze(0)[:3]))
                    print('radius',math.sqrt(Xc**2 + Yc**2))
                
            else:
                

                print('==========================================================')
                print('===================finished ! =========================')
                print('==========================================================')
                k = 501

        rate.sleep()




if __name__ == "__main__":
    t1 = threading.Thread(target=build_new_model)
    t2 = threading.Thread(target=main_control)
    
    # start threads
    t1.start()
    t2.start()
    # wait for threads to finish
    t1.join()
    t2.join()
    

