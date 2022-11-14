#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped ,TwistStamped, Twist, Pose
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
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
    start_pub = rospy.Publisher("start_time", Bool, queue_size=10)
    ffaa = rospy.Subscriber("reference", Twist, callback = refv_cb)
    wwss = rospy.Subscriber("reference2", Pose, callback = refp_cb)


    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    # set MPC 
    t_horizon = 10
    N = 20 # number of optimization nodes until time horizon
    model = linear_quad_model()
    acados_solver = acados_settinngs(t_horizon = t_horizon,N=N)
    

    # Setpoint publishing MUST be faster than 2Hz
    Hz = 20
    rate = rospy.Rate(Hz)


    # set traj 
    discretization_dt = 0.05
    radius = 8
    z = 3
    lin_acc = 1
    clockwise = True
    yawing = True
    v_max = 4
    v_average = 4
    sim_t = 25

    t_speedup = v_average/lin_acc
    t_speeddown = t_speedup
    t_uniform_circular = sim_t - t_speedup*2
    angle_acc = lin_acc / radius  # rad/s^2
    t_speedup_points = np.linspace(start = 0, stop = t_speedup, num = int(t_speedup/discretization_dt)+1)
    angle_points_1 = 0.5 * angle_acc * t_speedup_points**2
    anglevel_points_1 = angle_acc * t_speedup_points
    t_uniform_circular_points = np.linspace(start= discretization_dt, stop=t_uniform_circular, num=int(t_uniform_circular/discretization_dt))
    angle_points_2 = angle_points_1[-1] + t_uniform_circular_points * v_average/radius
    anglevel_points_2 = t_uniform_circular_points * 0 + anglevel_points_1[-1]
    t_speeddown_points = np.linspace(start = discretization_dt, stop = t_speeddown, num = int(t_speeddown/discretization_dt))
    angle_points_3 = angle_points_2[-1] + v_average/radius * t_speeddown_points - 0.5 * angle_acc * t_speeddown_points**2
    anglevel_points_3 = anglevel_points_2[-1] - angle_acc * t_speeddown_points

    angle_points = np.concatenate((angle_points_1,angle_points_2,angle_points_3))
    anglevel_points = np.concatenate((anglevel_points_1,anglevel_points_2,anglevel_points_3))
    pos_traj_x = radius * np.sin(angle_points)
    pos_traj_y = radius * np.cos(angle_points)
    pos_traj_z = np.ones_like(pos_traj_x) * z
    vel_traj_x = anglevel_points * radius * np.cos(angle_points)
    vel_traj_y = anglevel_points * radius * np.sin(angle_points)
    ref = np.zeros((vel_traj_x.shape[0],12))
    ref[::,0] = pos_traj_x
    ref[::,1] = pos_traj_y
    ref[::,2] = pos_traj_z
    ref[::,6] = vel_traj_x
    ref[::,7] = vel_traj_y

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

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
    pose.pose.position.x = 0
    pose.pose.position.y = 8
    pose.pose.position.z = 3

    # Send a few setpoints before starting
    for i in range(100):   
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

    k=0
    Z_last = 0
    iindex = 0
    pos_record = np.zeros((vel_traj_x.shape[0],3))

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

        current_states = np.array(
            [Xc, Yc, Zc] + [roll, pitch, yaw] + [Vxc, Vyc, Vzc] + [Wxc, Wyc, Wzc]
        )
        
        if (k < 400):

            local_pos_pub.publish(pose)
            start_pub.publish(flag)
            k=k+1
        else:
            
            target = ref[iindex:iindex+N + 1]
            if target.shape[0] == N+1:
                vx,vy,vz,p,q,r = run_solver(N=N,model=model,
                acados_solver=acados_solver,initial_state = current_states,
                ref=target)
                vel.linear.x = vx
                vel.linear.y = vy
                vel.linear.z = vz
                vel.angular.x = 0
                vel.angular.y = 0
                vel.angular.z = 0

                pos_record[iindex][0] = Xc
                pos_record[iindex][1] = Yc
                pos_record[iindex][2] = Zc
                iindex = iindex + 1
                setvel.publish(vel)
                start_pub.publish(flag)
                
            else:
                np.savetxt('data/posrecord.out',pos_record,delimiter=',')
                print('data saved  ! ')
                break
        rate.sleep()

        print('x is:')
        print(pos_1.pose.position.x)
        print('y is:')
        print(pos_1.pose.position.y)
        print('z is:')
        print(pos_1.pose.position.z)
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

                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')
                ax.plot(pos_record[:iindex,0],pos_record[:iindex,1],pos_record[:iindex,2])
                ax.plot(pos_traj_x[:iindex],pos_traj_y[:iindex],pos_traj_z[:iindex])
                plt.show()


        '''

