

import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
import math



if __name__ == "__main__":
   
    N = 20
    # set traj 
    discretization_dt = 0.05
    radius = 8
    z = 3
    lin_acc = 0.5
    clockwise = True
    yawing = True
    v_max = 3
    v_average = 3
    sim_t = 120

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
    pos_record = np.genfromtxt('posrecord.out',delimiter=',')
    iindex = vel_traj_x.shape[0] - N

    fig = plt.figure(figsize = (18,18))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(pos_record[:iindex,0],pos_record[:iindex,1],pos_record[:iindex,2])
    ax.plot(pos_traj_x[:iindex],pos_traj_y[:iindex],pos_traj_z[:iindex])
    plt.show()
	
    print('average height')
    print(np.mean(pos_record[:iindex,2]))
