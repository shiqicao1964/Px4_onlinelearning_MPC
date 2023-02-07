#!/usr/bin/env python
# coding: utf-8

# In[275]:


import numpy as np
import matplotlib.pyplot as plt
import math
import sympy


# In[359]:


def eight_trag(speed = 3,x_w = 3,y_w = 4,z_w = 0,H = 5,dT = 0.05,sim_t = 100):
    t_abl = [0]
    t = np.linspace(0, 2*np.pi, num=10000)
    x = x_w * np.cos(t) / (1 + np.sin(t)**2)
    y = y_w * np.sin(t) * np.cos(t) / (1 + np.sin(t)**2)
    z = H + z_w*np.sin(t)
    x0 = x_w * np.cos(0) / (1 + np.sin(0)**2)
    y0 = y_w * np.sin(0) * np.cos(0) / (1 + np.sin(0)**2)
    z0 = H + z_w*np.sin(0)

    for n in range(len(t)):
        # calculate if dist == dT * speed
        dist = math.sqrt((x[n]-x0)**2 + (y[n]-y0)**2 + (z[n]-z0)**2)
        if dist > dT * speed:
            t_abl.append(t[n])
            x0 = x[n]
            y0 = y[n]
            z0 = z[n]
    t_new = np.array(t_abl)
    T_onecircle = len(t_new)*dT
    circles = int(np.ceil(sim_t/T_onecircle))
  
    t_abl = [0]
    vx = []
    vy = []
    vz = []
    t = np.linspace(0, circles*2*np.pi, num=10000*circles)
    x = x_w * np.cos(t) / (1 + np.sin(t)**2)
    y = y_w * np.sin(t) * np.cos(t) / (1 + np.sin(t)**2)
    z = H + z_w*np.sin(t)
    x0 = x_w * np.cos(0) / (1 + np.sin(0)**2)
    y0 = y_w * np.sin(0) * np.cos(0) / (1 + np.sin(0)**2)
    z0 = H + z_w*np.sin(0)
    for n in range(len(t)):
        # calculate if dist == dT * speed
        dx = x[n]-x0
        dy = y[n]-y0
        dz = z[n]-z0
        dist = math.sqrt((dx)**2 + (dy)**2 + (dz)**2)
        if dist > dT * speed:
            t_abl.append(t[n])
            vx.append( dx * speed / dist )
            vy.append( dy * speed / dist )
            vz.append( dz * speed / dist )
            x0 = x[n]
            y0 = y[n]
            z0 = z[n]
    t_new = np.array(t_abl)
    x = x_w * np.cos(t_new) / (1 + np.sin(t_new)**2)
    y = y_w * np.sin(t_new) * np.cos(t_new) / (1 + np.sin(t_new)**2)
    z =  H + z_w*np.sin(t_new)
    vx = np.array(vx)
    vy = np.array(vy)
    vz = np.array(vz)
    return x[:-1], y[:-1], z[:-1],t_new,vx,vy,vz


# In[368]:


x,y,z,t,vx,vy,vz = eight_trag(speed = 4,x_w = 3,y_w = 4,z_w = 0.1,H = 5,dT = 0.05,sim_t = 100)
ref = np.zeros((x.shape[0],12))
ref[::,0] = x
ref[::,1] = y
ref[::,2] = z
ref[::,6] = vx
ref[::,7] = vy
ref[::,8] = vz


# In[369]:


print(x.shape,y.shape,z.shape,vx.shape,vy.shape,vz.shape)


# In[370]:


fig = plt.figure(figsize = (15,15))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x[:],y[:],z[:])
ax.scatter(x[0],y[0],z[0],color='red')
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_zlim(4, 6)
plt.show()


# In[371]:


linearV = np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2)/dT
plt.scatter(t[:-2],linearV)
plt.ylim(0, 6)


# In[ ]:




