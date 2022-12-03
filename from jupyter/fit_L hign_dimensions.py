import numpy as np
import matplotlib.pyplot as plt
import casadi as cs
from casadi.casadi import *
import math 
from scipy.optimize import minimize
from math import sqrt

def f(x, y):
    return np.sin(x) + 2* np.cos(x) + 0.5* y**2 + 0.1*y**3

  

x = np.linspace(-4,4,100)
y = np.linspace(-4,4,100)
X, Y = np.meshgrid(x1_linspace, x2_linspace)
Z =  f(X,Y)
z = f(x,y)

fig = plt.figure(figsize = (18,18))
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(X, Y, Z, rstride=1, cstride=1,
                 edgecolor='none')
ax.scatter(X[55,55], Y[55,55], Z[55,55], marker='^',linewidth=5,color = '#ff2300')
plt.show()
