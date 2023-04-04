#!/usr/bin/env python
# coding: utf-8

# In[1]:


from casadi.casadi import *
from tqdm import trange
import time
import numpy as np
from math import sqrt
import math 
import casadi as cs
from scipy.optimize import minimize
import numpy as np
import matplotlib.pyplot as plt
import casadi as cs
from casadi.casadi import *
import math 
from scipy.optimize import minimize
from math import sqrt
import time
import numpy as np
from sklearn.decomposition import PCA


# In[2]:


predict_train = np.genfromtxt("2_c data/predict.out", delimiter=",")
control_train = np.genfromtxt("2_c data/controls.out", delimiter=",")
measurement_train = np.genfromtxt("2_c data/measurement.out", delimiter=",")


# In[3]:


predict_test = np.genfromtxt("2_c_testset/predict.out", delimiter=",")
control_test = np.genfromtxt("2_c_testset/controls.out", delimiter=",")
measurement_test = np.genfromtxt("2_c_testset/measurement.out", delimiter=",")


# In[4]:


class GPR:

    def __init__(self, optimize=True):
        self.is_fit = False
        self.train_X, self.train_y = None, None
        self.params = {"l": 0.3, "sigma_f": 0.2}
        self.optimize = optimize
    def kernel(self, x1, x2):
        dist_matrix = np.sum(x1**2, 1).reshape(-1, 1) + np.sum(x2**2, 1) - 2 * np.dot(x1, x2.T)
        return self.params["sigma_f"] ** 2 * np.exp(-0.5 / self.params["l"] ** 2 * dist_matrix)
    
    def fit(self, X, y,bounds_):
        # store train data
        self.train_X = np.asarray(X)
        self.train_y = np.asarray(y)
        self.params['sigma_f'] = np.std(y)
         # hyper parameters optimization
        def negative_log_likelihood_loss(params):
            self.params["l"]= params
            Kyy = self.kernel(self.train_X, self.train_X) #+ 1e-8 * np.eye(len(self.train_X))
            loss = 0.5 * self.train_y.T.dot(np.linalg.inv(Kyy)).dot(self.train_y) + 0.5 * np.linalg.slogdet(Kyy)[1] + 0.5 * len(self.train_X) * np.log(2 * np.pi)
            return np.sum(loss.ravel())

        if self.optimize:
            res = minimize(negative_log_likelihood_loss, [self.params["l"]],
                   bounds=bounds_,
                   method='L-BFGS-B')
            self.params["l"] = res.x[0]

        self.is_fit = True


# In[5]:


def kernel(x1, x2,sigf,l):
    x1 = x1.T
    x2 = x2.T
    dist_matrix = np.sum(x1**2, 1).reshape(-1, 1) + np.sum(x2**2, 1) - 2 * np.dot(x1, x2.T)
    return sigf ** 2 * np.exp(-0.5 / l ** 2 * dist_matrix)
def fit_gp(sig_f,l,X,Y,S):
    K = kernel(X,X,sig_f,l)
    x1 = S.T
    x2 = X.T
    dist_matrix = cs.sum2(x1**2) + cs.sum2(x2**2) - ((cs.mtimes(x1, x2.T))*2).T
    Kstar = (sig_f ** 2 * np.exp(-0.5 / l ** 2 * dist_matrix)).T
    error = cs.mtimes ( cs.mtimes(Kstar,np.linalg.inv(K) ), Y.T ).T
    return error


# In[6]:


# Declare model variables
roll = cs.MX.sym('roll')  # position
pitch = cs.MX.sym('pitch')
yaw = cs.MX.sym('yaw')

x_ = cs.MX.sym('x_')
y_ = cs.MX.sym('y_')
z_ = cs.MX.sym('z_')

p = cs.MX.sym('p')
q = cs.MX.sym('q')
r = cs.MX.sym('r')

vx = cs.MX.sym('vx')
vy = cs.MX.sym('vy')
vz = cs.MX.sym('vz')
# Full state vector (12-dimensional)
x = cs.vertcat(x_,y_,z_,roll,pitch,yaw,vx,vy,vz,p,q,r)
# Control input vector
u1 = cs.MX.sym('u1')
u2 = cs.MX.sym('u2')
u3 = cs.MX.sym('u3')
u4 = cs.MX.sym('u4')
u = cs.vertcat(u1, u2, u3, u4)
# new dims 
V1 = cs.MX.sym('V1')
V2 = cs.MX.sym('V2')
V3 = cs.MX.sym('V3')
V4 = cs.MX.sym('V4')
V5 = cs.MX.sym('V5')
V6 = cs.MX.sym('V6')
V7 = cs.MX.sym('V7')
V8 = cs.MX.sym('V8')
Vmm = cs.vertcat(V1, V2, V3, V4, V5, V6 ,V7, V8)


# In[7]:


x1 = (measurement_train[:,0:-1])
x2 = (control_train[:,0:-1])
input_state_train = np.concatenate(( x1 , x2 ), axis=0)
error_y_train = (measurement_train[:,1:] - predict_train[:,1:])
# use 100 points in GP 
down_sample_factor = int(500 / 200 )
X = input_state_train[:,::down_sample_factor]
Y = error_y_train[:,::down_sample_factor]

Y = Y[[8],:]
print('X',X.shape)
print('Y',Y.shape)
dim = 8
V = cs.MX.sym('V',dim,1)
pca = PCA(n_components=dim)
pca.fit(X.T)
W = pca.components_
print('Loading matrix:\n', W.shape)
X_hand = np.dot(X.T, W.T).T
print('the reduced X_hand\n',X_hand.shape)

gpr1 = GPR()
gpr1.fit(X_hand.T,Y.T , [(0.01, 2)] )
l_1 = gpr1.params['l']
sig_f_1 = gpr1.params['sigma_f']
print('L, sigma_f',l_1,sig_f_1)
t1 = time.time()
error_8 = fit_gp(sig_f_1,l_1,X_hand,Y,V)
print('time for GP train',time.time() - t1)


# In[8]:


x1 = (measurement_test[:,0:-1])
x2 = (control_test[:,0:-1])
input_state_test = np.concatenate(( x1 , x2 ), axis=0)
error_y_test = (measurement_test[:,1:] - predict_test[:,1:])
error_num = cs.Function('error_8',[V], [error_8])

for i in range(10):
    print(error_y_test[3,i])


# In[9]:


low_dim_result = np.array([])
for i in range(error_y_test[8,:].shape[0]):
    query_point = W.dot(input_state_test[:,i][:,np.newaxis]) 
    low_dim_result = np.append(low_dim_result,error_num(query_point))
    if i < 10 :
        print(error_num(query_point) )

low_dim_result.shape


# In[10]:


low_dim_result


# In[11]:


query_point = W.dot(input_state_test[:,3][:,np.newaxis]) 
print(query_point)
print(input_state_test[:,i][:,np.newaxis])


# In[12]:


plt.plot(np.arange(0,error_y_test[8,:].shape[0]),error_y_test[8,:],c='orange')
plt.plot(np.arange(0,error_y_test[8,:].shape[0]),low_dim_result,c='red')


# In[13]:


np.mean ((low_dim_result - error_y_test[8,:])**2)


# In[14]:


plt.plot(np.arange(0,error_y_train[3,:].shape[0]),error_y_train[3,:],c='orange')
plt.plot(np.arange(0,error_y_test[3,:].shape[0]),error_y_test[3,:],c='blue')


# In[ ]:




