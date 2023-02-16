#!/usr/bin/env python
# coding: utf-8

# In[9]:


import numpy as np
import matplotlib.pyplot as plt
import casadi as cs
from casadi.casadi import *
import math 
from scipy.optimize import minimize
from math import sqrt
import time


# In[2]:


predict = np.genfromtxt("model_predict_record.out", delimiter=",")
measurement = np.genfromtxt("measurements_record.out", delimiter=",")
print(predict.shape)
print(measurement.shape)


# In[3]:


N = 500
useful_p = predict[:,1:N]
useful_m = measurement[:,1:N]
input_state = measurement[:,0:N-1]
print(input_state.shape)
ind = 100
error_y = useful_m - useful_p


# In[33]:


def kernel(x1, x2,sigf,l):
    x1 = x1.T
    x2 = x2.T
    dist_matrix = np.sum(x1**2, 1).reshape(-1, 1) + np.sum(x2**2, 1) - 2 * np.dot(x1, x2.T)
    return sigf ** 2 * np.exp(-0.5 / l ** 2 * dist_matrix)

def y_hat(X,y,query_slot_index,sigf,l) :
    query_point = X[:,query_slot_index]
    traian_setX = np.delete(X, query_slot_index, axis=1)
    train_sety = np.delete(y, query_slot_index, axis=1)
    
    Ks = kernel(query_point,traian_setX,sigf,l)
    Kxxinv = np.linalg.inv( kernel(traian_setX,traian_setX,sigf,l))
    
    y_hat = np.dot ( np.dot(  Ks ,Kxxinv ), train_sety.T ).T
    return y_hat


def error_square(X,y,query_slot_index,sigf,l):
    Validation_y = y[:,query_slot_index]
    error_square = sum (sum ((y_hat(X,y,query_slot_index,sigf,l) - Validation_y)**2))
    return error_square

def RMS (X,y,sigf,l,slot_number):
    totalRMS = 0
    slot_size = int(X.shape[1]/slot_number)
    for i in range(slot_number):
        query_slot_index = np.array(range(i*slot_size,(i+1)*slot_size  ))
        i_RMS = error_square(X,y,query_slot_index,sigf,l)
        totalRMS += i_RMS
    return totalRMS

def negative_log_likelihood_loss(X,y,sigf,l):
    K = kernel(X,X,sigf,l)
    loss = 0.5 * y.dot(np.linalg.inv(K)).dot(y.T) + 0.5 * np.linalg.slogdet(K)[1] + 0.5 * X.shape[1] * np.log(2 * np.pi)
    return loss.ravel()

from scipy.optimize import minimize


class GPR:

    def __init__(self, optimize=True):
        self.is_fit = False
        self.train_X, self.train_y = None, None
        self.params = {"l": 0.5, "sigma_f": 0.2}
        self.optimize = optimize
    def kernel(self, x1, x2):
        dist_matrix = np.sum(x1**2, 1).reshape(-1, 1) + np.sum(x2**2, 1) - 2 * np.dot(x1, x2.T)
        return self.params["sigma_f"] ** 2 * np.exp(-0.5 / self.params["l"] ** 2 * dist_matrix)
    
    def fit(self, X, y):
        # store train data
        self.train_X = np.asarray(X)
        self.train_y = np.asarray(y)

         # hyper parameters optimization
        def negative_log_likelihood_loss(params):
            self.params["l"], self.params["sigma_f"] = params[0], params[1]
            Kyy = self.kernel(self.train_X, self.train_X) + 1e-8 * np.eye(len(self.train_X))
            loss = 0.5 * self.train_y.T.dot(np.linalg.inv(Kyy)).dot(self.train_y) + 0.5 * np.linalg.slogdet(Kyy)[1] + 0.5 * len(self.train_X) * np.log(2 * np.pi)
            return np.sum(loss.ravel())

        if self.optimize:
            res = minimize(negative_log_likelihood_loss, [self.params["l"], self.params["sigma_f"]],
                   bounds=((1e-5, 1e4), (1e-5, 1e4)),
                   method='L-BFGS-B')
            self.params["l"], self.params["sigma_f"] = res.x[0], res.x[1]

        self.is_fit = True


# In[34]:


# method 1

test_gpr = GPR()
for k in range(12):
    t_start = time.time()
    test_gpr.fit(input_state.T,error_y[k,:].T)
    print(f'length_scale{k}:',test_gpr.params['l'],'sig_f:',test_gpr.params['sigma_f'], 'time used', time.time() - t_start)


# In[12]:


# method 2
t_start = time.time()
fun = lambda m: (
    RMS (input_state,error_y, m[0] ,m[1], 15)
)
bnds = ((1e-3,1e2), (1e-3, 1e2))
res = minimize(fun,[np.std(error_y),1], method='L-BFGS-B', bounds=bnds, options = {'maxls': 500})
print('std error_y')
print(np.std(error_y))

print('length_scale:',res.x[1],'sig_f:',res.x[0], 'time used', time.time() - t_start) 


# In[25]:


from tqdm import trange
t_start = time.time()

slot_number = 15
errorRMS_1 = np.zeros(60)
length_scales = np.zeros(60)
for k in trange(60):
    length_scale = math.exp(k/10 -3)
    length_scales[k] = length_scale
    errorRMS_1[k] = RMS(input_state,error_y,np.std(error_y),length_scale,slot_number)
plt.plot(length_scales,errorRMS_1)
plt.show()
minindex_1 = numpy.where(errorRMS_1 == numpy.amin(errorRMS_1))[0]
errorRMS_2 = np.zeros(14)
length_scales_2 = np.zeros(14)
for k in trange(14):
    length_scale = math.exp(k/50 -3.14 + minindex_1/10)
    length_scales_2[k] = length_scale
    errorRMS_2[k] = RMS(input_state,error_y,np.std(error_y),length_scale,slot_number)
plt.plot(length_scales_2,errorRMS_2)
plt.show()
minindex = numpy.where(errorRMS_2 == numpy.amin(errorRMS_2))
print()
print('length_scale:',length_scales_2[minindex[0]],'sig_f:',np.std(error_y), 'time used', time.time() - t_start) 


# In[9]:


def fit(X,y,l,sig_f):
    
    L = np.diag(np.ones(  X.shape[0]  ) * l)
    L = L**2

    K = kernel(X,X,sig_f,l)
    error = cs.SX.sym('error',y.shape[0],1)
    x = cs.SX.sym('x',X.shape[0],1)

    Kstar = cs.SX.sym('Kstar', x.shape[1] ,X.shape[1])
    for i in range(x.shape[1]):
        for j in range(X.shape[1]):
            zj = X[:,j][:,np.newaxis]
            zi = x
            Kstar[i,j] = sig_f**2 * cs.exp(   -cs.mtimes(    cs.mtimes( (zi-zj).T  ,  np.linalg.inv(L)   ) ,    (zi-zj)  )   / 2 ) 
    error = cs.mtimes ( cs.mtimes(Kstar,np.linalg.inv(K) ), y.T ).T
    
    return error,x


# In[14]:


sig_f = np.std(error_y)
length_scale = length_scales_2[minindex[0]]
#sig_f,length_scale = res.x[0],res.x[1]

optimal,x = fit(input_state,error_y,length_scale,sig_f)
optimal_func = cs.Function('f',[x],[optimal])

RMS_y = np.zeros((12,1))
RMS_error_error = np.zeros((12,1))

test_index = 530
x_testing = measurement[:,test_index]
y_valid = measurement[:,test_index+1][:,np.newaxis] - predict[:,test_index+1][:,np.newaxis]
test_y = optimal_func(x_testing)
RMS_error_error += (test_y - y_valid)**2
RMS_y += y_valid**2


# In[136]:


A = np.array([1,2,3,4,5,6,7,8])
B = np.array([1.2,2.1,3.5,4.1,5.5,6.1,7.2,8.9])

print ( np.sqrt (sum( (A - B)**2 ) ))

from scipy.spatial import distance
cityblock_dist = distance.cityblock(A,B)
euclidean_dist = distance.euclidean(A,B)
print(cityblock_dist)
print(euclidean_dist)


# In[15]:


test_y


# In[16]:


y_valid


# In[121]:


print('%')
print(np.abs(test_y - y_valid) / y_valid * 100)


# In[ ]:





# In[ ]:




