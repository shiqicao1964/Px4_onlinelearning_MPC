#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import matplotlib.pyplot as plt
import casadi as cs
from casadi.casadi import *
import math 
from scipy.optimize import minimize
from math import sqrt


# In[2]:


predictA = np.genfromtxt("model_predict_A.out", delimiter=",")
measurementA = np.genfromtxt("measurements_A.out", delimiter=",")
predictB = np.genfromtxt("model_predict_B.out", delimiter=",")
measurementB = np.genfromtxt("measurements_B.out", delimiter=",")
predictC = np.genfromtxt("model_predict_C.out", delimiter=",")
measurementC = np.genfromtxt("measurements_C.out", delimiter=",")
print(predictA.shape)
print(measurementA.shape)
print(predictB.shape)
print(measurementB.shape)
print(predictC.shape)
print(measurementC.shape)
ind = 20
measurementA = measurementA[:,:-ind]
predictA = predictA[:,:-ind]
ind = 700
measurementB = measurementB[:,:-ind]
predictB = predictB[:,:-ind]
ind = 20
measurementC = measurementC[:,:-ind]
predictC = predictC[:,:-ind]


# In[3]:



fig = plt.figure(figsize = (18,18))
ax = fig.add_subplot(111, projection='3d')
ax.plot(measurementA[0,:],measurementA[1,:],measurementA[2,:])
ax.scatter(measurementA[0,-1],measurementA[1,-1],measurementA[2,-1])
plt.show()


# In[4]:



fig = plt.figure(figsize = (18,18))
ax = fig.add_subplot(111, projection='3d')
ax.plot(measurementB[0,:],measurementB[1,:],measurementB[2,:])
ax.scatter(measurementB[0,-1],measurementB[1,-1],measurementB[2,-1],)
plt.show()


# In[5]:



fig = plt.figure(figsize = (18,18))
ax = fig.add_subplot(111, projection='3d')
ax.plot(measurementC[0,:],measurementC[1,:],measurementC[2,:])
ax.scatter(measurementC[0,-1],measurementC[1,-1],measurementC[2,-1])
plt.show()


# In[6]:


input_state_A = measurementA[:,0:-1]
error_y_A = measurementA[:,1:] - predictA[:,1:]

input_state_B = measurementB[:,0:-1]
error_y_B = measurementB[:,1:] - predictB[:,1:]

input_state_C = measurementC[:,0:-1]
error_y_C = measurementC[:,1:] - predictC[:,1:]


print(input_state_A.shape)
print(error_y_A.shape)
print(input_state_B.shape)
print(error_y_B.shape)
print(input_state_C.shape)
print(error_y_C.shape)


# In[7]:


input_state = np.concatenate((input_state_A,input_state_B,input_state_C),axis=1)
error_y = np.concatenate((error_y_A,error_y_B,error_y_C),axis=1)
print(input_state.shape)
print(error_y.shape)


# In[8]:


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


# In[ ]:


fun = lambda m: (
    RMS (input_state,error_y, m[0] ,m[1], 15)
)
bnds = ((1e-3,1e2), (1e-3, 1e2))
res = minimize(fun,[np.std(error_y),1], method='L-BFGS-B', bounds=bnds, options = {'maxls': 500})
print('std error_y')
print(np.std(error_y))

res


# In[9]:


from tqdm import trange
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
print(length_scales_2[minindex[0]])


# In[12]:


'L = [0.74081822]'


# In[10]:


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


# In[18]:


test_measurement = np.genfromtxt("test_measure.out", delimiter=",")[:,0:-20]
test_predict = np.genfromtxt("test_predict.out", delimiter=",")[:,0:-20]
input_state_test = test_measurement[:,0:-1]
error_y_test = test_measurement[:,1:] - test_predict[:,1:]


# In[ ]:


sig_f = np.std(error_y)
length_scale = length_scales_2[minindex[0]]
#sig_f,length_scale = res.x[0],res.x[1]

optimal,x = fit(input_state,error_y,length_scale,sig_f)
optimal_func = cs.Function('f',[x],[optimal])


# In[34]:



RMS_y = np.zeros((12,1))
RMS_error_error = np.zeros((12,1))
#for i in range(input_state_test.shape[1]):
i = 50
x_testing = input_state_test[:,i]
y_valid = error_y_test[:,i][:,np.newaxis]
test_y = optimal_func(x_testing)
print(test_y.shape)
RMS_error_error += (test_y - y_valid)**2
RMS_y += y_valid**2
    
#print(RMS_error_error/n)


# In[35]:


test_y


# In[36]:


y_valid


# In[ ]:


A = np.array([1,2,3,4,5,6,7,8])
B = np.array([1.2,2.1,3.5,4.1,5.5,6.1,7.2,8.9])

print ( np.sqrt (sum( (A - B)**2 ) ))

from scipy.spatial import distance
cityblock_dist = distance.cityblock(A,B)
euclidean_dist = distance.euclidean(A,B)
print(cityblock_dist)
print(euclidean_dist)


# In[ ]:


y_valid


# In[ ]:


print('%')
print(np.abs(test_y - y_valid) / y_valid * 100)


# In[ ]:





# In[ ]:




