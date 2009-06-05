# coding=utf-8
"""
Functions for working with twists stored as [w,v]
"""
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from numpy import zeros #,array, sin, cos, eye, dot, hstack, vstack :no more need
#from numpy.linalg import norm .... no more need

cimport numpy as np
import numpy as np
DTYPE = np.float 
ctypedef np.float_t DTYPE_t

cdef extern from "math.h":
    float cosf(float theta)
    float sinf(float theta)
    float sqrtf(float x)
cpdef adjacency(np.ndarray tw):
    """
    return the adjacency matrix
    """
#    return array(
#        [[     0,-tw[2], tw[1],      0,     0,     0],
#         [ tw[2],     0,-tw[0],      0,     0,     0],
#         [-tw[1], tw[0],     0,      0,     0,     0],
#         [     0,-tw[5], tw[4],      0,-tw[2], tw[1]],
#         [ tw[5],     0,-tw[3],  tw[2],     0,-tw[0]],
#         [-tw[4], tw[3],     0, -tw[1], tw[0],     0]])



    cdef np.ndarray[DTYPE_t, ndim=2] A = zeros((6,6),dtype=DTYPE)
    A[0,1]=-tw[2]
    A[0,2]=tw[1]

    A[1,0]=tw[2]
    A[1,2]=-tw[0]

    A[2,0]=-tw[1]
    A[2,1]=tw[0]

    A[3,1]=-tw[5]
    A[3,2]=tw[4]
    A[3,4]=-tw[2]
    A[3,5]=tw[1]

    A[4,0]=tw[5]
    A[4,2]=-tw[3]
    A[4,3]=tw[2]
    A[4,5]=-tw[0]

    A[5,0]=-tw[4]
    A[5,1]=tw[3]
    A[5,3]=-tw[1]
    A[5,4]=tw[0]
    return A

cpdef exp(np.ndarray tw):
    """
    return the exponential of the twist matrix
    """
    cdef int i,j,k
    cdef np.ndarray[DTYPE_t, ndim=1] w = zeros((3),dtype=DTYPE)
    cdef np.ndarray[DTYPE_t, ndim=1] v = zeros((3),dtype=DTYPE)
    cdef np.ndarray[DTYPE_t, ndim=2] wx = zeros((3,3),dtype=DTYPE)
    #this is used further in R and p calculation 
    cdef np.ndarray[DTYPE_t, ndim=2] R = zeros((3,3),dtype=DTYPE)
    cdef np.ndarray[DTYPE_t, ndim=2] temp = zeros((3,3),dtype=DTYPE)
    cdef np.ndarray[DTYPE_t, ndim=1] p = zeros((3),dtype=DTYPE)
    cdef np.ndarray[DTYPE_t, ndim=2] E = zeros((4,4),dtype=DTYPE)
    E[3,3]= 1.
    for i in range(3):
        w[i] = tw[i]
        v[i] = tw[i+3]

#    wx = array(
#        [[     0,-tw[2], tw[1]],
#         [ tw[2],     0,-tw[0]],
#         [-tw[1], tw[0],     0]])

    wx[0,1]= -tw[2]
    wx[0,2]= tw[1]

    wx[1,0]= tw[2]
    wx[1,2]= -tw[0]

    wx[2,0]= -tw[1]
    wx[2,1]= tw[0]

#    t = norm(w)
    cdef float t=sqrtf(w[0]*w[0] + w[1]*w[1]+ w[2]*w[2])
    cdef float cc
    cdef float sc
    cdef float dsc
    cdef float dot_w_wt = 0
    if t >= 0.001:
        cc = (1-cosf(t))/(t**2)
        sc = sinf(t)/t
        dsc = (t-sinf(t))/(t**3)
    else:
        cc = 1./2.
        sc = 1.-t**2/6.
        dsc = 1./6.0

#    R = eye(3) + sc*wx + cc*dot(wx,wx)
#    p = dot(sc*eye(3)+cc*wx+dsc*dot(w,w.T), v)
#return vstack((hstack((R, p.reshape(3,1))),
#                   array([[0., 0., 0., 1.]])))

    for i in range(3):
        for j in range(3):
            for k in range (3):
                R[i,j]= R[i,j] + wx[i,k] * wx[k,j]
                dot_w_wt = dot_w_wt + w[k] * w[k]
            if (i==j):
                R[i,j]= 1. + cc * R[i,j] + sc * wx[i,j]
                temp[i,j]= sc  + cc * wx[i,j] + dot_w_wt*dsc
                dot_w_wt= 0.
            else:
                R[i,j]= cc * R[i,j] + sc * wx[i,j]
                temp[i,j]= cc * wx[i,j] + dot_w_wt*dsc
                dot_w_wt= 0.
            E[i,j]=R[i,j]
        for k in range (3):
            p[i]=p[i] + temp[i,k]* v[k]
        E[i,3]=p[i]
    return E
