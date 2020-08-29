#!/usr/bin/env python
# coding: utf-8

# # Quaternion

import numpy as np
from .utils_ta import skew_symmetric

def from_euler(r, p, y):
    # Convert euler angle to quaternion

    # Input :
    # r = roll
    # p = pitch
    # y = yaw

    # Output :
    # quaternion form in numpy array

    cr = np.cos(r/2)
    sr = np.sin(r/2)
    cp = np.cos(p/2)
    sp = np.sin(p/2)
    cy = np.cos(y/2)
    sy = np.sin(y/2)

    q1 = cr*cp*cy + sr*sp*sy
    q2 = sr*cp*cy - cr*sp*sy
    q3 = cr*sp*cy + sr*cp*sy
    q4 = cr*cp*sy - sr*sp*cy

    return np.array([q1, q2, q3, q4]).astype('float64')

def to_euler(q0, q1, q2, q3):
    # Convert quaternion to euler angle

    # Output :
    # euler angle (roll, pitch, yaw)

    r = np.arctan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1**2 + q2**2))
    p = np.arcsin(2 * (q0*q2 - q3*q1))
    y = np.arctan2(2 * (q0*q3 + q1*q2), 1 - 2 * (q2**2 + q3**2))

    return np.array([r, p, y]).astype('float64')

def from_axis_angle(a1, a2, a3):
    # Convert axis angle to quaternion

    # Output :
    # quaternion form in numpy array
    axis_angle = np.array([a1, a2, a3])
    norm = np.linalg.norm(axis_angle)
    q = np.zeros(4)

    q[0] = np.cos(norm/2)
    if norm < 10**(-10) :
        q[1:] = 0
    else :
        q[1:] = axis_angle / norm * np.sin(norm/2)

    return q.astype('float64')

def to_axis_angle(q0, q1, q2, q3):
    x = 2*np.arccos(q0)

    return (x*np.array([q1, q2, q3])/np.sin(x/2)).astype('float64')

def product(left, right):
    # Quaternion product or multiplication
    qv = np.array([left[1], left[2], left[3]])
    m = np.zeros([4, 4])
    m[0,1:] = -qv
    m[1:,0] = qv
    m[1:,1:] = skew_symmetric(*qv)

    return np.dot(left[0] * np.eye(4) + m, right).astype('float64')

def to_rotation_matrix(w,x,y,z):
    qv = np.array([x,y,z]).reshape(3,1)

    return ((w**2 - qv.T @ qv) * np.eye(3) + 2 * qv @ qv.T + 2 * w * skew_symmetric(*qv)).astype('float64')
