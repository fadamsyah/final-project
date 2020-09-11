import numpy as np
from scipy import integrate

def IAE(e, t):
    x = t - t[0]
    y = np.abs(e)
    return integrate.cumtrapz(y, x, initial=0)[-1]

def ISE(e, t):
    x = t - t[0]
    y = np.square(e)
    return integrate.cumtrapz(y, x, initial=0)[-1]

def ITAE(e, t):
    x = t - t[0]
    y = np.abs(e) * x
    return integrate.cumtrapz(y, x, initial=0)[-1]

def RMSE(e):
    return np.sqrt(np.mean(np.square(e)))