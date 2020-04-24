
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R
import math
from parameters_static import *

#get cross product matrix
def getproductmatrix(theta_n):
    return np.array([[0,-theta_n[2],theta_n[1]],
                     [theta_n[2],0,-theta_n[0]],
                     [-theta_n[1],theta_n[0],0]])
#initialize IMU coordinate use accel and mag
def GetRotation(acc,mag):
    z = acc.copy()
    y = mag.copy()
    x = getproductmatrix(y.reshape((-1))).dot(z.reshape((-1, 1)))
    y = getproductmatrix(z.reshape((-1))).dot(x.reshape((-1, 1)))
    z = z.reshape((-1, 1))
    x /= np.linalg.norm(x)
    y /= np.linalg.norm(y)
    z /= np.linalg.norm(z)
    mRwb = np.hstack((x, y, z)).transpose()

    return mRwb
#update mag world vector
def updatemw(Rbw, inc, mw):
    em = Rbw.T @ inc[9:12]
    em[0] = 0
    fopp = mw[2] - em[2]
    fadj = mw[1] - em[1]
    if (fadj < 0.0):
        fadj = 0.0
    fhyp = math.sqrt(fopp * fopp + fadj * fadj)
    if (fhyp != 0.0):
        ftmp = 1.0 / fhyp
        fsindelta = fopp * ftmp
        fcosdelta = fadj * ftmp

        if (fsindelta > SINDELTAMAX):
            fsindelta = SINDELTAMAX
            fcosdelta = COSDELTAMAX

        else:
            if (fsindelta < -SINDELTAMAX):
                fsindelta = -SINDELTAMAX
                fcosdelta = COSDELTAMAX

        mw[1] = fcosdelta
        mw[2] = fsindelta


