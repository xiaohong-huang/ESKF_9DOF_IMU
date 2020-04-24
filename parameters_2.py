import numpy as np
import pandas as pd
from parameters_static import *


#read data
IMUDATA=pd.read_csv("IMU_2.txt").iloc[1:-2].astype(np.float32)
data_gyr1=(IMUDATA[["gyrx","gyry","gyrz"]].values)
data_accel1=IMUDATA[["accx","accy","accz"]].values.astype(np.float)
data_magnitute1=IMUDATA[["magx","magy","magz"]].values.astype(np.float)
data_angle=IMUDATA[["anglex","angley","anglez"]].values.astype(np.float)


