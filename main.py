
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R
from parameters_2 import *
from visualization1 import *
import math
from utils import *
begin=0
#initialize IMU coordinate use accel and mag
Rwb=GetRotation(data_accel1[begin], data_magnitute1[begin])
#get initialize rotation from chip
Rw1b1 = R.from_euler("ZYX", [data_angle[begin][2], data_angle[begin][0], data_angle[begin][1]]).as_matrix()
#get rotation from my IMU cordinate to chip coordinate
Rw1w=Rw1b1@Rwb.T
#get rotation from world to IMU
Rbw=Rwb.transpose()
#initialize mag world vector
mw=Rwb@data_magnitute1[begin].reshape((3,1))
mw[0]=0
mw/=np.linalg.norm(mw)


for i in range(begin,len(data_accel1)):
    #get rotation from chip and rotate to my IMU coordinate use Rw1w
    Rwb1 = Rw1w.T@R.from_euler("ZYX", [data_angle[i][2], data_angle[i][0], data_angle[i][1]]).as_matrix()
    gyr=data_gyr1[i].reshape((3,1))
    acc=data_accel1[i].reshape((3,1))
    #normalize mag measurement
    mag=data_magnitute1[i].reshape((3,1))/np.linalg.norm(data_magnitute1[i])
    #update rotation matrix and acc bias
    Rbw = R.from_rotvec((-(gyr - biasg)).reshape((-1)) * deltat).as_matrix() @ Rbw
    biasa = biasa * FCA_9DOF_GBY_KALMAN
    #calculate residual
    Residual[0:3]=acc-biasa-Rbw@gw
    Residual[3:6]=mag-Rbw@mw
    #calculate Jacobian
    Jacobian[0:3, 0:3] = getproductmatrix(Rbw @ gw)
    Jacobian[3:6, 0:3] = getproductmatrix(Rbw @ mw)
    Jacobian[0:3, 3:6] = getproductmatrix(Rbw @ gw)*deltat
    Jacobian[3:6, 3:6] = getproductmatrix(Rbw @ mw)*deltat
    #calculate weight base on gyr
    Weight=np.eye(12)
    if np.linalg.norm(gyr[0:3])>=threshold:
        Weight[6:9, 6:9] *= np.linalg.norm(gyr[0:3])/threshold
    #update P
    Fx[0:3,0:3]=R.from_rotvec((-(gyr-biasg)).reshape((-1))*deltat).as_matrix()
    P[:]=Fx@P@Fx.T+Weight@Fi@Weight.T
    #solve Kalmanfilter
    Hessian=Jacobian.T@Rinv@Jacobian+np.linalg.inv(P)
    P[:]=np.linalg.inv(Hessian)
    b=Jacobian.T@Rinv@Residual
    inc=P@b
    #update variable
    biasa-=inc[6:9]
    biasg-=inc[3:6]
    #avoide large update in theta .threshold is 0.5deg
    if np.linalg.norm(inc[0:3])>PI/180*0.5:
        inc[0:3]/=np.linalg.norm(inc[0:3])
        inc[0:3]*=PI / 180 * 0.5
    Rbw = R.from_rotvec(-inc[0:3].reshape((-1))).as_matrix() @ Rbw
    updatemw(Rbw, inc, mw)
    #visualize IMU eyery 5*0.01sec
    if i%5==0:
        visualization_main(R.from_matrix(Rbw.T).as_quat())
        # visualization_main(R.from_matrix(Rw1b1).as_quat())
    #difference between chip result and my result.use quaternion
    dif = R.from_matrix((Rbw@Rwb1)).as_quat()#Rw1b1.T@Rwb
    print(i,
          R.from_matrix(Rbw.T).as_euler("XYZ")/PI*180,
          R.from_matrix(Rwb1).as_euler("XYZ")/PI*180,
          math.acos(dif[-1]) * 2 / PI * 180,
          Weight[6,6]

          )
    static.append(math.acos(dif[-1]) * 2 / PI * 180)
    pass
print(np.array(static).mean(),np.array(static).min(),np.array(static).max())




