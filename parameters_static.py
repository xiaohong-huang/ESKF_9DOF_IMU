import numpy as np
# sample rate
deltat=1/100
deltatsq=deltat**2
#PI
PI=3.1415926535898
#acc measurement sigma^2
a_sigma2 =0.0025**2#
#acc bias sigma^2
ba_sigmaa2 =1e-2**2
#mag measurement sigma^2
m_sigma2 =2.5e-2**2
#magbias sigma^2
mb_sigma2 =5e-2**2
#gyr measurement sigma^2
g_sigma2 =6E-12
#gyr bias sigma^2
gb_sigma2 =6E-12
#theta initialize sigma^2
t_initialize_sigma2 =g_sigma2
#gyr bias initialize sigma^2
gb_initialize_sigma2 =gb_sigma2
#acc bias initialize sigma^2
ab_initialize_sigma2 =0.07**2
#mag bias initialize sigma^2
mb_initialize_sigma2 =mb_sigma2
#acc bias decay
FCA_9DOF_GBY_KALMAN =0.5
#initialize Jacobian of acc bias and mag bias
Jacobian=np.zeros((6,12))
Jacobian[0][6] = Jacobian[1][7] = Jacobian[2][8] = -1.0
Jacobian[3][9] = Jacobian[4][10] = Jacobian[5][11] = -1.0
#initialize measurement covariance and inv covariance
Covariance=np.zeros((6,6))
Covariance[0:3,0:3]=np.eye(3)*a_sigma2
Covariance[3:6,3:6]=np.eye(3)*ba_sigmaa2
Rinv = np.linalg.inv(Covariance)
#initialize residual
Residual=np.zeros((6,1))
#initialize update matrix of P
Fx=np.eye(12)
Fx[6:9,6:9]*=FCA_9DOF_GBY_KALMAN
Fx[0:3,3:6]=np.eye(3)*deltat
#initialize update covariance of P
Fi=np.zeros((12,12)).astype(np.float64)
Fi[0:3,0:3]=np.eye(3)*(g_sigma2+gb_sigma2)*deltat**2
Fi[3:6,3:6]=np.eye(3)*gb_sigma2*deltat**2
Fi[6:9,6:9]=np.eye(3)*ba_sigmaa2*deltat**2
Fi[9:12,9:12]=np.eye(3)*mb_sigma2*deltat**2
#initialize world vector of gravity and mag
gw=np.zeros((3,1))
gw[2]=1
mw=np.zeros((3,1))
mw[1]=1
#initialize variable covariance P
P=np.zeros((12,12))
for i in range(3):
    P[i][i] = t_initialize_sigma2
    P[i + 3][i + 3] = gb_initialize_sigma2
    P[i + 6][i + 6] = ab_initialize_sigma2
    P[i + 9][i + 9] = mb_initialize_sigma2
#initialize variable acc bias,mag bias and gyr bias
biasa=np.zeros((3,1))
biasm=np.zeros((3,1))
biasg=np.zeros((3,1))
static=[]
#ehreshold of gyr
threshold=PI/2
#sin and cos limit
SINDELTAMAX =0.9063078
COSDELTAMAX =0.4226183

