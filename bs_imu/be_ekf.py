import numpy as np
from bs_img import quat_multi_mat




class RelativeEKF:
    R_default = np.diag([0.5,0.5,0.5,0.2,0.2,0.2,0.5,0.5,0.5,0.5])
    Q_default = 50*np.diag([0.5,0.5,0.5,0.2,0.2,0.2,0.5,0.05,0.5,0.5])
    P_default = np.diag([0.5,0.5,0.5,0.2,0.2,0.2,0.05,0.05,0.05,0.05])
    
    def __init__(self):
        # 状态变量pos,vel,quat
        
        # 初始化维度
        self.dim = 10

        # 状态噪声和观测噪声
        self.R = self.R_default
        self.Q = self.Q_default
        self.P_pre = self.P_default


        # 状态变量
        self.X_pre = None


        self.X_post = None
        self.P_post = None


    def set_noise_matrix(self,R,Q):
        # 设置噪声矩阵
        self.R = R
        self.Q = Q

    def init_state(self,X):
        # 初始化状态变量
        self.X_pre = X

    def clac_matirx_F(self,dt,angle_vel):
        # 对应于状态变量pos,vel,quat的矩阵F
        # 构造F
        assis_quat = [0]+(-angle_vel).tolist()
        F = np.identity(self.dim)
        F[:3,3:6] = np.identity(3)*dt
        F[6:,6:] = np.identity(4) + 0.5*dt*quat_multi_mat(assis_quat,-1)
        return F

    def clac_matirx_H(self):
        # 构造H
        H = np.identity(self.dim)
        return H
    
    def prrdict(self,dt,lin_acc,angle_vel):
        F = self.clac_matirx_F(dt,angle_vel)
        uk = np.zeros(10)
        uk[3:6] = -lin_acc*dt

        # 预测
        self.X_post = F.dot(self.X_pre) + uk
        self.P_post = F.dot(self.P_pre.dot(F.T))+self.R

    def update(self,Z):
        # 更新
        H = self.clac_matirx_H()

        inv_K = np.linalg.inv(H.dot(self.P_post.dot(H.T))+self.Q)
        K = np.dot(self.P_post.dot(H.T),inv_K)

        self.X_pre = self.X_post + np.dot(K,Z-H.dot(self.X_post))
        self.P_pre = np.dot(np.identity(self.dim) - K.dot(H),self.P_post)
    
