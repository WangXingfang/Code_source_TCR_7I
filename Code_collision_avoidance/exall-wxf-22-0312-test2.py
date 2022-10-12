#!/usr/bin/env python3
# NOTE: 先在本文件夹下打开gepetto-gui，再运行该程序
'''——————————————————
主要内容：
避碰+精确的六边形期望轨迹
——————————————————'''

from re import T
import pinocchio as pin
import numpy as np
from numpy.linalg import inv, pinv
from os.path import dirname, join, abspath
from pinocchio.visualize import GepettoVisualizer
import time
from scipy.signal import savgol_filter
import cv2
from numpy.core.fromnumeric import reshape
from KinectToolBox import ktb
import open3d as o3d
import copy
import rospy
from sensor_msgs.msg import JointState
from robot_msgs.srv import tcrScript
from robot_msgs.msg import systemState
import sys
import threading
import socket


class RobWithCam:
    '''
    '''
    # 载入模型
    def __init__(self):
        # Load the URDF model. Conversion with str seems to be necessary when executing this file with ipython
        pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))),"HRI_wxf")
        mesh_dir = pinocchio_model_dir
        urdf_model_path = pinocchio_model_dir + '/armmodel/urdf/armmodel(tool).urdf'
        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir)
        self.data = self.model.createData()            # 创建数据模型
        
        self.q = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])               # 初始角度，直立
        self.q0 = self.q                              # 保存上一时刻的关节角度
        self.vq = np.zeros([2,self.model.nv])          # 前一时刻和现在的角速度
        self.oMtool = np.array([0,0,0])                # 机械臂末端的位置
        self.fcobs = np.zeros([3,6])                   # 3个关节前后时刻的力
        self.qobs = np.zeros(8)                        
        # 关节限角的设置
        self.jointlowerlimit = np.array([-2.967, -2.094, -2.967, -2.792, -2.967, -0.5, -0.5, -2.967])
        self.jointupperlimit = np.array([ 2.967,  2.094,  2.967,      0,  2.967,  1.8,  1.5,  2.967])
        self.closedistance = np.zeros(2)
        "-----正常工作轨迹定义-----"
        q_all_hexagon = np.load('q_all_hexagon.npy') # [n,7]
        self.oMgoal_all = q_all_hexagon[3500:, :]    # 六边形轨迹在关节空间的集合
        self.goal_length = len(self.oMgoal_all)      # 轨迹集合的个数
        self.goalnum = 0    # 向哪个目标移动
        self.fast_work = False
        self.slow_work = False
        self.bet_fa2sl = False
        "---视觉部分定义---"
        self.kinect2 = ktb.Kinect()
        self.pointcloud = o3d.geometry.PointCloud()
        # 虚拟机械臂骨架坐标转换
        self.rob_cam = np.zeros([3,3]) # [x,y,z]
        self.rob_dep = np.zeros([3,2]) # [u,v]
        # 内参系数
        # self.f_x = 365.36151123046875
        # self.f_y = 365.36151123046875
        # self.c_x = 256.1164855957031
        # self.c_y = 211.66619873046875
        self.tf_rob_2_cam = np.zeros([4,4])
        self.tf_cam_2_rob = np.zeros([4,4])

        '---远程控制线程定义---'
        self.state0 = np.zeros(7)      # 机械臂此刻关节状态
        self.state1 = np.zeros(7)      # 下一时刻
        self.state_now  = np.zeros(7)  # 实时更新当前状态
        self.control0 = np.zeros(7)    # 此刻的控制命令
        self.control1 = np.zeros(7)    # 下一时刻
        self.control_m0 = np.zeros(7)    # 缓存此刻控制命令
        self.control_m1 = np.zeros(7)    # 缓存下一时刻控制命令
        self.thread_state = True       # 线程状态
        self.con_update = False        # 控制命令是否更新

        '-----实验数据的保存-----'
        self.q_all_save0 = [] # 保存关节角度，1000hz
        self.q_all_save1 = [] # 保存关节角度，和程序同频
        self.clo_dis_save = [] # 障碍到骨架的最近距离

    def tf_calibration(self): # 手眼标定结果，相机与基座的坐标转换关系
        T = np.eye(4)
        el_angles = np.array([-78,-1.0,-170])
        rad_angles = el_angles * np.pi / 180
        theta = rad_angles
        R_x = np.array([[1,         0,                  0                 ],
                        [0,         np.cos(theta[0]),  -np.sin(theta[0])  ],
                        [0,         np.sin(theta[0]),   np.cos(theta[0])  ]
                        ])
        R_y = np.array([[np.cos(theta[1]),      0,      np.sin(theta[1])  ],
                        [0,                     1,      0                 ],
                        [-np.sin(theta[1]),     0,      np.cos(theta[1])  ]
                        ])
        R_z = np.array([[np.cos(theta[2]),     -np.sin(theta[2]),     0],
                        [np.sin(theta[2]),      np.cos(theta[2]),     0],
                        [0,                     0,                    1]])
        R = np.dot(R_x, np.dot( R_y, R_z ))
        T[:3,:3] = R
        T[:3,3] = np.array([0.14,-0.33,1.36])
        # print(T)
        #齐次转换矩阵
        self.tf_rob_2_cam = T
        self.tf_cam_2_rob = np.linalg.inv(self.tf_rob_2_cam)


    def DeptoCloud(self):      
        depth_frame0 = self.kinect2.get_depthframe() # 获得深度图
        depth_frame = copy.deepcopy(depth_frame0)
        # 只保留深度为（0.8m, 2.0m）之间的数据
        depth_frame[depth_frame < 500] = 0
        depth_frame[depth_frame > 2500] = 0                                             # 0.033 ｓ
        # 3D数据，单位为m
        xyz = self.kinect2.get_depth_ptcld(depth_frame, scale=1000)                                # 0.048 s
        xyz = reshape(xyz,(-1,3))
        # 坐标轴修正，与相机坐标轴重合
        xyz[:,0],xyz[:,1],xyz[:,2] = xyz[:,1].copy(),xyz[:,2].copy(),xyz[:,0].copy()    # 0.050 s        
        '根据机械臂关节位置,实时改变机械臂的探测位置,外扩0.4m'
        rob_world = np.ones([4,3]) # [x,y,z,1]
        rob_world[0:3,:] = np.transpose(self.ForwardKinematics()) # 关节坐标，基座坐标系[x,y,z,1].T
        rob_cam0 = np.dot(self.tf_rob_2_cam,rob_world) # 关节坐标，相机坐标系[x,y,z,1].T
        self.rob_cam = np.transpose(rob_cam0[0:3,:]) # [x,y,z] [3*3]矩阵
        xyz = xyz[xyz[:,0] > (np.min(rob_cam0[0,:])-0.4)]
        xyz = xyz[xyz[:,0] < (np.max(rob_cam0[0,:])+0.4)]
        xyz = xyz[xyz[:,1] > (np.min(rob_cam0[1,:])-0.4)]
        xyz = xyz[xyz[:,1] < (np.max(rob_cam0[1,:])+0.4)]
        xyz = xyz[xyz[:,2] > (np.min(rob_cam0[2,:])-0.4)]
        xyz = xyz[xyz[:,2] < (np.max(rob_cam0[2,:])+0.4)]
        '将机械臂形成包络，从深度空间中剔除'
        value0 = (np.sqrt(np.sum((xyz- rob_cam0[:3,0])**2 ,axis=1)))
        xyz = xyz[value0>0.50,:] # 障碍到基座的距离大于0.35
        for j in range(0,2):  # 对于每段机械臂部分      
            vec_line = (rob_cam0[:3,j+1] - rob_cam0[:3,j]).reshape(1,3)      # 机械臂的矢量[1,3]            
            vec_ob_line = xyz - rob_cam0[:3,j]                # 从障碍到关节的矢量[n,3]           
            pro = np.dot(vec_ob_line, vec_line.T) / np.sum((vec_line)**2)  # 计算两个矢量的比例关系[n,1]
            pro = pro.reshape(len(pro),1)
            cona = np.argwhere(pro<0)
            conb = np.argwhere(pro>1)
            if j == 0: # 第一段连杆
                clo_pos0 = rob_cam0[:3,j] + np.dot(pro,vec_line)           # 最近点位于中间[n,3]
                clo_pos0[cona,:] = rob_cam0[:3,j]      # 最近点位于开头
                clo_pos0[conb,:] = rob_cam0[:3,j+1]    # 最近点位于结尾
                clo_dis0 = (np.sqrt(np.sum((xyz - clo_pos0)**2 ,axis=1))) # 障碍到机械臂的最近距离[n,1]
                xyz = xyz[clo_dis0>0.20,:] # 障碍到第一连杆的距离大于0.1
            if j == 1: # 第二段连杆
                clo_pos1 = rob_cam0[:3,j] + np.dot(pro,vec_line)           # 最近点位于中间[n,3]
                clo_pos1[cona,:] = rob_cam0[:3,j]      # 最近点位于开头
                clo_pos1[conb,:] = rob_cam0[:3,j+1]    # 最近点位于结尾
                clo_dis1 = (np.sqrt(np.sum((xyz - clo_pos1)**2 ,axis=1)))# 障碍到机械臂的最近距离[n,1]
                xyz = xyz[clo_dis1>0.20,:] # 障碍到第二连杆的距离大于0.1
        value1 = (np.sqrt(np.sum((xyz- rob_cam0[:3,1])**2 ,axis=1)))
        xyz = xyz[value1>0.25,:] # 障碍到基座的距离大于0.35
        value2 = (np.sqrt(np.sum((xyz- rob_cam0[:3,2])**2 ,axis=1)))
        xyz = xyz[value2>0.25,:] # 障碍到基座的距离大于0.35
        # 数组变为点云格式
        self.pointcloud.points = o3d.utility.Vector3dVector(xyz)
        # 下采样/体素滤波
        self.pointcloud = self.pointcloud.voxel_down_sample(voxel_size=0.02)                     # 0.052 s
        # 剔除噪声（统计式离群点移除）
        cl, ind = self.pointcloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.5)
        self.pointcloud = self.pointcloud.select_by_index(ind)                                  # 0.075 s
        return depth_frame0

    def CloudtoRepulsion(self):
        # rob_world = np.ones([4,3]) # [x,y,z,1]
        # rob_world[0:3,:] = np.transpose(self.ForwardKinematics()) # 关节坐标，基座坐标系[x,y,z,1].T
        # rob_cam0 = np.dot(self.tf_rob_2_cam,rob_world)
        # self.rob_cam = np.transpose(rob_cam0[0:3,:]) # [x,y,z] [3*3]矩阵
        re_vec_cam = np.zeros([3,3])  # 排斥向量，相机坐标系下||最近距离||对应最近障碍的排斥力数值
        clo_points0 = np.zeros([1,7]) # 第一连杆对应的最近点(clo_dis/obs_cam/clo_pos)
        clo_points1 = np.zeros([1,7]) # 第二连杆对应的最近点
        for i in range(1):
            "------排斥向量计算--------"
            # 确定障碍点到机械臂的最近距离与对应的关节序号
            obs_cam = np.asarray(self.pointcloud.points)                  
            for j in range(0,2):  # 对于每段机械臂部分      
                vec_line = (self.rob_cam[j+1,:] - self.rob_cam[j,:]).reshape(1,3)      # 机械臂的矢量[1,3]            
                vec_ob_line = obs_cam - self.rob_cam[j,:]                # 从障碍到关节的矢量[n,3]           
                pro = np.dot(vec_ob_line, vec_line.T) / np.sum((vec_line)**2)  # 计算两个矢量的比例关系[n,1]
                pro = pro.reshape(len(pro),1)
                cona = np.argwhere(pro<0)
                conb = np.argwhere(pro>1)
                if j == 0: # 第一段连杆
                    clo_pos0 = self.rob_cam[j,:] + np.dot(pro,vec_line)           # 最近点位于中间[n,3]
                    clo_pos0[cona,:] = self.rob_cam[j,:]      # 最近点位于开头
                    clo_pos0[conb,:] = self.rob_cam[j+1,:]    # 最近点位于结尾
                    clo_dis0 = (np.sqrt(np.sum((obs_cam - clo_pos0)**2 ,axis=1))).reshape(len(pro),1) # 障碍到机械臂的最近距离[n,1]
                    clo_link0 = pro  # 障碍作用的关节序号[n,1]
                    clo_link0 = 0 + pro
                    clo_link0[cona] = 0
                    clo_link0[conb] = 1
                    num = np.argmin(clo_dis0)
                    clo_points0[i,0]   = clo_dis0[num, 0]
                    clo_points0[i,1:4] = obs_cam[num,:]
                    clo_points0[i,4:7] = clo_pos0[num,:]
                if j == 1: # 第二段连杆
                    clo_pos1 = self.rob_cam[j,:] + np.dot(pro,vec_line)           # 最近点位于中间[n,3]
                    clo_pos1[cona,:] = self.rob_cam[j,:]      # 最近点位于开头
                    clo_pos1[conb,:] = self.rob_cam[j+1,:]    # 最近点位于结尾
                    clo_dis1 = (np.sqrt(np.sum((obs_cam - clo_pos1)**2 ,axis=1))).reshape(len(pro),1) # 障碍到机械臂的最近距离[n,1]
                    clo_link1 = pro  # 障碍作用的关节序号[n,1]
                    clo_link1 = 1 + pro
                    clo_link1[cona] = 1
                    clo_link1[conb] = 2   
                    num = np.argmin(clo_dis1)
                    clo_points1[i,0]   = clo_dis1[num, 0]
                    clo_points1[i,1:4] = obs_cam[num,:]
                    clo_points1[i,4:7] = clo_pos1[num,:]
            '------当无障碍时才向目标位置移动------'
            self.closedistance[0] = np.min(clo_dis0)
            self.closedistance[1] = np.min(clo_dis1)
            if self.closedistance[0] > 0.35 and self.closedistance[1] > 0.35:
                print('safe !')
            else:
                self.fast_work = False # 不进行任何正常工作
                self.slow_work = False          
            
            # 计算排斥向量
            if np.min(clo_dis0) < 0.40: # 第一个连杆离障碍较近
                obs_num = np.argwhere(clo_dis0[:,0]<0.40) # 只对阈值内障碍计算排斥向量[m,1]
                vec = clo_pos0[obs_num[:,0],:] - obs_cam[obs_num[:,0],:] # 目标-障碍[m,3]
                dist = clo_dis0[obs_num[:,0],:] # [m,1]
                f_max = 200
                pp = 0.08
                aa = 0.12
                r = 0.40
                fc = f_max / (1+np.exp((dist*2/pp-1)*aa)) * vec/dist # 排斥力[m,3]
                num0 = np.argmin(dist[:,0])
                fc_max = np.linalg.norm(fc[num0,:])   # 最大的排斥力
                re_j0 = np.sum(fc*(1-clo_link0[obs_num[:,0],:]), axis=0)# 考虑所有障碍点，决定方向
                if np.linalg.norm(re_j0) != 0:
                    re_j0 = re_j0/np.linalg.norm(re_j0)
                    re_vec_cam[0,0:3] = re_j0*fc_max      # 最大排斥力决定大小
                re_j1 = np.sum(fc*clo_link0[obs_num[:,0],:], axis=0)
                if np.linalg.norm(re_j1) !=0:
                    re_j1 = re_j1/np.linalg.norm(re_j1)
                    re_vec_cam[1,0:3] = re_j1*fc_max
            if np.min(clo_dis1) < 0.40: # 第二个连杆离障碍较近
                obs_num = np.argwhere(clo_dis1[:,0]<0.40) # 只对阈值内障碍计算排斥向量[m,1]
                vec = clo_pos1[obs_num[:,0],:] - obs_cam[obs_num[:,0],:] # 目标-障碍[n,3]
                dist = clo_dis1[obs_num[:,0],:] # [n,1]
                f_max = 200
                pp = 0.08
                aa = 0.12
                r = 0.40
                fc = f_max / (1+np.exp((dist*2/pp-1)*aa)) * vec/dist # 排斥力[n,3]
                num1 = np.argmin(dist[:,0])
                fc_max1 = np.linalg.norm(fc[num1,:])   # 最大的排斥力
                re_j11 = np.sum(fc*(2-clo_link1[obs_num[:,0],:]), axis=0)# 考虑所有障碍点，决定方向
                if np.linalg.norm(re_j11) != 0:
                    if np.min(clo_dis0) < 0.40:
                        re_j11 = (re_j11+re_j1)/np.linalg.norm(re_j11+re_j1)
                        re_vec_cam[1,0:3] = re_j11*fc_max
                        if fc_max1 > fc_max:
                            re_vec_cam[1,0:3] = re_j11*fc_max1       # 最大排斥力决定大小
                    else:
                        re_j11 = re_j11/np.linalg.norm(re_j11)
                        re_vec_cam[1,0:3] = re_j11*fc_max1       # 最大排斥力决定大小
                re_j2 = np.sum(fc*(clo_link1[obs_num[:,0],:]-1), axis=0)
                if np.linalg.norm(re_j2) != 0:
                    re_j2 = re_j2/np.linalg.norm(re_j2)
                    re_vec_cam[2,0:3] = re_j2*fc_max1
        # 排斥向量转换到基座坐标系下
        re_vec_cam0 = np.ones([4,3])
        re_vec_cam0[0:3,:] = np.transpose(re_vec_cam) # [x,y,z,1].T
        re_vec_world0 = np.dot(self.tf_cam_2_rob,re_vec_cam0)
        re_vec_world0[0:3,:] += -self.tf_cam_2_rob[0:3,-1].reshape(3,1)
        re_vec_world = np.transpose(re_vec_world0[0:3,:])
        # 确定障碍和机械臂的最近点
        clo_points2 = np.zeros([2,7])
        if len(clo_points0)>0 and len(clo_points1) > 0:
            clo_points2[0,:] = clo_points0[np.argmin(clo_points0[:,0]), :]
            clo_points2[1,:] = clo_points1[np.argmin(clo_points1[:,0]), :]
        else:
            clo_points2[:,0] = 100

        return re_vec_cam, re_vec_world, clo_points2

    # 求关节扭矩，动力学
    def RepulsiontoTau(self,re_vec_world):
        fc = re_vec_world # [3,3]
        # print("fc0: ", fc)
        tau = np.zeros([8])
        self.fcobs[:,0:3] = self.fcobs[:,3:6]  # 保存当前虚拟力
        self.fcobs[:,3:6] = fc                 # 保存之后虚拟力
        fc = self.ComputeObsVel()      # 引入障碍速度的计算
        J1 = self.ComputeLinkJacobian("link1")[:3]
        J3 = self.ComputeLinkJacobian("link3")[:3]
        J5 = self.ComputeLinkJacobian("link5")[:3]
        tau += np.transpose(J1) @ fc[0,:]
        tau += np.transpose(J3) @ fc[1,:]
        tau += np.transpose(J5) @ fc[2,:]
        # print("fc1: ", fc) 
        # print("tau: ", tau)  
        return tau

    # 限制关节角度 下一时刻的关节角度
    def JointLimit(self,q,vq,t):
        q1 = pin.integrate(self.model,q,vq*t)
        q1 = np.array(q1)
        mes   = np.zeros(3)  # 距离限角的最小值||下/上限角||快要到达限角的关节代号
        q_des = np.abs(q1 - self.jointlowerlimit)
        mes[0] = 0                      # 先考虑下限关节角度
        mes[1] = np.min(q_des)          # 最小角度差
        mes[2] = np.argmin(q_des)       # 最小值的所在关节代号（0-7）
        q_des = np.abs(q1 - self.jointupperlimit)
        if np.min(q_des) < mes[1]:      # 若更靠近上限角度
            mes[0] = 1                  # 代表上限角度
            mes[1] = np.min(q_des)
            mes[2] = np.argmin(q_des)
        # 将关节限角差转换为施加到对应关节的扭矩
        if mes[1] < 0.524:              # 阈值为30°||0.524
            tau_max = 3
            p = 0.4
            a = 3
            tau = tau_max / (1 + np.exp((mes[1]*2/p-1.5)*a))
            if int(mes[0]) == 1:
                tau = -tau              # 扭矩反向
            tauq = np.zeros(8)
            tauq[int(mes[2])] = tau          # 只在对应关节施加扭矩             
            M = pin.crba(self.model,self.data,q1) # 惯性矩阵
            # b = pin.rnea(self.model,self.data,self.q,np.zeros(self.model.nv),np.zeros(self.model.nv)) # 偏差
            aq = inv(M) @ tauq
            dt = 0.00001
            vq = aq*dt                            # 下一时刻的角速度
            # vq = np.clip(vq, -1, 1)           # 限制最值
            q2 = pin.integrate(self.model,q1,vq*dt) 
        else:
            q2 = q1

        return q2

    # 把六维姿态转换到李群(旋转矩阵+平移矩阵)
    def XYZRPYtoSE3(self,xyzrpy):
        rotate = pin.utils.rotate
        R = rotate('x',xyzrpy[3]) @ rotate('y',xyzrpy[4]) @ rotate('z',xyzrpy[5])
        p = np.array(xyzrpy[:3])
        return pin.SE3(R,p)

    # # 逆运动学，使得机械臂末端向目标位置移动
    # def InverseKinematics(self):
    #     der_q = np.array([0.2, 0.6, 0, -1.5, 0, 0, 0, 0]) - self.q
    #     qq = np.linalg.norm(der_q)
    #     if qq > 0.3:
    #         vq_max = 0.10
    #         ap = 0.3
    #         bp = -2.5
    #         cp = 5
    #         fc = vq_max / (1+ap*np.exp(bp*qq+cp))
    #         vq = der_q / qq * fc
    #         q = self.JointLimit(self.q,vq,1)
    #     else:
    #         q = self.q
    #     # q = pin.integrate(self.model,self.q,vq*DT)
    #     return q

    # 逆运动学，使得机械臂末端向目标位置移动
    # def InverseKinematics(self):
    #     '''
    #     进入正常工作模式，分快速工作和慢速工作两种情况
    #     快速工作：从避碰状态快速返回期望轨迹
    #     慢速工作：在快速工作之后，慢速跟踪期望轨迹
    #     '''
    #     der_q = np.zeros(8)
    #     der_q[:7] = self.oMgoal_all[self.goalnum, :7] - self.q[:7]
    #     norm_derq = np.linalg.norm(der_q)
    #     if norm_derq > 1.0 or self.fast_work == True:
    #         self.fast_work = True
    #         self.slow_work = False
    #         vq = der_q / norm_derq * 0.35
    #         q = self.JointLimit(self.q, vq, 1)
    #     else:
    #         self.fast_work = True
    #         self.slow_work = False
    #         q = copy.deepcopy(self.q)
            
    #     return q


    # 根据关节角度求解关节的空间位置,注意q是否为期望的关节状态
    def ForwardKinematics(self):
        pin.forwardKinematics(self.model,self.data,self.q0)       
        pos_armmodel = np.zeros([3,3]) # 一共三个坐标系，0-2/3-4/5-7
        pos_armmodel[0,:] = np.array(self.data.oMi[0].translation[[0,1,2]]) #基座的位置
        pos_armmodel[1,:] = np.array(self.data.oMi[3].translation[[0,1,2]]) #关节3位置
        pos_armmodel[2,:] = np.array(self.data.oMi[5].translation[[0,1,2]]) #关节5位置
        return pos_armmodel
  
    # 雅克比矩阵的求解,针对一个连杆
    def ComputeLinkJacobian(self,jointid):
        pin.forwardKinematics(self.model,self.data,self.q)
        pin.framesForwardKinematics(self.model,self.data,self.q)
        pin.computeJointJacobians(self.model,self.data,self.q)
        J1 = pin.getFrameJacobian(self.model,self.data,self.model.getFrameId(jointid),pin.LOCAL_WORLD_ALIGNED)
        return J1
    
    # 雅克比矩阵的求解,针对一个关节
    def ComputeJointJacobian(self,jointid):
        pin.computeJointJacobians(self.model, self.data, self.q)
        armjointX = self.model.getJointId(jointid)
        J1 = pin.getJointJacobian(self.model,self.data,armjointX,pin.LOCAL_WORLD_ALIGNED)
        return J1
    
    # 计算末端的六维位姿，输入需要的关节角度
    def ComputePosJee(self,q):
        # 求末端的六维位置/速度
        pin.forwardKinematics(self.model,self.data,q) 
        pos_j8 = np.zeros(6)
        # oMi[0]为universe，j9和link8末端固连，只能求到j8（没加末端执行器之前的真正末端）      
        pos_j8[0:3] = self.data.oMi[8].translation[[0,1,2]]
        pos_j8[3:6] = pin.utils.matrixToRpy(self.data.oMi[8].rotation)
        # pos_j8[3:6][pos_j8[3:6] >  2.8] =  2.8 #防止姿态角从π突变到-π
        # pos_j8[3:6][pos_j8[3:6] < -2.8] = -2.8        
        return pos_j8
    
    # "---------重新设计障碍速度的引入方法----------"
    # pivot算法使得机械臂的抖动比较厉害，与他的原理有关
    def ComputeObsVel(self):
        f_3 = np.zeros([3,3])
        for i in range(3):
            f_0   = self.fcobs[i,0:3]  # 当前力
            f_1   = self.fcobs[i,3:6]  # 之后力
            f_der = (f_1 - f_0)        # 力导数
            f_0_norm = np.linalg.norm(f_0)
            f_1_norm = np.linalg.norm(f_1)
            f_der_norm = np.linalg.norm(f_der)
            if f_0_norm > 0.1 and  f_1_norm > 0.1:
                f_1_dir    = f_1 / f_1_norm     # 之后力的单位向量
                f_der_dir  = f_der / f_der_norm # 力导数的单位向量
                p_1 = np.arccos(f_der_dir@f_1_dir)
                if p_1 < 1.2:
                    n_1 = np.cross(f_1_dir,f_der_dir)
                    v_1 = np.cross(n_1,f_1_dir)
                    v_1 = v_1 / np.linalg.norm(v_1)
                    f_limit = np.sin(1.05)*f_1_dir + np.cos(1.05)*v_1 # 最大转向30°
                    # 当速度超过阈值时，使得力的方向更加倾向于垂直方向
                    mm = 1 / (0.5 + np.exp(-0.5 * (f_der_norm-3))) - 0.667
                    f_2 = np.sin(p_1)*f_1_dir + np.cos(p_1)*np.exp(mm)*f_limit
                    f_3[i,:] = f_1_norm * f_2 / np.linalg.norm(f_2)
                else:
                    f_3[i,:] = f_1
            else:
                f_3[i,:] = f_1
     
        return f_3

    # 零空间控制
    def NullSpaceFilter(self):
        A0 = self.ComputeLinkJacobian("link9")
        A1 = pinv(A0[0:3,:]) # 位置雅可比的伪逆
        A2 = pinv(A0[3:6,:]) # 姿态雅可比的伪逆
        r_des = self.ComputePosJee(self.q) # 期望的末端位姿
        "--------------什么才是真正期望的末端位姿------------"
        r_des[1] = -0.5 
        r_des[2] =  0.4
        
        # v_des = (self.q - self.q0) * 0.1  # 期望的末端速度
        # q1  = self.JointLimit(self.q,self.vq[1,:],0.1)
        q1 = pin.integrate(self.model,self.q,self.vq[1,:]*1) 
        r = self.ComputePosJee(q1) # 下一时刻的末端位姿（避碰）
        # v = (q1 - self.q0) * 0.1  # 下一时刻的末端速度（避碰）
        v_der  = pin.difference(self.model,self.q0,self.q)
        v_der -= pin.difference(self.model,self.q0,q1)
        r_der = r_des - r
        kp1 = 40
        kd1 = 0.2
        kp2 = 2
        kd2 = 0.05
        r1_out = kp1*r_der[0:3] + kd1*v_der[0:3] # PD控制(线速度)
        r2_out = kp2*r_der[3:6] + kd2*v_der[3:6] # PD控制（角速度）
        dq = A1 @ r1_out + (np.eye(8) - A1 @ A0[0:3,:]) @ A2 @ r2_out
        # dq = A2 @ r2_out + (np.eye(8) - A2 @ A0[3:6,:]) @ A1 @ r1_out
        return dq

    # 根据扭矩求期望的关节角度
    def ComputeAngleFromTau(self,tauq):
        M = pin.crba(self.model,self.data,self.q) # 惯性矩阵
        # b = pin.rnea(self.model,self.data,self.q,np.zeros(self.model.nv),np.zeros(self.model.nv)) # 偏差
        aq = inv(M) @ tauq
        ### Integrate the acceleration twice instead of this comment
        dt = 0.01
        self.vq[0,:] = self.vq[1,:] # 保存目前角速度
        "-----------能否使用v = v0 + 1/2*a*t^2-------------"
        self.vq[1,:] = aq*dt        # 下一时刻的角速度
        self.vq[1,:] = self.vq[1,:]*0.7 + self.vq[0,:]*0.3  # 滤波，角速度的平滑性
        self.vq[1,:] = np.clip(self.vq[1,:], -0.8, 0.8) # 限制最值
        if np.linalg.norm(tauq) > 1e-5: # 零空间控制
            dq1 = self.vq[1,:]
            dq2 = self.NullSpaceFilter()
            q_obs_1 = self.JointLimit(self.q,dq2,0.02)
            q_obs_1 = self.JointLimit(q_obs_1,dq1,1)   
        else:
            q_obs_1 = self.JointLimit(self.q,self.vq[1,:],1)
        "--------暂时取消零空间控制--------"
        q_obs_1 = self.JointLimit(self.q,self.vq[1,:],1)
        q_obs_1 = q_obs_1 * 0.8 + self.q * 0.2 # 平滑一下
        return q_obs_1

    def DepthDisplay(self, re_vec_cam, depth_frame, clo_points):
        # 将施加到每个关节的排斥向量映射到像素坐标系
        re_vec_dep = np.zeros([3,3])  # 排斥向量，像素坐标系下||是否有排斥向量(0/1)
        re_vec_cam[:,0:3] = re_vec_cam[:,0:3]/200
        re_vec_cam[:,0:3] += self.rob_cam
        for i in range(0,3):
            if np.linalg.norm(re_vec_cam[i,0:3]) != 0:
                re_vec_dep[i,0] = 365.3615*re_vec_cam[i,0]/re_vec_cam[i,2] + 256.1165
                re_vec_dep[i,1] = 365.3615*re_vec_cam[i,1]/re_vec_cam[i,2] + 211.6662
                re_vec_dep[i,2] = 1
        re_vec_dep = re_vec_dep.astype(int)
        re_vec_dep[:,1] = 424 - re_vec_dep[:,1]
        # 图像实时显示
        depth_frame[depth_frame < 500] = 0
        depth_frame[depth_frame > 3000] = 0    
        depth_frame = ((depth_frame-500)*0.1)
        depth_frame[depth_frame < 0] = 0
        depth_frame = depth_frame.astype(np.uint8)
        rgb_frame = cv2.cvtColor(depth_frame,cv2.COLOR_GRAY2RGB)
        # 更新机械臂的像素坐标
        # self.f_x = 365.36151123046875
        # self.f_y = 365.36151123046875
        # self.c_x = 256.1164855957031
        # self.c_y = 211.66619873046875
        self.rob_dep[:,0] = 365.3615*self.rob_cam[:,0]/self.rob_cam[:,2] + 256.1165
        self.rob_dep[:,1] = 365.3615*self.rob_cam[:,1]/self.rob_cam[:,2] + 211.6662
        self.rob_dep = self.rob_dep.astype(int)
        self.rob_dep[:,1] = 424 - self.rob_dep[:,1]
        "----在图中显示虚拟机械臂骨架的位置-----"
        cv2.circle(rgb_frame, (self.rob_dep[0][0], self.rob_dep[0][1]), 5, (0, 255, 0), 2)
        for i in range(0,2):
            cv2.line(rgb_frame, (self.rob_dep[i][0], self.rob_dep[i][1]), (self.rob_dep[i+1][0], self.rob_dep[i+1][1]), (0, 255, 0), 2)
            cv2.circle(rgb_frame, (self.rob_dep[i+1][0], self.rob_dep[i+1][1]), 5, (0, 255, 0), 2)
        "----显示排斥向量-----"
        for i in range(0,3):
            if re_vec_dep[i,2] == 1:
                cv2.arrowedLine(rgb_frame, (self.rob_dep[i,0],self.rob_dep[i,1]), (re_vec_dep[i,0], re_vec_dep[i,1]), (0,0,255), 2, 0, 0, 0.2) 
        "----显示最近点----"
        for i in range(0,2):
            if clo_points[i,0] < 0.40:
                clo_points_dep = np.zeros([2,2])
                clo_points_dep[0,0] = 365.3615*clo_points[i,1]/clo_points[i,3] + 256.1165
                clo_points_dep[0,1] = 365.3615*clo_points[i,2]/clo_points[i,3] + 211.6662
                clo_points_dep[1,0] = 365.3615*clo_points[i,4]/clo_points[i,6] + 256.1165
                clo_points_dep[1,1] = 365.3615*clo_points[i,5]/clo_points[i,6] + 211.6662
                clo_points_dep = clo_points_dep.astype(int)
                clo_points_dep[:,1] = 424 - clo_points_dep[:,1]
                cv2.circle(rgb_frame, (clo_points_dep[0,0], clo_points_dep[0,1]), 5, (255, 0, 0), 2)
                cv2.circle(rgb_frame, (clo_points_dep[1,0], clo_points_dep[1,1]), 5, (255, 0, 0), 2)
                cv2.line(rgb_frame, (clo_points_dep[0,0], clo_points_dep[0,1]), (clo_points_dep[1,0], clo_points_dep[1,1]), (255, 0, 0), 2)
        cv2.imshow('rgb_frame', rgb_frame)

    # 建立发送命令的线程，定周期以8ms发送
    def send_s1(self):
        socket_send = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        count = 0
        while self.thread_state:
            try:
                t0 = time.time()
                if self.con_update:
                    count = 19
                    # self.state0 = self.state_now
                    self.con_update = False
                if count > 0:  # 两个命令间的过渡
                    bl = count / 20
                    dc = bl * (self.control_m1 - self.control_m0) / 280 + (1 - bl) * (self.control1 - self.control0) / 280
                    count -= 1
                else:
                    dc = (self.control1 - self.control0) / 280
                dc = np.clip(dc,-0.0005,0.0005)
                self.state1 = self.state0 + dc
                # 不再避碰，正常工作
                if self.closedistance[0] > 0.35 and self.closedistance[1] > 0.35:
                    # 快速返回期望轨迹的情况
                    if self.slow_work == False:
                        self.fast_work = True
                        der_q = self.oMgoal_all[self.goalnum, :7] - self.state0
                        mo_q = np.linalg.norm(der_q)
                        if mo_q > 0.0015:
                            dc += der_q / mo_q * 0.0013
                            dc = np.clip(dc,-0.0010,0.0013)
                            self.state1 = self.state0 + dc
                        else:
                            self.slow_work = True
                            self.fast_work = False
                    # 六边形轨迹运动
                    if self.slow_work == True:
                        if self.goalnum >= self.goal_length-1:
                            self.goalnum = 0
                        self.goalnum += 1 # 下一个目标位置
                        self.state1 = self.oMgoal_all[self.goalnum, :7]

                # 关节角度的最后保障
                q1lowerlimit_if = self.state1[:] > self.jointlowerlimit[:7]
                q1upperlimit_if = self.state1[:] < self.jointupperlimit[:7]
                if False in q1lowerlimit_if:
                    xx = np.argwhere(q1lowerlimit_if == False).flatten()
                    for xxx in enumerate(xx):
                        self.state1[xxx[1]] = self.jointlowerlimit[xxx[1]]
                if False in q1upperlimit_if:
                    bb = np.argwhere(q1upperlimit_if == False).flatten()
                    for bbb in enumerate(bb):
                        self.state1[bbb[1]] = self.jointupperlimit[bbb[1]]
                # 保存关节角度
                self.q_all_save0.append(self.state1)
                self.state0 = self.state1
                aa = copy.deepcopy(self.state1)
                task_command_str = 'servoJ(0,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)'%(aa[0],aa[1],aa[2],aa[3],aa[4],aa[5],aa[6])
                print('task_command: ', task_command_str)
                socket_send.sendto(task_command_str.encode(),("192.168.0.101",8000))
                t1 = time.time()
                dt = 0.001 - (t1 - t0)
                if dt > 0:
                    print('时间再延时',dt,'s')
                    time.sleep(dt)
            except Exception as e:
                print('send_s1 error!', e)
                sys.exit(1)



    "-------主循环-------"
    # 循环改变机械臂位姿并显示
    def CyclicDisplay(self,viz):
        # global command_send
        self.viz = viz
        # 实时显示曲线
        m = np.zeros([8,50])
        m1 = m
        j = 0
        k = 0

        socket_send = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)       
        # 提前准备
        task_command = 'EnMotor(0,-1)'
        print(task_command)
        socket_send.sendto(task_command.encode(),("192.168.0.101",8000))
        input() #输入任意按键进行下一步
        task_command = 'moveJ(0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.1)'
        print(task_command)
        socket_send.sendto(task_command.encode(),("192.168.0.101",8000))
        input() #输入任意按键进行下一步  

        xxx = 0
        run_time = []

        # 建立远程通信，ros接口
        rospy.init_node('arm_listener', anonymous=True)
        # command_send = rospy.ServiceProxy('tcrScript', tcrScript)
        # 开启线程
        thread1 = threading.Thread(target=self.send_s1)
        thread1.start()
        # thread2 = threading.Thread(target=self.rev_state_now)
        # thread2.start()
        while True:
            try:
                data_now = rospy.wait_for_message('joint_states', JointState, timeout=None)
                # print(data_now.position)
                self.q[0:7] = np.array(data_now.position)



                # # 计算运行时间
                # if xxx == 0:
                #     begin_time = time.time()
                #     xxx = 1
                # else:
                #     end_time = time.time()
                #     der_time = end_time - begin_time
                #     run_time.append(der_time)
                #     xxx = 0
                # if len(run_time) == 200:
                #     print('平均运行时间：　', sum(run_time)/200)
                #     print('最大运行时间：　', max(run_time))
                #     # break



                self.q0 = copy.deepcopy(self.q) # 保存上一时刻的关节角度     
                # self.q = self.InverseKinematics() # 正常运动的下一时刻角度值,在cloudtorepulsion中更新
                a = self.DeptoCloud()
                b = self.CloudtoRepulsion()           
                # 根据最近点求扭矩
                tau_obs = self.RepulsiontoTau(b[1])
                q_obs = self.ComputeAngleFromTau(tau_obs)
                self.q = copy.deepcopy(q_obs)

                # 使用Savitzky-Golay 滤波器后得到平滑图线
                m[:,j] = self.q
                if j == 49 or k == 1:
                    k = 1
                    j = 49
                    for jj in range(7):
                        m1[jj,:] = savgol_filter(m[jj,:], 11, 3, mode= 'nearest')
                    self.q = self.q * 0.2 + m1[:,-1] * 0.8
                    m2 = m1[:,1:]
                    m[:,0:49] = m2
                else: 
                    j += 1

                # 限速
                q_der = self.q - self.q0
                q_der = np.clip(q_der,-0.3,0.3)
                self.q = self.q0 + q_der
                # 关节角度的最后保障
                q1lowerlimit_if = self.q[:] > self.jointlowerlimit[:]
                q1upperlimit_if = self.q[:] < self.jointupperlimit[:]
                if False in q1lowerlimit_if:
                    aa = np.argwhere(q1lowerlimit_if == False).flatten()
                    for aaa in enumerate(aa):
                        self.q[aaa[1]] = self.jointlowerlimit[aaa[1]]
                if False in q1upperlimit_if:
                    bb = np.argwhere(q1upperlimit_if == False).flatten()
                    for bbb in enumerate(bb):
                        self.q[bbb[1]] = self.jointupperlimit[bbb[1]]
                
                # print("q_der: ", self.q-self.q0)
                # print("q: ", self.q)
                # self.q = np.array([0,0,0,0,0,0,0,0])

                # '-------定周期^^^发送控制命令-------'
                # # 计算运行时间
                # end_time = time.time()
                # der_time = end_time - begin_time
                # begin_time = time.time()
                # if der_time < 0.15: # 保持0.2s时间周期
                #     time.sleep(0.15 - der_time)
                # # task_command_str = 'servoJ(0,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)'%(self.q[0],self.q[1],self.q[2],self.q[3],self.q[4],self.q[5],self.q[6])
                # # command_send(task_command_str)
                # thread1 = threading.Thread(target=self.q_dif, args=(0.01, self.q0, self.q,))
                # thread1.start() 
                # thread1.join() 

                '---更新控制命令---'
                self.control_m0 = copy.deepcopy(self.control0)
                self.control_m1 = copy.deepcopy(self.control1)
                self.control0 = copy.deepcopy(self.q0[0:7])
                self.control1 = copy.deepcopy(self.q[0:7])
                self.con_update = True

                self.q_all_save1.append(self.q[0:7])
                clo_ex = copy.deepcopy(self.closedistance) # 必须要转存一下，不然全是最后一个数
                self.clo_dis_save.append(clo_ex)  

                self.viz.display(self.q)   # 仿真显示
                # print("q: ", self.q)
                self.DepthDisplay(b[0], a, b[2]) # 图像显示
                # time.sleep(0.1)
                # 结束
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.thread_state = False
                    break
            except Exception as e:
                print('输出异常: ', e)
                self.thread_state = False
                q_all_save0_np = np.array(self.q_all_save0)
                q_all_save1_np = np.array(self.q_all_save1)
                clo_dis_save_np = np.array(self.clo_dis_save)
                # np.save('q_all_save0(test2)', q_all_save0_np)
                # np.save('q_all_save1(test2)', q_all_save1_np)
                # np.save('clo_dis_save(test2)', clo_dis_save_np)
                sys.exit(1)

   
# 主函数
if __name__ == '__main__':
    rwo = RobWithCam()
    rwo.tf_calibration() # 获得机械臂和相机之间的转换关系
    np.set_printoptions(formatter={'float': '{: 0.3f}'.format}) # 只显示小数点后三位   
    viz = GepettoVisualizer(rwo.model, rwo.collision_model, rwo.visual_model)# 三维显示
    viz.initViewer(loadModel=True)
    viz.display(rwo.CyclicDisplay(viz))
