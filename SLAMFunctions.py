import numpy as np
from gtsam import (Cal3_S2, DoglegOptimizer,GenericProjectionFactorCal3_S2,
Marginals,NonlinearFactorGraph, PinholeCameraCal3_S2, Point3,Pose3,
PriorFactorPoint3, PriorFactorPose3, Rot3, Values)


def createTransformationMatrix(R,t):
    T = np.concatenate((R, t), axis=1)
    return T

#' Rotation Functions
def generateRotationMatrix(roll,pitch,yaw,label):
    R_yaw = rot3yaw(yaw)
    R_pitch = rot3pitch(pitch)
    R_roll = rot3roll(roll)
    #going up in hierarchy: e.g from body to world R^w_b
    R = R_yaw @ R_pitch @ R_roll
    if label == 'up':
        R = R
    
    #goes down in hierarchy: e.g from world to body R^b_w
    else:
        R = R.T
    return R

def rot3yaw(yaw):
    R = np.zeros((3,3))
    R[0:2,0:2] = np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    R[2,2] = 1
    return R

def rot3pitch(pitch):
    R = np.zeros((3,3))
    R[0,0] = np.cos(pitch)
    R[0,2] = np.sin(pitch)
    R[2,0] = -np.sin(pitch)
    R[2,2] = np.cos(pitch)
    R[1,1] = 1
    return R

def rot3roll(roll):
    R = np.zeros((3,3))
    R[1:3,1:3] = np.array([[np.cos(roll),-np.sin(roll)],[np.sin(roll),np.cos(roll)]])
    R[0,0] = 1
    return R

def rotBody2Ned(yaw,pitch,roll):
    R_yaw = rot3yaw(yaw)
    R_pitch = rot3pitch(pitch)
    R_roll = rot3roll(roll)
    R = R_yaw@R_pitch@R_roll
    R_wb = R
    return R_wb

def rotNed2Body(yaw,pitch,roll):
    R = rotBody2Ned(yaw,pitch,roll)
    return R.T


def TransformWorld2Body(pose):
    Rb_w = RotMat2D(pi_2_pi(pose[2,0])).T
    tw_b = pose[0:2,0]
    tb_w = -Rb_w@tw_b
    Tb_w = np.zeros((3,3))
    Tb_w[0:2,0:2] = Rb_w 
    Tb_w[0:2,2] = tb_w
    Tb_w[2,2] = 1
    return Tb_w


def TransformBody2World(pose):
    Rw_b = RotMat2D(pi_2_pi(pose[2,0]))
    tw_b = pose[0:2,0]
    Tw_b = np.zeros((3,3))
    Tw_b[0:2,0:2] = Rw_b 
    Tw_b[0:2,2] = tw_b
    Tw_b[2,2] = 1
    return Tw_b
    

def TransformBody2DVL():  
    Rdvl_b = RotMat2D(0).T
    tb_dvl = np.array([0.37, -0.2]).T    #DVLPosWrtCO = "0.37,-0.2,0.2"
    tdvl_b = -Rdvl_b@tb_dvl
    Tdvl_b = np.zeros((3,3))
    Tdvl_b[0:2,0:2] = Rdvl_b 
    Tdvl_b[0:2,2] = tdvl_b
    Tdvl_b[2,2] = 1
    return Tdvl_b


def TransformDVL2Body(): 
    Rb_dvl = RotMat2D(0)
    tb_dvl = np.array([0.37, -0.2]).T     #DVLPosWrtCO = "0.37,-0.2,0.2"
    Tb_dvl = np.zeros((3,3))
    Tb_dvl[0:2,0:2] = Rb_dvl
    Tb_dvl[0:2,2] = tb_dvl
    Tb_dvl[2,2] = 1
    return Tb_dvl