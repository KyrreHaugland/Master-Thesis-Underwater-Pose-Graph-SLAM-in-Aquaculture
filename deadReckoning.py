import numpy as np
import SLAMFunctions as sf
import math

class deadReckoning():

    def __init__(self,start_position_array = np.array([0,0,0,0])):
            self.pose =  start_position_array #1x4 array
            self.pose_array = start_position_array
        
        
    
    def increment_DR(self,dvl_speed,gyroscope,dt):
        u = np.concatenate((dvl_speed,np.array([[gyroscope[0,2]]])),axis=1)#stack dvl_speed and gyroscope


        current_pose = self.get_pose()
        R = sf.rotBody2Ned(current_pose[0,3], 0, 0)
        T = np.concatenate((R,np.array([[0,0,0]]).T),axis=1)
        T = np.concatenate((T,np.array([[0,0,0,1]])),axis=0)

        new_pose = ((T@u.T)*dt + current_pose.T)

        new_pose[3,0] = self.pi_2_pi(new_pose[3,0])
        self.update_pose_and_poseList(new_pose.T)
        return new_pose.T
    
    def update_pose_and_poseList(self,new_pose):
        self.pose = new_pose
        self.pose_array = np.concatenate((self.pose_array,new_pose),axis=0)
    
    def get_pose(self):
        return self.pose
    
    def get_pose_array(self):
        return self.pose_array

    @staticmethod
    def pi_2_pi(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi