import gtsam
import numpy as np
import matplotlib.pyplot as plt
from gtsam import symbol_shorthand
L = symbol_shorthand.L
X = symbol_shorthand.X

#from gtsam.examples import SFMdata
from gtsam import (Cal3_S2, DoglegOptimizer,GenericProjectionFactorCal3_S2,
Marginals,NonlinearFactorGraph, PinholeCameraCal3_S2, Point2, Point3,Pose3,
PriorFactorPoint3, PriorFactorPose3, Rot3, Values, BetweenFactorPose3,Pose3AttitudeFactor,
NoiseModelFactor, Unit3, EssentialMatrixFactor)
from gtsam.utils import plot

import SLAMFunctions as sf
import gtsam_absolute_factors
import gtsam.utils.plot as gtsam_plot
from mpl_toolkits.mplot3d import Axes3D



class iSAM2FactorGraph:
    #'Intrinsic Camera Matrix & FOV
    K                   = Cal3_S2(1952.3, 1948.4, 0.0, 952.9353, 547.9819)
    FOV                 = np.deg2rad(67.0) 

    #'DVL properties:
    betaAngles          = np.array([45,135,225,315])
    gammaAngle          = np.deg2rad(25)

    #'Lists & Trackers
    pose_variables      = []
    landmark_variables  = []
    pose_tracker        = 0

    #'Noise Variables
    CAMERA_NOISE        = gtsam.noiseModel.Isotropic.Sigma(2, 1.0) 
    ODOMETRY_NOISE      = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e6, 1e6, np.deg2rad(1), 0.01, 0.01, 0.02]))
    DEPTH_NOISE         = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.02]))
    COMPASS_NOISE       = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e6, 1e6, np.deg2rad(2), 1e6, 1e6, 1e6])) 
    POSE_PRIOR_NOISE    = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.3, 0.1, 0.1, 0.1])) 
    USBL_NOISE          = gtsam.noiseModel.Diagonal.Sigmas(np.array([1.3,1.3])) 
    ATTITUDE_NOISE      = gtsam.noiseModel.Isotropic.Sigma(2,np.deg2rad(5)) #pitch and roll noise

    DVL_DISTANCE_NOISE  = 0.06
    LOOP_CLOSURE_ROTATION_NOISE = [np.deg2rad(5),np.deg2rad(5),np.deg2rad(7.5)]



    #' Displacements with Respect to CO/CG
    CO2DVL              = Point3(0.37, -0.2, 0.2)             
    CO2TRN              = np.array([[0.45, 0.35, -0.27]])        
    DVL2Camera          = Point3(0.08, 0.17, 0.2)      
    CO2IMU              = Point3(0, 0, 0)

    def __init__(self):
        self.isam_params = gtsam.ISAM2Params()
        self.isam = gtsam.ISAM2(self.isam_params)
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values() 

    def initialize_graph(self, ori_meas,start_position_array = np.array([0,0,0])):
        # Create a key for the first pose.
        X0 = X(self.pose_tracker)

        rot3_orientation = self.create_rot3(ori_meas,'Body2Ned')
        # Update the list with the new pose variable key.
        self.pose_variables.append(X0)
        
        init_pose = Pose3(rot3_orientation,self.array2point3(start_position_array))

        factor = PriorFactorPose3(X0, init_pose, self.POSE_PRIOR_NOISE)

        # Add a prior on pose X0 at the origin.
        self.graph.add(factor)
        
        # Set an initial estimate for the first pose.
        self.initial_estimate.insert(X0, init_pose)
        
    
    def create_compass_factor(self,yaw_meas):
        X1 = X(self.pose_tracker)

        rot3_orientation = self.create_rot3(np.array([[0,0,yaw_meas]]),'Body2Ned')
        t0 = np.array([[0,0,0]])
        compass_orientation = Pose3(rot3_orientation,self.array2point3(t0))

        compass_factor = gtsam.PriorFactorPose3(X1, compass_orientation, self.COMPASS_NOISE)

        self.graph.add(compass_factor)

        return compass_factor        

    def add_odometry(self,dvl_speed,gyroscope,dt):
        #NOTE: this should only be added if flag is set to 1
        angles_increment = gyroscope*dt
        dvl_increment = dvl_speed*dt

        rotation_increment = self.create_rot3(angles_increment,'Body_2_2Body_1') 
        translation_increment = Point3(dvl_increment[0,0],dvl_increment[0,1],dvl_increment[0,2])

        odometry = Pose3(rotation_increment,translation_increment)
        
        X0 = X(self.pose_tracker-1)
        X1 = X(self.pose_tracker)
        self.graph.add(gtsam.BetweenFactorPose3(X0,X1,odometry,self.ODOMETRY_NOISE))
        
        self.pose_variables.append(X1) 

    def add_orientation(self,ori_meas):
        X1 = X(self.pose_tracker)

        attitude = self.find_vehicle_down_vector_in_NED_frame(ori_meas)
        tilt_factor = Pose3AttitudeFactor(X1, attitude, self.ATTITUDE_NOISE)   

        yaw_meas = ori_meas[0,2]
        compass_factor = self.create_compass_factor(yaw_meas)

        self.graph.add(tilt_factor)
        self.graph.add(compass_factor)

    def add_usbl(self,usbl_meas,usbl_avail,ori_meas):
        
        if (usbl_avail==True):
            X1 = X(self.pose_tracker)
            yaw_meas = ori_meas[0,2]
            R_wb = sf.rotBody2Ned(yaw_meas,0,0)

            world_to_transponder = R_wb@self.CO2TRN.T

            #convert the measurement to body_frame:
            usbl_factor = gtsam_absolute_factors.USBLFactor(X1,self.array2point2(usbl_meas-world_to_transponder),self.USBL_NOISE)
            self.graph.add(usbl_factor)

    def add_depth(self,d_meas):
        X1 = X(self.pose_tracker)

        depth_factor = gtsam_absolute_factors.DepthFactor(X1,Point3(0,0,d_meas),self.DEPTH_NOISE)

        self.graph.add(depth_factor)

    def increment_pose_tracker(self):
        self.pose_tracker += 1
    
    def get_pose_tracker(self):
        return self.pose_tracker

    def insert_initial_pose_estimates(self,KF_input,ori_meas):
        X1 = X(self.pose_tracker)

        rot3_orientation = self.create_rot3(ori_meas,'Body2Ned')
        ned_position = self.array2point3(KF_input)

        self.initial_estimate.insert(X1,Pose3(rot3_orientation,ned_position))

    def add_loop_closure(self,prev_keyframe_id,cur_keyframe_id,prev_rotation_cur,prev_translation_cur,dvl_distance_estimate_cur):
        #pose_prev is the pose_tracker of when the image was taken
        #pose_cur is the pose_tracker of the current image

        X0 = X(prev_keyframe_id)
        X1 = X(cur_keyframe_id)
        
        relative_rotation = Rot3(prev_rotation_cur.T)
        relative_translation_normalized = -prev_rotation_cur.T@prev_translation_cur
        relative_translation = relative_translation_normalized*dvl_distance_estimate_cur
        
        #estimate the uncertainty propagation based on depth uncertainty:
        translation_uncertainty = (relative_translation_normalized**2)*self.DVL_DISTANCE_NOISE
        uncertainty_list_copy = self.get_LOOP_CLOSURE_ROTATION_NOISE()
        uncertainty_list = list(uncertainty_list_copy)
        uncertainty_list.extend(translation_uncertainty[:,0].flatten().tolist())
        uncertainty_array = np.array(uncertainty_list)
        uncertainty_list.pop() #removing the 3 last elements

        rotation_increment = relative_rotation 
        translation_increment = self.array2point3(relative_translation.T)

        transformation = Pose3(rotation_increment,translation_increment)
        tf_uncertainty = gtsam.noiseModel.Diagonal.Sigmas(uncertainty_array)
        
        self.graph.add(gtsam.BetweenFactorPose3(X0,X1,transformation,tf_uncertainty))

    def get_LOOP_CLOSURE_ROTATION_NOISE(self):
        return self.LOOP_CLOSURE_ROTATION_NOISE


    def optimize_graph(self):
        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimate, params)
        result = optimizer.optimize()
        return result

    def plot_result(self,result):
        marginals = gtsam.Marginals(self.graph, result)
        resultPoses = gtsam.utilities.allPose3s(result) 

        fig = plt.figure(1)
        axes = fig.add_subplot(111,projection='3d')
        axes.invert_zaxis() 
        axes.invert_xaxis()

        for i in range(resultPoses.size()):
            plot.plot_pose3_on_axes(axes, resultPoses.atPose3(X(i)),axis_length = 1)
            plot.plot_covariance_ellipse_3d(axes,resultPoses.atPose3(X(i)).translation(),marginals.marginalCovariance(X(i))[3:6,3:6],scale=0.25) #axes, origin, covariance
        
        axes.set_xlabel('X - North')
        axes.set_ylabel('Y - East')
        axes.set_zlabel('Z - Down')

        #'figure 2
        fig2 = plt.figure(2)
        axes1 = fig2.add_subplot(111,projection='3d')
        axes1.invert_zaxis() 
        axes1.invert_xaxis()

        plot.plot_trajectory(2, result, scale=0.5, marginals=marginals,title="Trajectory")
        axes1.set_xlabel('X - North')
        axes1.set_ylabel('Y - East')
        axes1.set_zlabel('Z - Down')


        plt.show()
    
    def generatePoseGraphResults(self,result):
        marginals = gtsam.Marginals(self.graph, result)
        resultPoses = gtsam.utilities.allPose3s(result) 

        for i in range(resultPoses.size()):
            #decompose
            pose_i = resultPoses.atPose3(X(i))
            R = pose_i.rotation()
            roll = R.roll()
            pitch = R.pitch()
            yaw = R.yaw()
            result_i = np.array([[roll,pitch,yaw,pose_i.x(),pose_i.y(),pose_i.z()]])
            if i ==0:
                resultArray =result_i
            else:
                resultArray = np.concatenate((resultArray,result_i),axis=0)



        return resultArray

    def plot_error(self,result,Time,EtaMeasured,EtaHat,DR_pose_list):
        poseGraphResults = self.generatePoseGraphResults(result)
        #error wrt EKF
        fig, [[ax1, ax2], [ax3, ax4]] = plt.subplots(nrows=2, ncols=2)

        ax1.plot(Time.T,np.abs(EtaHat[:,0]-EtaMeasured[:,0]),"r",label = '$e(x_{meas})$')
        ax1.plot(Time.T,np.abs(EtaHat[:,0]-poseGraphResults[:,3]),"g",label = '$e(\hat{x}_{pg}})$')
        ax1.plot(Time.T,np.abs(EtaHat[:,0]-DR_pose_list[:,0]),"k",label = '$e(x_{dr})$')
        ax1.legend(shadow=True, fancybox=True)
        ax1.set_xlim(0,)
        ax1.set_xlabel('Time')
        ax1.grid(True)

        ax2.plot(Time.T,np.abs(EtaHat[:,1]-EtaMeasured[:,1]),"r",label = '$e(y_{meas})$')
        ax2.plot(Time.T,np.abs(EtaHat[:,1]-poseGraphResults[:,4]),"g",label = '$e(\hat{y}_{pg}})$')
        ax2.plot(Time.T,np.abs(EtaHat[:,1]-DR_pose_list[:,1]),"k",label = '$e(y_{dr})$')
        ax2.legend(shadow=True, fancybox=True)
        ax2.set_xlim(0,)
        ax2.set_xlabel('Time')
        ax2.grid(True)

        ax3.plot(Time.T,np.abs(EtaHat[:,2]-EtaMeasured[:,2]),"r",label = '$e(z_{meas})$')
        ax3.plot(Time.T,np.abs(EtaHat[:,2]-poseGraphResults[:,5]),"g",label = '$e(\hat{z}_{pg}})$')
        ax3.plot(Time.T,np.abs(EtaHat[:,2]-DR_pose_list[:,2]),"k",label = '$e(z_{dr})$')
        ax3.legend(shadow=True, fancybox=True)
        ax3.set_xlim(0,)
        ax3.set_xlabel('Time')
        ax3.grid(True)

        ax4.plot(Time.T,np.abs(EtaHat[:,3]-EtaMeasured[:,3]),"r",label = '$e(\psi_{meas})$')
        ax4.plot(Time.T,np.abs(EtaHat[:,3]-poseGraphResults[:,2]),"g",label = '$e(\hat{\psi}_{pg})$')
        ax4.plot(Time.T,np.abs(EtaHat[:,3]-DR_pose_list[:,3]),"k",label = '$e(\psi_{dr})$')
        ax4.legend(shadow=True, fancybox=True)
        ax4.set_xlim(0,)
        ax4.set_xlabel('Time')
        ax4.grid(True)

        fig.suptitle('Position Error', fontsize=16)

        fig.tight_layout()
        plt.show()

    def plot_comparison(self,result,Time,EtaMeasured,EtaHat,DR_pose_list):
        poseGraphResults = self.generatePoseGraphResults(result)
        print(DR_pose_list.shape)

        fig, [[ax1, ax2], [ax3, ax4]] = plt.subplots(nrows=2, ncols=2)

        ax1.plot(Time.T,EtaMeasured[:,0],"r",label = '$x_{meas}$')
        ax1.plot(Time.T,EtaHat[:,0],"b",label = '$\hat{x}$')
        ax1.plot(Time.T,poseGraphResults[:,3],"g",label = '$\hat{x}_{pg}}$')
        ax1.plot(Time.T,DR_pose_list[:,0],"k",label = '$x_{dr}$')
        ax1.legend(shadow=True, fancybox=True)
        ax1.set_xlim(0,)
        ax1.set_xlabel('Time')
        ax1.grid(True)

        ax2.plot(Time.T,EtaMeasured[:,1],"r",label = '$y_{meas}$')
        ax2.plot(Time.T,EtaHat[:,1],"b",label = '$\hat{y}$')
        ax2.plot(Time.T,poseGraphResults[:,4],"g",label = '$\hat{y}_{pg}}$')
        ax2.plot(Time.T,DR_pose_list[:,1],"k",label = '$y_{dr}$')
        ax2.legend(shadow=True, fancybox=True)
        ax2.set_xlim(0,)
        ax2.set_xlabel('Time')
        ax2.grid(True)

        ax3.plot(Time.T,EtaMeasured[:,2],"r",label = '$z_{meas}$')
        ax3.plot(Time.T,EtaHat[:,2],"b",label = '$\hat{z}$')
        ax3.plot(Time.T,poseGraphResults[:,5],"g",label = '$\hat{z}_{pg}}$')
        ax3.plot(Time.T,DR_pose_list[:,2],"k",label = '$z_{dr}$')
        ax3.legend(shadow=True, fancybox=True)
        ax3.set_xlim(0,)
        ax3.set_xlabel('Time')
        ax3.grid(True)

        ax4.plot(Time.T,EtaMeasured[:,3],"r",label = '$\psi_{meas}$')
        ax4.plot(Time.T,EtaHat[:,3],"b",label = '$\hat{\psi}$')
        ax4.plot(Time.T,poseGraphResults[:,2],"g",label = '$\hat{\psi}_{pg}$')
        ax4.plot(Time.T,DR_pose_list[:,3],"k",label = '$\psi_{dr}$')
        ax4.legend(shadow=True, fancybox=True)
        ax4.set_xlim(0,)
        ax4.set_xlabel('Time')
        ax4.grid(True)

        fig.suptitle('Position', fontsize=16)


        fig.tight_layout()
        plt.show()


    @staticmethod
    def array2point3(x):
        return Point3(x[0,0],x[0,1],x[0,2])
    
    @staticmethod
    def array2point2(x):
        return Point2(x[0,0],x[0,1])

    @staticmethod
    def create_pose3(rot3,t):
        pose = Pose3(rot3,t)

        return pose
    
    @staticmethod
    def create_rot3(rot_array,rot_type):        
        roll        = rot_array[0,0]
        pitch       = rot_array[0,1]
        yaw         = rot_array[0,2]
        if rot_type == rot_type == 'Ned2Body':
            R = sf.rotNed2Body(yaw,pitch,roll) 
        if rot_type == 'Body2Ned' or 'incrementPose' or 'Body_2_2Body_1':
            R = sf.rotBody2Ned(yaw,pitch,roll) 
        return Rot3(R)
    
    @staticmethod
    def find_vehicle_down_vector_in_NED_frame(rot_array):
        roll        = rot_array[0,0]
        pitch       = rot_array[0,1]
        yaw         = rot_array[0,2]    

        R_wb        = sf.rotBody2Ned(yaw,pitch,roll)
        x_b         = np.array([0,0,1]).T
        x_w         = R_wb@x_b
        x_w_unit    = Unit3(x_w)
        return x_w_unit
    
    @staticmethod
    def format_measurements(vel_4DOF_raw, abs_meas_raw, dvl_beams_raw, dvl_beams_flags_raw,kf_4DOF_pose_estimate_raw,usbl_meas_raw, kf_4DOF_speed_estimate_raw):
        vel_4DOF                    = np.array([vel_4DOF_raw])
        dvl_speed                   = np.array([vel_4DOF[0,0:3]])                # [u,v,w]       - 1x3 array
        gyroscope                   = np.array([[0,0,vel_4DOF[0,3].item()]])     # [p,q,r]       - 1x3 array
        abs_meas                    = np.array([abs_meas_raw])
        d_meas                      = abs_meas[0,0]                              # [depth]       - scalar
        ori_meas                    = np.array([abs_meas[0,1:4]])                # [Φ,ϴ,Ψ]       - 1x3 array

        dvl_beams                   = np.array([dvl_beams_raw])                  # [b1,b2,b3,b4] - 1x4 array
        dvl_beams_flag              = np.array([dvl_beams_flags_raw])            # [f1,f2,f3,f4] - 1x4 array
        
        kf_4DOF_estimate            = np.array([kf_4DOF_pose_estimate_raw])      # [x,y,z,Ψ]     - 1x4 array
        kf_4DOF_speed_estimate      = np.array([kf_4DOF_speed_estimate_raw])     # [u,v,w,r]     - 1x4 array

        usbl_meas                   = np.array([usbl_meas_raw])                  # [x,y,z]       - 1x4 array
        return dvl_speed, gyroscope, d_meas, ori_meas, dvl_beams, dvl_beams_flag, kf_4DOF_estimate, usbl_meas,kf_4DOF_speed_estimate 
    
