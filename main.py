from __future__ import print_function

import time
import gtsam
import matplotlib.pyplot as plt
from gtsam import symbol_shorthand
L = symbol_shorthand.L
X = symbol_shorthand.X
 
from gtsam.examples import SFMdata
from gtsam import (Cal3_S2, DoglegOptimizer,GenericProjectionFactorCal3_S2,
Marginals,NonlinearFactorGraph, PinholeCameraCal3_S2, Point3,Pose3,
PriorFactorPoint3, PriorFactorPose3, Rot3, Values, BetweenFactorPose3)
from gtsam.utils import plot

import sys
sys.path.append("/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/code/Experiments/openCV2 Experiment")
#adding this path to be able to load pickles
sys.path.append('/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/code/prototyping isam/BOW_testing') 

import numpy as np
import cv2
from oct2py import octave
from oct2py import Oct2Py

import DataExtractionFunctions as DExF
import ImageProcessingFunctions as IP
import SLAMFunctions as sf
from Isam2Class import iSAM2FactorGraph

import gtsam_absolute_factors
import os

import pickle
from numpy.lib.npyio import load
from BOWandKeyFrameClass import BSTNode, binary_search_tree, BOW, KeyFrame,validate_loop_closure
import time
import datetime
from accessDataSwitch import access_data
from dvlClass import dvlPlaneEstimation
from deadReckoning import deadReckoning

oc = Oct2Py()
#Enabling CLAHS function from MATLAB implementation
oc.addpath('/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/code/Experiments/Image Processing Experiment/uwit')


#' Case where video is starting before telemetri:

#' CSV and video files:
Dataset_index = 4
TelemetriDataPath, FhSimPath, videoFile,videoStartTimeLocal ,videoLength = access_data(Dataset_index)
cap = cv2.VideoCapture(videoFile)

#' Video properties:
frameRate = 59.94
invFrameRate = 1/frameRate

#' Tuneable parameters
featureDetector = 'SIFT'
imgHeight = 1080
imgWidth = 1920
blurSize = 5
scale_percent = 100
timeTracker = 0                      #tracking nr seconds [s]
frameTracker = 0
nextMeasurementTime = 0
measurementIdx = 0
dt = 0.1
betweenPoseTracker = 1

#' KeyFrame Class parameters
KEYFRAME_SAMPLING_PERIODE = 1        #given in seconds
keyframe_list = []
BATCH_SIZE = int(10)
S_L_TH = 0.4
K = np.array([[1952.3, 0, 952.9353],
[0, 1948.4, 547.9819],
[0,0,1]])

#' Loop Closure parameters
YAW_TH = np.sin(np.deg2rad(7.5))     # Determines yaw threshold
DEPTH_TH = 0.8                       # Determines depth threshold
S_G_TH = 0.45
N_LINK_PROPOSALS = 3
NR_RECENT_IMAGES_TO_EXCLUDE_IN_LOOP_CLOSURE = 10   
HOMOGRAPHY_TH = 0.7                  #determinant threshold



#' Loading and creating offline BOW dictionary:
SearchTree = pickle.load(open("SearchTree.dat", "rb"))
RootNode = pickle.load(open("RootNode.dat", "rb"))
nr_of_words_in_vocabulary = pickle.load(open("nr_of_words_in_vocabulary.dat", "rb"))

print('Tree depth',SearchTree.getTreeDepth())

BagOfWords = BOW(RootNode,SearchTree.getTreeDepth(),nr_of_words_in_vocabulary,N_LINK_PROPOSALS,NR_RECENT_IMAGES_TO_EXCLUDE_IN_LOOP_CLOSURE)

# ' Extract Raw Data:
fhSimData = DExF.extractFhSimData(FhSimPath)
telemetriData = DExF.extractTelemetriData(TelemetriDataPath)

#' Extracting Data:
clockStartTime, clockEndTime, nrCentiSecondVideoRoundedUp, TELEMETRI_STARTS_RECORDING_LAST = DExF.extractClockTime(fhSimData,telemetriData,videoStartTimeLocal,videoLength)
Time, RPM, EtaMeasured4, NuMeasured4, USBLAvail, DVLAvail, EtaHat, NuHat, BiasHat, DVL_beams, DVL_beams_flags, RPM2Tau, CmdTau, NetFollowingMode, EtaMeasured6, NuMeasured6,AbsoluteMeasurements,USBL_raw = DExF.extractCSVinfo(telemetriData,clockStartTime,clockEndTime)


#' Find start frame of video:
startFrame = DExF.findStartFrameOfVideo(fhSimData,clockStartTime,videoStartTimeLocal,nrCentiSecondVideoRoundedUp,TELEMETRI_STARTS_RECORDING_LAST,frameRate)
if startFrame>0:
    #the video capture operators from cv2 is 0 indexed
    startFrame = startFrame-1 
cap.set(1,startFrame)

#' Create DVL class:
dvl_class = dvlPlaneEstimation()
dist, yaw_desired = None, None


# ' Create Isam2Class and Initialize:
isam2object = iSAM2FactorGraph()

dvl_speed, gyroscope, d_meas, ori_meas, dvl_beams, dvl_beams_flag, kf_4DOF_estimate, usbl_meas, kf_4DOF_speed_estimate = isam2object.format_measurements(NuMeasured4[measurementIdx,:], AbsoluteMeasurements[measurementIdx,:], DVL_beams[measurementIdx,:], DVL_beams_flags[measurementIdx,:],EtaHat[measurementIdx,:],USBL_raw[measurementIdx,:],NuHat[measurementIdx,:])
isam2object.initialize_graph(ori_meas,kf_4DOF_estimate)

# ' Initialize Deadreckoning:
DR = deadReckoning(kf_4DOF_estimate)



depth_array = np.array([[]])            #stores depth measurements from keyframes
yaw_array = np.array([[]])              #stores yaw measurements from keyframes
node_index = 0
first_measurementIdx = measurementIdx
while(True):
    frameTracker += 1
    timeTracker = (frameTracker-1)/frameRate
    newMeas = (abs(timeTracker - nextMeasurementTime)< invFrameRate)
    
    #' Setting initial frame
    if frameTracker == 1:
        ret, frame_prev = cap.read()
        frame_prev = KeyFrame.process_image(frame_prev,blurSize)
        kp_prev, des_prev = IP.getKeyPointsAndFeatures(frame_prev,featureDetector)
    else:
        ret, frame_cur = cap.read()

    #' Gathering Data
    if (newMeas == True):
        nextMeasurementTime += 0.1
        measurementIdx += 1 

        dvl_speed, gyroscope, d_meas, ori_meas, dvl_beams, dvl_beams_flag, kf_4DOF_estimate, usbl_meas,kf_4DOF_speed_estimate = isam2object.format_measurements(NuMeasured4[measurementIdx,:], AbsoluteMeasurements[measurementIdx,:], DVL_beams[measurementIdx,:], DVL_beams_flags[measurementIdx,:],EtaHat[measurementIdx,:],USBL_raw[measurementIdx,:],NuHat[measurementIdx,:])
        
        #'Estimating distance from net and desired heading:
        camera_distance_from_net, yaw_d, n_d_normalized = dvl_class.estimate_distance_camera_to_net_and_yaw(dvl_beams,dvl_beams_flag,ori_meas[0,2])
        if camera_distance_from_net is not None:
            #update depth estimate
            dist, yaw_desired, n_dvl = camera_distance_from_net, yaw_d, n_d_normalized

        #'add initial estimate to pose graph optimization:
        isam2object.increment_pose_tracker()
        isam2object.insert_initial_pose_estimates(kf_4DOF_estimate,ori_meas)
        
        
        #'Adding Factors Graph:
        if DVLAvail[0,measurementIdx]==1:
            isam2object.add_odometry(dvl_speed,gyroscope,dt)
            DR.increment_DR(dvl_speed,gyroscope,dt)
        else:
            vel_kf_estimate = np.array([kf_4DOF_speed_estimate[0,0:3]])
            angular_vel_kf_estimate = np.array([[0,0,kf_4DOF_speed_estimate[0,3].item()]]) 
            isam2object.add_odometry(vel_kf_estimate,angular_vel_kf_estimate,dt)
            DR.increment_DR(vel_kf_estimate,angular_vel_kf_estimate,dt)

        isam2object.add_depth(d_meas)
        isam2object.add_usbl(usbl_meas, USBLAvail[0,measurementIdx],ori_meas) 
        isam2object.add_orientation(ori_meas)
        

        #'Adding KeyFrame - Based on consecutive matching
        node_index = isam2object.get_pose_tracker()
        
        #'Plot Results:
        
        if node_index%500 ==0:
            DR_pose_array = DR.get_pose_array()
            results = isam2object.optimize_graph()
            isam2object.plot_comparison(results,Time[0,first_measurementIdx:(measurementIdx+1)],EtaMeasured4[first_measurementIdx:(measurementIdx+1)],EtaHat[first_measurementIdx:(measurementIdx+1)],DR_pose_array)
            isam2object.plot_error(results,Time[0,first_measurementIdx:(measurementIdx+1)],EtaMeasured4[first_measurementIdx:(measurementIdx+1)],EtaHat[first_measurementIdx:(measurementIdx+1)],DR_pose_array)


        #'only sample a keyframe for every second
        if node_index != 0 and (node_index %(KEYFRAME_SAMPLING_PERIODE*10)==0): 
            time_captured = str(datetime.timedelta(seconds=(timeTracker+startFrame*invFrameRate)))
            X_prev_id = node_index-int(KEYFRAME_SAMPLING_PERIODE*10) 
            X_cur_id = node_index

            #'Image Capture from Video 
            keyframe_list.append(KeyFrame(X_prev_id, X_cur_id, frame_cur, blurSize, featureDetector, frame_prev, kp_prev, des_prev,time_captured,ori_meas))

            cur_good_kps, cur_good_des = keyframe_list[-1].get_good_features_in_cur_image(0)
            image_histogram, H, S_L, S_G, TF_IDF_histogram, new_S_L, new_S_G, new_TF_IDF_histogram = BagOfWords.main_function(cur_good_des, keyframe_list, BATCH_SIZE) 
            
            if not new_S_L:
                print('no updates')
            else:
                for i in range (0,len(new_S_L)):
                    keyframe_list[i].update_global_and_local_saliency_and_TF_IDF_histogram(new_S_L[i],new_S_G[i],new_TF_IDF_histogram[i])
                    frame_histogram_update ,S_L__update, S_G_update, TF_IDF_histogram_update = keyframe_list[i].get_BOW_data()
                    BagOfWords.update_TF_IDF_histograms(i,TF_IDF_histogram_update)
                
            keyframe_list[-1].add_BOW_data(image_histogram,TF_IDF_histogram,S_L,S_G) 
            BagOfWords.add_TF_IDF_histogram(TF_IDF_histogram)

            frame_prev = keyframe_list[-1].get_image()
            kp_prev, des_prev = keyframe_list[-1].get_kps_and_des()
            
            #' Don't want to match this image with previous frame
            if S_L< S_L_TH:
                #removing the last keyframe if not enough local saliency
                keyframe_list.pop() 
                BagOfWords.remove_TF_IDF_histogram()
            else:
                depth_array = np.concatenate((depth_array, np.array([[d_meas]])), axis=1) 
                yaw_array   = np.concatenate((yaw_array, np.array([[ori_meas[0,2]]])), axis=1)
                #' Want to match with previous frame and do loop closure
                if (S_L>= S_L_TH) and (S_G >=S_G_TH):
                    n_link_hypothesis = BagOfWords.loop_closure_link_hypothesis(DEPTH_TH,d_meas,depth_array,YAW_TH,ori_meas[0,2],yaw_array,TF_IDF_histogram)
                    print('n_link_hypothesis',n_link_hypothesis)
                    if len(n_link_hypothesis)!=0:
                        if (n_link_hypothesis[0][1]>0.45):
                            prev_image_idx = n_link_hypothesis[0][0]
                            S_L_proposal, S_G_proposal = keyframe_list[prev_image_idx].get_local_saliency_and_global_saliency()
                          
                            ImageMatches, R, T, loop_closure_found= validate_loop_closure(keyframe_list[-1],keyframe_list[prev_image_idx],K,'SIFT',HOMOGRAPHY_TH, n_dvl)  
                            if loop_closure_found==True:
                                print('Loop Closure Detected')

                                #plot before adding loop closure
                                DR_pose_array = DR.get_pose_array()

                                isam2object.add_loop_closure(keyframe_list[prev_image_idx].get_keyframe_id(),keyframe_list[-1].get_keyframe_id(),R,T,dist)

                                #plot after adding loop-closure
                                results = isam2object.optimize_graph()
                                #isam2object.plot_comparison(results,Time[0,first_measurementIdx:(measurementIdx+1)],EtaMeasured4[first_measurementIdx:(measurementIdx+1)],EtaHat[first_measurementIdx:(measurementIdx+1)],DR_pose_array)
                                #isam2object.plot_result(results)
                                
                                #cv2.imshow('ImageMatches',ImageMatches)
                                #waitKey = cv2.waitKey(15000)

                #' Image should only be matched with previous frame
                else:
                    _ = BagOfWords.increment_cost_matrix(TF_IDF_histogram)
                    print('S_G not above threshold', S_G)
                


    





