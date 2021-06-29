""" Example python module. """

from __future__ import print_function

import gtsam
import numpy as np
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D


from gtsam import (Cal3_S2, DoglegOptimizer,GenericProjectionFactorCal3_S2,
Marginals,NonlinearFactorGraph, PinholeCameraCal3_S2, Point3,Pose3,
PriorFactorPoint3, PriorFactorPose3, Rot3, Values, BetweenFactorPose3,Pose3AttitudeFactor,NoiseModelFactor,Point2,Rot3AttitudeFactor,Unit3)

import gtsam_absolute_factors

from gtsam.utils import plot


def rot3yaw(yaw):
    #input radians
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
    #pre-fixed rule
    R = R_yaw@R_pitch@R_roll
    R_wb = R
    return R_wb

def rotNed2Body(yaw,pitch,roll):
    R = rotBody2Ned(yaw,pitch,roll)
    return R.T

# pylint: disable = no-member
def create_rot3(rot_array,rot_type):        
        roll        = rot_array[0,0]
        pitch       = rot_array[0,1]
        yaw         = rot_array[0,2]
        if rot_type == 'incrementPose' or rot_type == 'Ned2Body':
            R = rotNed2Body(yaw,pitch,roll) #usikker på denne pga https://gtsam.org/2021/02/23/uncertainties-part2.html
        if rot_type == 'Body2Ned':
            R = rotBody2Ned(yaw,pitch,roll) #tror denne forholder seg til https://gtsam.org/2021/02/23/uncertainties-part2.html. Expressing everything in world frame
        #TODO: create other scenarios needed
        return Rot3(R)

def find_gravity_unit_NED_frame(rot_array):
    roll        = rot_array[0,0]
    pitch       = rot_array[0,1]
    yaw         = rot_array[0,2]    
    R_wb = rotBody2Ned(yaw,pitch,roll)
    x_b = np.array([0,0,1]).T
    x_w = R_wb@x_b
    print(x_w)
    x_w_unit = Unit3(x_w)
    return x_w_unit


def array2point3(x):
        return Point3(x[0,0],x[0,1],x[0,2])

def Compass_factor(key,yaw_meas):
    COMPASS_NOISE       = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e6, 1e6, np.deg2rad(1), 1e6, 1e6, 1e6]))

    rot3_orientation = create_rot3(np.array([[0,0,yaw_meas]]),'Body2Ned')
    t0 = np.array([[0,0,0]])
    priorMean = Pose3(rot3_orientation,array2point3(t0))

    compass_factor  =gtsam.PriorFactorPose3(key, priorMean, COMPASS_NOISE)

    return compass_factor

if __name__ == "__main__":
    ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
    PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
    #UNARAY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1,0.1]))
    USBL_NOISE          = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.15,0.15]))
    DEPTH_NOISE         = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01]))
    TILT_NOISE          = gtsam.noiseModel.Diagonal.Sigmas(np.array([np.deg2rad(1),np.deg2rad(1),np.deg2rad(1)]))
    #ATTITUDE_NOISE      = gtsam.noiseModel.Diagonal.Sigmas(np.array([np.deg2rad(1),np.deg2rad(1)]))
    ATTITUDE_NOISE      = gtsam.noiseModel.Isotropic.Sigma(2,np.deg2rad(5))


    ODOMETRY_NOISE      = gtsam.noiseModel.Diagonal.Sigmas(np.array([np.deg2rad(3), np.deg2rad(3), np.deg2rad(3), 0.3, 0.3, 1.0]))
    POSE_PRIOR_NOISE    = gtsam.noiseModel.Diagonal.Sigmas(np.array([np.deg2rad(1), np.deg2rad(1), np.deg2rad(1), 0.1, 0.1, 0.1]))


    graph = gtsam.NonlinearFactorGraph()

    # ' Creating Prior:
    rot3_orientation = create_rot3(np.array([[0,0,0]]),'Body2Ned')
    t0 = np.array([[0,0,0]])
    priorMean = Pose3(rot3_orientation,array2point3(t0))

    #priorMean = gtsam.Pose3(0.0, 0.0, 0.0, 0.0 ,0.0, 0.0)  # prior at origin
    graph.add(gtsam.PriorFactorPose3(1, priorMean, POSE_PRIOR_NOISE))
 

    odometry_1 = gtsam.Pose2(2.0, 0.0, 0.0)



    # ' Add Odometry measurements:
    angles_increment_1 = np.array([[0,0,np.deg2rad(30)]]) #pitch 30deg and yaw 30deg
    rotation_increment_1 = create_rot3(angles_increment_1,'Body2Ned') 
    translation_increment_1 = Point3(2,2,2)
    odometry_1 = Pose3(rotation_increment_1,translation_increment_1)
    graph.add(gtsam.BetweenFactorPose3(1,2,odometry_1,ODOMETRY_NOISE))

    angles_increment_2 = np.array([[np.deg2rad(30),np.deg2rad(30),np.deg2rad(60)]]) #pitch 30deg and yaw 30deg
    rotation_increment_2 = create_rot3(angles_increment_2,'Body2Ned') 
    translation_increment_2 = Point3(2,2,2)
    odometry_2 = Pose3(rotation_increment_2,translation_increment_2)
    graph.add(gtsam.BetweenFactorPose3(2,3,odometry_2,ODOMETRY_NOISE))

    #angles_increment_2 = np.array([[0,0,np.deg2rad(60)]]) #yaw 60deg
    #rotation_increment_2 = create_rot3(angles_increment_2,'incrementPose') 
    #translation_increment_2 = Point3(2,0,1)
    #odometry_2 = Pose3(rotation_increment_2,translation_increment_2)
    #graph.add(gtsam.BetweenFactorPose3(2,3,odometry_2,ODOMETRY_NOISE))


    #' Add the OTHER FACTORS:
    #' DEPTH FACTOR: OK
    Depth_factor_2 = gtsam_absolute_factors.DepthFactor(2,Point3(0,0,2),DEPTH_NOISE)
    Depth_factor_3 = gtsam_absolute_factors.DepthFactor(3,Point3(0,0,4),DEPTH_NOISE)
    graph.add(Depth_factor_2)
    graph.add(Depth_factor_3)

    #' USBL FACTOR: OK
    USBL_factor_2 = gtsam_absolute_factors.USBLFactor(2,Point2(2,2),USBL_NOISE)
    USBL_factor_3 = gtsam_absolute_factors.USBLFactor(3,Point2(4,4),USBL_NOISE)
    graph.add(USBL_factor_2)
    graph.add(USBL_factor_3)

    #' Tilt FACTOR: 
    #Tilt_factor_2 = gtsam_absolute_factors.TiltFactor(2,Point3(0,0,np.deg2rad(30)),TILT_NOISE)
    #Tilt_factor_3 = gtsam_absolute_factors.TiltFactor(3,Point3(0,0,np.deg2rad(80)),TILT_NOISE)
    #graph.add(Tilt_factor_2)
    #graph.add(Tilt_factor_3)

    
    #' Try to test out Pose3AttitudeFactor:
    attitude_2 = find_gravity_unit_NED_frame(angles_increment_1)    #outputs the gravity vector in body relative to NED-frame
    attitude_3 = find_gravity_unit_NED_frame(angles_increment_1)

    Tilt_factor_2 = Pose3AttitudeFactor(2, attitude_2, ATTITUDE_NOISE)
    Tilt_factor_3 = Pose3AttitudeFactor(3, attitude_3, ATTITUDE_NOISE)
    graph.add(Tilt_factor_2)
    graph.add(Tilt_factor_3)
    #print(Tilt_factor_2)
    print("\nFactor Graph:\n{}".format(graph))
    
    #' Trying out magnet
    compass_factor_2 = Compass_factor(2,np.deg2rad(30))
    compass_factor_3 = Compass_factor(3,np.deg2rad(90))

    graph.add(compass_factor_2)
    graph.add(compass_factor_3)



    #' Initial Estimate:
    initial = gtsam.Values()

    initial.insert(1, gtsam.Pose3(create_rot3(np.array([[0.1,0.1,0.2]]),'Body2Ned'), Point3(1.1, 1.0, 0.0)))
    initial.insert(2, gtsam.Pose3(create_rot3(np.array([[np.deg2rad(5),np.deg2rad(45),np.deg2rad(35)]]),'Body2Ned'), Point3(2.1, 1.0, 1.5)))
    initial.insert(3, gtsam.Pose3(create_rot3(np.array([[np.deg2rad(5),np.deg2rad(5),np.deg2rad(50)]]),'Body2Ned'), Point3(3.1, 1.0, 2.0)))
    
    #unaryFactor1 = gtsam_example3.GPSPose2Factor(1,Point2(0, 0),UNARAY_NOISE)
    #unaryFactor2 = gtsam_example3.GPSPose2Factor(2,Point2(2, 0),UNARAY_NOISE)
    #unaryFactor3 = gtsam_example3.GPSPose2Factor(3,Point2(4, 0),UNARAY_NOISE)
    #print(unaryFactor1)
    #graph.add(unaryFactor1)
    #graph.add(unaryFactor2)
    #graph.add(unaryFactor3)
    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()
    print("\nFinal Result:\n{}".format(result))

    marginals = gtsam.Marginals(graph, result)
    for i in range(1, 4):
        print("X{} covariance:\n{}\n".format(i, marginals.marginalCovariance(i)))

    print(type(marginals.marginalCovariance(1)))
    #print(type((result(1))))
    resultPoses = gtsam.utilities.allPose3s(result) 
    print(type(resultPoses.atPose3(1))) #returns pose3
    fig = plt.figure()
    axes = fig.add_subplot(111,projection='3d')
    axes.invert_zaxis() #Setting positive Z-axis down
    axes.invert_xaxis()

    for i in range(resultPoses.size()):
        plot.plot_pose3_on_axes(axes, resultPoses.atPose3(i+1),axis_length = 1)
        
        plot.plot_covariance_ellipse_3d(axes,resultPoses.atPose3(i+1).translation(),marginals.marginalCovariance(i+1)[3:6,3:6],scale=0.25) #axes, origin, covariance
    
    axes.set_xlim(-1, 10)
    axes.set_ylim(-1, 10)
    #create NORTH-EAST-DOWN format
    axes.set_xlabel('X - North')
    axes.set_ylabel('Y - East')
    axes.set_zlabel('Z - Down')


    plt.show()
    """
    graph.add(gtsam.BetweenFactorPose2(1, 2, odometry, ODOMETRY_NOISE))
    graph.add(gtsam.BetweenFactorPose2(2, 3, odometry, ODOMETRY_NOISE))
    initial = gtsam.Values()

    initial.insert(1, gtsam.Pose2(0.5, 0.0, 0.2))
    initial.insert(2, gtsam.Pose2(2.3, 0.1, -0.2))
    initial.insert(3, gtsam.Pose2(4.1, 0.1, 0.1))
    print("\nInitial Estimate:\n{}".format(initial))

    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()
    print("\nFinal Result:\n{}".format(result))

    marginals = gtsam.Marginals(graph, result)
    for i in range(1, 4):
        print("X{} covariance:\n{}\n".format(i, marginals.marginalCovariance(i)))

    fig = plt.figure(0)
    for i in range(1, 4):
        gtsam_plot.plot_pose2(0, result.atPose2(i), 0.5, marginals.marginalCovariance(i))
    plt.axis('equal')
    plt.show()
    
    print('hello')
    """
"""
    func_map = {
        'meaning_of_everything': meaning_of_everything,
        'create_special_2d_pose': create_special_2d_pose,
        'greet': greet
    }
    args = parse_arguments()
    func = func_map.get(args.command, meaning_of_everything)
    print(func())
"""
