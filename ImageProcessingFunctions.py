import cv2
import numpy as np
import os
from matplotlib import pyplot as plt



#This function is taken from https://www.programcreek.com/python/?code=nansencenter%2Fsea_ice_drift%2Fsea_ice_drift-master%2FLICENSE#
def find_orb_key_points(image,
                    edgeThreshold=10,
                    nFeatures=100000,
                    nLevels=7,
                    patchSize=32,
                    **kwargs):

    if cv2.__version__.startswith('3.') or cv2.__version__.startswith('4.'):
        detector = cv2.ORB_create()
        detector.setEdgeThreshold(edgeThreshold)
        detector.setMaxFeatures(nFeatures)
        detector.setNLevels(nLevels)
        detector.setPatchSize(patchSize)
    else:
        detector = cv2.ORB()
        detector.setInt('edgeThreshold', edgeThreshold)
        detector.setInt('nFeatures', nFeatures)
        detector.setInt('nLevels', nLevels)
        detector.setInt('patchSize', patchSize)

    keyPoints, descriptors = detector.detectAndCompute(image, None)

    return keyPoints, descriptors 

def find_sift_key_points(image,nOctaveLayers1=4,contrastThreshold1=0.065,edgeThreshold1=20,sigma1=1.6,**kwargs):    
    sift = cv2.xfeatures2d.SIFT_create(nfeatures = 2000,nOctaveLayers = nOctaveLayers1,contrastThreshold = contrastThreshold1,edgeThreshold = edgeThreshold1,sigma=sigma1)
    kp, des = sift.detectAndCompute(image,None)

    return kp, des

def find_akaze_key_points(image):
    akaze = cv2.AKAZE_create()
    kp, des = akaze.detectAndCompute(image, None)

    return kp, des


def getKeyPointsAndFeatures(image,featureDetector):
    if featureDetector == 'SIFT':
        keyPoints, descriptors = find_sift_key_points(image)
    elif featureDetector == 'ORB':
        keyPoints, descriptors = find_orb_key_points(image)
    elif featureDetector == 'AKAZE':
        keyPoints, descriptors = find_akaze_key_points(image)
    
    return keyPoints, descriptors 


def FeatureMatching(img1,img2,kp1,des1,kp2,des2,featureDetector):
    # FLANN based search: https://docs.opencv.org/master/d1/de0/tutorial_py_feature_homography.html
    if featureDetector =='SIFT':
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    elif featureDetector=='ORB' or featureDetector=='AKAZE':
        FLANN_INDEX_LSH = 6
        index_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 6, # 12
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2
    
    search_params = dict(checks = 50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    if (des1 is None) or (des2 is None):
        ImageMatches = None
        good = []
        src_pts = np.array([])
        dst_pts = np.array([])
        H = 0
        matchesMask = []
        return ImageMatches,good, src_pts, dst_pts, H,matchesMask   
    else:
        matches = flann.knnMatch(des1,des2,k=2)
        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        MIN_MATCH_COUNT = 10
        if len(good)>MIN_MATCH_COUNT:
            print('nr good matches', len(good))
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            #find homography if intrinsic camera matrix is not given
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()
            
            if (M is None or len(matchesMask)<=10):
                ImageMatches = None
                good = []
                src_pts = np.array([])
                dst_pts = np.array([])
                H = 0
                matchesMask = []
                return ImageMatches,good, src_pts, dst_pts, H,matchesMask  
            
            else:
                H = M

                h,w = img1.shape
                pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                dst = cv2.perspectiveTransform(pts,M)

                #copying images:
                img1_copy = img1.copy()
                img2_copy = img2.copy()
                
                img2 = cv2.polylines(img2_copy,[np.int32(dst)],True,255,3, cv2.LINE_AA)
                draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matchesMask, # draw only inliers
                            flags = 2)

                ImageMatches = cv2.drawMatches(img1_copy,kp1,img2_copy,kp2,good,None,**draw_params)

        else:
            print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT))
            matchesMask = None
            ImageMatches = None
            src_pts = np.array([])
            dst_pts = np.array([])
            H =0
            matchesMask = []

    
        return ImageMatches,good, src_pts, dst_pts, H, matchesMask    

