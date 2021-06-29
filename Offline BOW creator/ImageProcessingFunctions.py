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
    ''' Initiate detector and find key points on an image
    Parameters
    ----------
        image : 2D UInt8 Numpy array - image
        edgeThreshold : int - parameter for OpenCV detector
        nFeatures : int - parameter for OpenCV detector
        nLevels : int - parameter for OpenCV detector
        patchSize : int - parameter for OpenCV detector
    Returns
    -------
        keyPoints : list - coordinates of keypoint on image
        descriptors : list - binary descriptos of kepoints
    '''
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
    print('ORB detector initiated')

    keyPoints, descriptors = detector.detectAndCompute(image, None)
    print('Key points found: %d' % len(keyPoints))
    """[summary]

    Returns:
        [type]: [description]
    """    
    return keyPoints, descriptors 

def find_sift_key_points(image,nOctaveLayers1=4,contrastThreshold1=0.065,edgeThreshold1=20,sigma1=1.6,**kwargs):
    """[Finds keypoints using the SIFT algorithm]

    Args:
        image ([ndarray]): [blured image]
        nOctaveLayers (int, optional): [nr octave layers]. Defaults to 4.
        contrastThreshold (float, optional): [description]. Defaults to 0.065.
        edgeThreshold (int, optional): [description]. Defaults to 20.
        sigma (float, optional): [description]. Defaults to 1.6.

    Returns:
        kp ([ndarray]): [keypoints]
        des ([ndarray]): [descriptors]
    """    
                
    sift = cv2.xfeatures2d.SIFT_create(nOctaveLayers = nOctaveLayers1,contrastThreshold = contrastThreshold1,edgeThreshold = edgeThreshold1,sigma=sigma1)
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
    """[summary]: this function takes in two images and tries to match their features

    Args:
        image1 ([ndarray]): [description]
        image2 ([ndarray]): [description]
        kp1 ([ndarray]): [keypoints]
        des1 ([ndarray]): [feature descriptor]
        kp2 ([ndarray]): [keypoints]
        des2 ([ndarray]): [feature descriptor]
        featureDetector ([string]): [feature detector; e.g 'SIFT']

    Returns:
        [ndarray]: [Feature Matched Image]
        src_pts ([ndarray]): [points from image 1 that are good matches]
        dst_pts ([ndarray]): [points from image 2 that are good matches]
    """    

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
    matches = flann.knnMatch(des1,des2,k=2)
    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)
    print('nr good matches', len(good))
    # Now we set a condition that atleast 10 matches (defined by MIN_MATCH_COUNT) are to be there to find the object. 
    # Otherwise simply show a message saying not enough matches are present.

    # If enough matches are found, we extract the locations of matched keypoints in both the images. 
    # They are passed to find the perspective transformation. Once we get this 3x3 transformation matrix,
    # we use it to transform the corners of queryImage to corresponding points in trainImage. Then we draw it. 
    MIN_MATCH_COUNT = 10
    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        #find homography if intrinsic camera matrix is not given
        
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()

        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)
        
        img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

    else:
        print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
        matchesMask = None

    draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                    singlePointColor = None,
                    matchesMask = matchesMask, # draw only inliers
                    flags = 2)

    ImageMatches = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)

    return ImageMatches, good, src_pts, dst_pts    



def EstimateRotationAndTranslation(src_pts,dst_pts,K=0):
    """[summary]

    Args:
        src_pts ([ndarray]): [points from image 1 that are good matches]
        dst_pts ([ndarray]): [points from image 2 that are good matches]
        K ([ndarray]): [3x3 camera intrinsic matrix]
    """    
    if type(K) is int:
        E = 0
        R_est = 0
        t_est = 0
    else: 
        #' According to page 23 in Trym's book, it should be the normalized image coordinates that should be input the essential matrix estimate. 
        #' Need to check this
        E, mask = cv2.findEssentialMat(src_pts, dst_pts, K, cv2.RANSAC, threshold=1) # can use cv2.LMEDS also
        points, R_est, t_est, mask_pose = cv2.recoverPose(E, src_pts, dst_pts,K)

    #filter keypoint outliers found from essential matrix estimation:
    src_pts_in = []
    dst_pts_in = []
    for i in range(len(mask)):
        if mask[i] == 1:
            src_pts_in.append(src_pts[i])
            dst_pts_in.append(dst_pts[i])
    src_pts_inliers = np.array(src_pts_in)
    dst_pts_inliers = np.array(dst_pts_in)
    
    return E, R_est, t_est, src_pts_inliers, dst_pts_inliers
# N.B.1: trc is estimated up to scale (i.e. the algorithm always returns ||trc||=1, we need a scale in order to recover a translation which is coherent with previous estimated poses)
    # N.B.2: this function has problems in the following cases: [see Hartley/Zisserman Book]
    