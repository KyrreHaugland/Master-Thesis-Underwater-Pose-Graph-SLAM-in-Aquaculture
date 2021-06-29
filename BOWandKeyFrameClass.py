import numpy as np

from scipy import ndimage
from scipy.spatial import distance
from sklearn.cluster import KMeans
from sklearn.metrics.pairwise import euclidean_distances

from itertools import compress
import time

import cv2
import gtsam

from gtsam import symbol_shorthand
L = symbol_shorthand.L
X = symbol_shorthand.X

from gtsam import (Cal3_S2, DoglegOptimizer,GenericProjectionFactorCal3_S2,
Marginals,NonlinearFactorGraph, PinholeCameraCal3_S2, Point2, Point3,Pose3,
PriorFactorPoint3, PriorFactorPose3, Rot3, Values, BetweenFactorPose3,Pose3AttitudeFactor,NoiseModelFactor, Unit3)
from gtsam.utils import plot

import ImageProcessingFunctions as IP
from SLAMFunctions import rotBody2Ned

from oct2py import octave
from oct2py import Oct2Py
import os
from skimage import exposure
import math

oc = Oct2Py()
oc.addpath('/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/code/Experiments/Image Processing Experiment/uwit') 


class KeyFrame:  
    pose_w_c                = None     

    frame_histogram         = None           
    frame_TF_IDF_histogram  = None
    S_G                     = None           
    S_L                     = None           
    
    def __init__(self,key_prev,key_cur, image_raw, blurSize, featureDetector, frame_prev, kps_prev, des_prev, time_captured, ori_meas):
        #' Initializes KeyFrame by finding keypoints, descriptors, matches with prev image and estimating Pose
        self.frame_id                   = key_cur
        self.frame_cur                  = self.process_image(image_raw,blurSize)
        self.kps_cur, self.des_cur      = IP.getKeyPointsAndFeatures(self.frame_cur, featureDetector)


        self.frame_prev                 = frame_prev        
        self.kps_prev                   = kps_prev
        self.des_prev                   = des_prev

        self.good_matches_keys   = []        
        self.good_matches        = []        

        self.time_captured = time_captured

        #'matching
        image_with_matches, good_features, src_pts, dst_pts, H,matchesMask    = IP.FeatureMatching(frame_prev,self.frame_cur,self.kps_prev,self.des_prev,self.kps_cur,self.des_cur,featureDetector)
        self.good_matches.append(good_features) 
        self.good_matches_keys.append(key_prev) 

        self.keyframe_orientation = ori_meas 

    
    @staticmethod
    def process_image(image,blurSize):
        img_gray            = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        img_clahs           = oc.clahs(img_gray,15,15,nout='max_nout')
        processed_image      = cv2.GaussianBlur(img_clahs,(blurSize,blurSize),cv2.BORDER_DEFAULT)
        return processed_image

    def get_image(self):
        return self.frame_cur
    
    def get_keyframe_id(self):
        return self.frame_id 
    
    def get_local_saliency_and_global_saliency(self):
        return self.S_L, self.S_G
    
    def print_time_captured(self):
        print(self.time_captured)

    def get_keyframe_orientation(self):
        return self.keyframe_orientation

    def get_kps_and_des(self):
        return self.kps_cur, self.des_cur
    
    def matchImages(self,frame_prev, kps_prev, des_prev,featureDetector):
        matchImage, good_features, src_pts, dst_pts,H,matchesMask    = IP.FeatureMatching(frame_prev,self.get_image(),kps_prev,des_prev,self.kps_cur,self.des_cur,featureDetector) #NOTE: Litt usikker på rekkefølge her
        return matchImage,good_features, src_pts, dst_pts,H

    def add_BOW_data(self,image_histogram,TF_IDF_histogram,S_L,S_G):
        # NOTE: good_features should be the descriptors of current image match with previous image
        self.frame_histogram        = image_histogram
        self.frame_TF_IDF_histogram = TF_IDF_histogram
        self.S_L                    = S_L
        self.S_G                    = S_G #BagOfWords.main_function(good_features)
        

    def get_BOW_data(self):
        return self.frame_histogram, self.S_L, self.S_G, self.frame_TF_IDF_histogram

    def get_good_matches(self):
        return self.good_matches

    
    def get_good_features_in_cur_image(self,idx):
        good_features = self.get_good_matches()
        kps_cur, des_cur = self.get_kps_and_des()
        if idx == 0:
            cur_good_kps, cur_good_des = self.convert_matches_to_kp_and_descriptors(good_features[0],kps_cur,des_cur)
        else :
            cur_good_kps = 0
            cur_good_des= 0

        return cur_good_kps, cur_good_des

    def update_global_and_local_saliency_and_TF_IDF_histogram(self,S_L,S_G,TF_IDF_histogram):
        self.S_L                    = S_L
        self.S_G                    = S_G
        self.frame_TF_IDF_histogram = TF_IDF_histogram

    def full_bundle_adjustment(self):
        pass

    def triangulate(self,):
        pass

    @staticmethod
    def convert_matches_to_kp_and_descriptors(good_features,kps_cur,des_cur):
        #inputs kps_cur and des_cur which are the raw feature extraction from the current image.

        cur_good_des = []
        cur_good_kps = []
        for mat in good_features:
            cur_idx = mat.trainIdx      
            #prev_idx = mat.queryIdx see link for more info: https://stackoverflow.com/questions/30716610/how-to-get-pixel-coordinates-from-feature-matching-in-opencv-python 
            cur_good_kps.append(kps_cur[cur_idx].pt)
            cur_good_des.append(des_cur[cur_idx])
            
        cur_good_des = np.asarray(cur_good_des)
        cur_good_kps = np.asarray(cur_good_kps)
        return cur_good_kps, cur_good_des




"originaly inspired by the post https://qvault.io/python/binary-search-tree-in-python/"
class BSTNode:
    def __init__(self, clusterListInput,val=None,d =0):
        self.leftChild = None    #descriptor-of-cluster-senter 1
        self.rightChild = None   #descriptor-of-cluster-senter 2
        self.val = val      #descriptor-of-cluster-senter prev. This value is set to None if it is the initial node.
        self.clusterList = clusterListInput # contains all the clusterInput
        self.depth = d
        self.leafIdx = None #only given to leafs

    def hasRightChild(self):
        return self.rightChild
    
    def hasLeftChild(self):
        return self.leftChild
    
    def insertLeftChild(self,Node):
        self.leftChild = Node
    def insertRightChild(self,Node):
        self.rightChild = Node
    
    def findClusterSize(self):
        return self.clusterList.shape[0]
    
    def getClusterList(self):
        return self.clusterList
    
    def labelLeaf(self,leafIdx):
        self.leafIdx = leafIdx
    
    def getLeafIdx(self):
        return self.leafIdx

    def getDepth(self):
        return self.depth
    
    def getVal(self):
        return self.val


    def evalEuclideanDistance(self,word):
        #only run function if one knows that the node has child nodes
        #returns true if leftNode is closest
        dist_left = euclidean_distances(word,self.hasLeftChild().getVal())
        dist_right = euclidean_distances(word,self.hasRightChild().getVal())
        if (dist_left>dist_right):
            return False 
        else:
            return True
        
        
class binary_search_tree():
    def __init__(self,root_node: BSTNode):
        self.root = root_node
        self.depth = 0 
        self.leafCounter = 0


    def generate_search_tree(self,root_node, depthTracker=0):
        nr_of_descriptors = root_node.findClusterSize()
        depthTracker += 1

        if (depthTracker >self.getTreeDepth()):
            self.incrementTreeDepth()


        if nr_of_descriptors != 1:
            #divide into two clusters
            ClusterListofRoot = root_node.getClusterList()
            kmeans = KMeans(n_clusters = 2, n_init=10)
            kmeans.fit(ClusterListofRoot)

            clusterCenters = kmeans.cluster_centers_ 
            
            #' Adding left and right Cluster to tree 
            leftClusterCenter = np.array([clusterCenters[0]])
            leftCluster = ClusterListofRoot[self.ClusterIndicesNumpy(0,kmeans.labels_)] 
            left_node = BSTNode(leftCluster,leftClusterCenter,depthTracker)
            root_node.insertLeftChild(left_node)
            self.generate_search_tree(left_node,depthTracker)

            rightClusterCenter = np.array([clusterCenters[1]])
            rightCluster= ClusterListofRoot[self.ClusterIndicesNumpy(1,kmeans.labels_)] 
            right_node = BSTNode(rightCluster,rightClusterCenter,depthTracker)
            root_node.insertRightChild(right_node)
            self.generate_search_tree(right_node,depthTracker)

        else:
            root_node.labelLeaf(self.getLeafCounter()) 
            print(root_node.getLeafIdx())
            self.incremenLeafCounter()


    def incrementTreeDepth(self):
        self.depth += 1
    
    def getTreeDepth(self):
        return self.depth
    
    def getLeafCounter(self):
        return self.leafCounter
    
    def incremenLeafCounter(self):
        self.leafCounter += 1
    
    @staticmethod
    def ClusterIndicesNumpy(clustNum, labels_array): 
        return np.where(labels_array == clustNum)[0]

    def singleWordSearch(self,root_node: BSTNode, word):
        if root_node.getLeafIdx() == None:
            #evaluate right vs left 
            leftClosest = root_node.evalEuclideanDistance(word)
            if leftClosest:
                return self.singleWordSearch(root_node.hasLeftChild(),word)
            else:
                return self.singleWordSearch(root_node.hasRightChild(),word)
                
        else:
            leaf_val = root_node.getLeafIdx()
            return leaf_val
    
    def wordSearch(self,root_node: BSTNode, features : np.array, nr_words_in_vocabulary : int):
        nr_words = features.shape[0]
        histogram = np.zeros((1,nr_words_in_vocabulary))
        for i in range(0,nr_words):
            word = np.array([features[i,:]])
            word_idx = self.singleWordSearch(root_node,word)
            histogram[0,word_idx] += 1
        
        return histogram
    
    def get_root_node(self):
        return self.root


def cosine_distance(TF_IDF1, TF_IDF2):
    #this gives a value between 0 and 1. 1 Indicating 100% similar, 0 not similar at all
    cos_dist = TF_IDF1@TF_IDF2.T/(np.linalg.norm(TF_IDF1)* np.linalg.norm(TF_IDF2))
    if math.isnan(cos_dist):
        return 0
    else: 
        return cos_dist

def evaluate_yaw(yaw_th,yaw_cur,yaw_array):
    return np.sin(np.abs(yaw_array-yaw_cur)) <= yaw_th

def evaluate_depth(depth_th,depth_cur,depth_array):
    return np.abs(depth_array-depth_cur)<= depth_th

def evaluate_global_saliency(S_G_th,S_G_cur,S_G_array):
    return np.abs(S_G_cur-S_G_array)<= S_G_th

def generate_depth_and_yaw_filter(depth_th,depth_cur,depth_array,yaw_th,yaw_cur,yaw_array):
    return (evaluate_depth(depth_th,depth_cur,depth_array) & evaluate_yaw(yaw_th,yaw_cur,yaw_array))
   

class BOW(binary_search_tree):

    def __init__(self,root_node, treeDepth, nr_of_words,n_link_proposal,NR_RECENT_IMAGES_TO_EXCLUDE_FROM_LOOP_CLOSURE):
        super().__init__(root_node)
        self.depth                      = treeDepth
        self.leafCounter                = nr_of_words              

        #initializing histograms:
        self.global_histogram           = np.zeros((1,nr_of_words))    # Keeping global word count
        self.word_occurence_in_images   = np.zeros((1,nr_of_words))    
        self.binary_global_histogram    = (np.zeros((1,nr_of_words)) != 0)    # Binary histogram tracking all words discovered from dictionary

        self.N                          = 0                            # Number of images comprising the vocabulary database
        self.W                          = 0                            # Contains number of unique words 
        self.Rmax                       = 0                            # Maximum image rarity discovered

        #loop closure parameters:
        self.cost_matrix = None
        self.TF_IDF_histograms = []
        self.nr_images_excluded_from_loop_closure = NR_RECENT_IMAGES_TO_EXCLUDE_FROM_LOOP_CLOSURE
        self.n_link_proposal = n_link_proposal #number indicating how many best matches one should extract
    
    def local_saliency(self,image_histogram):
        #examine entropy (H):
        non_unique_word_count_in_image = sum(sum(image_histogram))
        nr_of_words_in_vocabulary = self.get_W()
        binary_image_histogram = (image_histogram>0)

        H = 0
        for i in range(0,self.leafCounter):
            word_found = binary_image_histogram[0,i]
            if word_found:
                p_w = image_histogram[0,i]/non_unique_word_count_in_image  
                H += p_w*np.log2(p_w)
        
        H = -H

        S_L = H/np.log2(nr_of_words_in_vocabulary)  

        return H, S_L

    def global_saliency_and_TF_IDF_histogram(self,image_histogram):
        N_t = self.get_N() 
        R_i_t = 0 

        number_of_images_containing_certain_word = self.get_word_occurence_in_images() 
        
        binary_image_histogram = (image_histogram>0)
        
        TF_IDF_histogram = np.zeros((1,binary_image_histogram.shape[1]))
        nr_words_in_image = sum(sum(image_histogram))
        
        for i in range(0,self.getLeafCounter()):
            word_found = binary_image_histogram[0,i]
            if word_found:
                images_containing_word_i = number_of_images_containing_certain_word[0,i]
                IDF = np.log2(N_t/images_containing_word_i)
                R_i_t += IDF
                TF_IDF_histogram[0,i] = (image_histogram[0,i]/nr_words_in_image)*IDF
                
        if R_i_t > self.get_Rmax():
            self.set_Rmax(R_i_t)
        
        if (R_i_t !=0 and self.get_Rmax()!=0): 
            S_G = R_i_t/self.get_Rmax()
        else:
            #start condition for image 1 #this will be updated
            S_G = 0 

        return S_G, TF_IDF_histogram
    

    def update_global_histogram(self,histogram):
        self.global_histogram = self.global_histogram + histogram
    
    
    def find_new_binary_global_histogram(self,image_histogram):
        binary_hist = (image_histogram>0)
        old_global = self.get_binary_global_histogram()

        new_global = np.logical_or(old_global,binary_hist)
        return new_global,binary_hist
    
    def find_newly_discovered_words(self,new_binary_global_histogram):
        old_global = self.get_binary_global_histogram()
        diff =new_binary_global_histogram^old_global #XOR operations: if one index is set to True, one know that this index has been added
        new_words_found = list(compress(range(len(diff[0,:])), diff[0,:])) 
        return new_words_found
    
    #' The saliency algorithm:
    def main_function(self,image_features,keyframeList,batch_size):
        
        #' Generating image histogram for features extracted:
        image_histogram = self.wordSearch(self.get_root_node(), image_features, self.getLeafCounter())

        #' Searching for new words and updating binary_global_histogram:
        new_binary_global_histogram,binary_hist = self.find_new_binary_global_histogram(image_histogram)
        new_words_found = self.find_newly_discovered_words(new_binary_global_histogram)
        self.update_word_occurence_in_images(binary_hist) 
        
        new_S_L, new_S_G, new_TF_IDF_histogram = [], [], []

        self.increment_N()   
        #checks if list is empty
        if not new_words_found:
            #calculate local and global saliency
            H, S_L = self.local_saliency(image_histogram)
            S_G, TF_IDF_histogram = self.global_saliency_and_TF_IDF_histogram(image_histogram)
        else:
            self.update_W(len(new_words_found)) #add all new words found
            self.set_binary_global_histogram(new_binary_global_histogram)

            # update for every batch of size X images
            if (self.get_N()%batch_size ==0) and (self.get_N()!=0):
                #adding this measurement
                H, S_L = self.local_saliency(image_histogram)
                S_G, TF_IDF_histogram = self.global_saliency_and_TF_IDF_histogram(image_histogram)

                #update
                start = time.time()
                new_S_L, new_S_G, new_TF_IDF_histogram = self.update_global_and_local_saliencies_and_TF_IDF_histogram(keyframeList[:-1])
                end = time.time()
                print(f"Update time of local and global saliency and TF_IDF_histogram {end - start}")
            else: 
                start = time.time()
                H, S_L = self.local_saliency(image_histogram)
                end = time.time()
                print(f"Runtime of the local saliency calculation {end - start}")
                S_G,TF_IDF_histogram = self.global_saliency_and_TF_IDF_histogram(image_histogram)

        return image_histogram, H, S_L, S_G, TF_IDF_histogram, new_S_L, new_S_G, new_TF_IDF_histogram
    
    #' These functions should be run when N and W has been updated, 
    def update_global_and_local_saliencies_and_TF_IDF_histogram(self,keyframes: list):
        nr_keyframes = len(keyframes)
        new_S_L = []
        new_S_G = []
        new_TF_IDF_histogram= []
        for idx in range(0,nr_keyframes):
            keyframe = keyframes[idx]
            frame_histogram, S_L_prev, S_G_prev,TF_IDF_histogram = keyframe.get_BOW_data()
            H, S_L_new = self.local_saliency(frame_histogram)
            S_G_new, TF_IDF_histogram = self.global_saliency_and_TF_IDF_histogram(frame_histogram)
            new_S_L.append(S_L_new)
            new_S_G.append(S_G_new)
            new_TF_IDF_histogram.append(TF_IDF_histogram)
        
        return new_S_L, new_S_G, new_TF_IDF_histogram

    def get_word_occurence_in_images(self):
        return self.word_occurence_in_images

    def update_word_occurence_in_images(self,binary_image_histogram):
        self.word_occurence_in_images = self.word_occurence_in_images.astype(int) + binary_image_histogram 
        
    def increment_N(self):
        self.N += 1

    def set_N(self,value):
        self.N = value

    def get_N(self):
        return self.N

    def get_W(self):
        return self.W
    
    def update_W(self,nr_new_words):
        self.W += nr_new_words

    def get_Rmax(self):
        return self.Rmax

    def set_Rmax(self,Rmax_new):
        self.Rmax = Rmax_new
    
    def get_binary_global_histogram(self):
        return self.binary_global_histogram
    
    def set_binary_global_histogram(self,binary_histogram):
        self.binary_global_histogram = binary_histogram

    #' Loop Closure Functions:
    def increment_cost_matrix(self,new_TF_IDF_histogram):
        TF_IDF_histograms = self.get_TF_IDF_histograms()
        new_cost_matrix = self.get_cost_matrix()
        nr_previous_histograms = len(TF_IDF_histograms)-1
        
        if new_cost_matrix is None:
            new_row = np.zeros((1,1))
            self.set_cost_matrix(new_row)

        else:
            rows, cols = new_cost_matrix.shape
            new_row = np.zeros((1,cols))

            for idx in range(0,nr_previous_histograms):
                #compare all histograms distance
                if idx ==0:
                    print(TF_IDF_histograms[idx])
                new_row[0,idx] = cosine_distance(new_TF_IDF_histogram, TF_IDF_histograms[idx])

            new_cost_matrix = np.concatenate((new_cost_matrix,new_row),axis=0) #add new row
            new_row = np.concatenate((new_row,np.zeros((1,1))),axis=1)
            new_cost_matrix = np.concatenate((new_cost_matrix,new_row.T),axis=1) #add new column

            #replacing old TF_IDF_array
            self.set_cost_matrix(new_cost_matrix)

        return new_row 

    def extract_n_link_hypothesis(self,filtered_array,filtered_array_idx):
        is_empty = (filtered_array_idx.size == 0)
        
        if is_empty:
            return []
        else:
            #check nr of links
            length_filtered_array = len(filtered_array)
            n_links = self.get_n_link_proposal()
            if length_filtered_array < n_links:
                print('filtered_array',filtered_array)
                print(filtered_array.shape)
                indices = np.argpartition(filtered_array, -length_filtered_array)[-length_filtered_array:]
                link_proposals = []
                for index in indices:
                    link_proposals.append((filtered_array[index], filtered_array_idx[index]))#' returns the index in the original array 
                link_proposals.sort(reverse=True)
                link_proposals = [(b, a) for a, b in link_proposals]
            else:
                indices = np.argpartition(filtered_array, -n_links)[-n_links:]
                link_proposals = []
                for index in indices:
                    link_proposals.append((filtered_array[index], filtered_array_idx[index]))#' returns the index in the original array 
                link_proposals.sort(reverse=True)
                link_proposals = [(b, a) for a, b in link_proposals]
            #return link_proposals[x][y] where x determines the best match, while y is (0 or 1) indicating index and value respectively
            return link_proposals

    

    def get_n_link_proposal(self):
        return self.n_link_proposal


    def update_cost_matrix():
        pass


    def get_cost_matrix(self):
        return self.cost_matrix

    def add_TF_IDF_histogram(self,TF_IDF_histogram):
        self.TF_IDF_histograms.append(TF_IDF_histogram)

    def remove_TF_IDF_histogram(self):
        self.TF_IDF_histograms.pop()    

    def get_nr_images_excluded_from_loop_closure(self):
        return self.nr_images_excluded_from_loop_closure

    def get_TF_IDF_histograms(self):
        return self.TF_IDF_histograms
    
    def update_TF_IDF_histograms(self,idx,new_TF_IDF_histogram):
        self.TF_IDF_histograms[idx] = new_TF_IDF_histogram


    def set_cost_matrix(self,new_cost_matrix):
        self.cost_matrix = new_cost_matrix

    #NOTE: this should only be called if global saliency of image is high enough
    def loop_closure_link_hypothesis(self,depth_th,depth_cur,depth_array,yaw_th,yaw_cur,yaw_array,new_TF_IDF_histogram):
        #look for changes in vocabulary size
        #if changes: update length of TF-IDF diagrams
        ignore_last_x_images = self.get_nr_images_excluded_from_loop_closure()
        
        new_row = self.increment_cost_matrix(new_TF_IDF_histogram)

        #don't include the previous x images in the loop closure
        new_row = new_row[0,:-ignore_last_x_images]

        #don't do loop closure if the nr of images accumulated is not high enough
        if (depth_array.shape[1]> ignore_last_x_images):
            filter = generate_depth_and_yaw_filter(depth_th,depth_cur,depth_array[0,:-ignore_last_x_images],yaw_th,yaw_cur,yaw_array[0,:-ignore_last_x_images])
            #setting values in new_row to 0 if not a potential candidate
            filtered_candidates = filter*new_row
            filtered_candidates_index = np.nonzero(filtered_candidates)
            filtered_candidates = filtered_candidates[filtered_candidates_index]

            #get n-best_candidates
            n_link_hypothesis = self.extract_n_link_hypothesis(filtered_candidates,filtered_candidates_index[0])
            return n_link_hypothesis
            

        else:
            #returns an empy list if too early in the SLAM
            return []


def validate_loop_closure(keyframe_current,keyframe_prev,K,featureDetector,HOMOGRAPHY_TH,n_dvl):
    MATCH_FOUND = False
    #this function takes in two keyframes: the current keyframe and the previous seen keyframe
    image_current = keyframe_current.get_image()
    image_prev = keyframe_prev.get_image()
    kps_cur_image, des_cur_image = IP.getKeyPointsAndFeatures(image_current,featureDetector) #keyframe_list[-1].get_kps_and_des()
    kps_prev_image, des_prev_image = IP.getKeyPointsAndFeatures(image_prev,featureDetector)#keyframe_list[prev_image_idx].get_kps_and_des()

    #1. it matches features using Homography
    ImageMatches, good, src_pts, dst_pts, H,matchesMask = IP.FeatureMatching(image_current,image_prev,kps_cur_image,des_cur_image,kps_prev_image,des_prev_image,featureDetector)

    #2. validates the homography
    #3. Decomposes the Homography into 4 possible transformation solutions
    #4. Reduce into 2 solutions based on the fact that the points should be in front of both images
    if matchesMask is None:
        ('not good enough matches or Homography is incorrect')
        return None, None, None, MATCH_FOUND 
    elif len(matchesMask) <=10 or np.linalg.det(H)<HOMOGRAPHY_TH:
        ('not good enough matches or Homography is incorrect')
        return None, None, None, MATCH_FOUND 
    else:
        MATCH_FOUND = True
            #input good matches                           
        #R, T = IP.EstimateRotationAndTranslationHomography(H,K)
        num, Rs, Ts, Ns = cv2.decomposeHomographyMat(H,K)
                                

        # filter negativ depth solutions:
        Rs_filtered = []
        Ts_filtered = []
        Ns_filtered = []
        for i in range(0,len(Ns)):
            #removing solutions that does not have positive depth                        
            if Ns[i][2,0]>0:
                Rs_filtered.append(Rs[i])
                Ts_filtered.append(Ts[i])
                Ns_filtered.append(Ns[i])

        #obtain rotation matrix for when keyframes was taken based on absolute measurements of yaw,pitch and roll 
        ori_meas_cur = keyframe_current.get_keyframe_orientation()
        ori_meas_prev = keyframe_prev.get_keyframe_orientation()
        abs_Rw_cur = rotBody2Ned(ori_meas_cur[0,2],ori_meas_cur[0,1],ori_meas_cur[0,0])
        abs_Rw_prev = rotBody2Ned(ori_meas_prev[0,2],ori_meas_prev[0,1],ori_meas_prev[0,0])
        abs_Rprev_cur = (abs_Rw_prev.T)@abs_Rw_cur

        #5. Determines which of the 2 remaining solutions that is correct
        matchIdx = evaluate_euclidean_distance_of_normal_vectors(n_dvl,Ns_filtered)

        return ImageMatches, Rs_filtered[matchIdx], Ts_filtered[matchIdx], MATCH_FOUND


def evaluate_euclidean_distance_of_normal_vectors(n_dvl,n_homography):
    #change the orientation of the normal vector obtained from the DVL
    n_dvl = -n_dvl
    dist = 1000  #a high number
    for i in range(0,len(n_homography)):
        dist_i = np.linalg.norm(n_dvl-n_homography[i])
        if dist_i < dist:
            dist = dist_i
            idx = i 

    return idx


    
    