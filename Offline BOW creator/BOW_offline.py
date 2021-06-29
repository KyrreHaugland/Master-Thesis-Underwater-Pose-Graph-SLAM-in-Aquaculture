import numpy as np
import cv2
import os
from scipy import ndimage
from scipy.spatial import distance
from sklearn.cluster import KMeans

from ImageProcessingFunctions import getKeyPointsAndFeatures


#Enabling CLAHS function from MATLAB implementation
# ' This is inspired by https://medium.com/@aybukeyalcinerr/bag-of-visual-words-bovw-db9500331b2f
blurSize = 15 

folder = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/VerticalNF/VerticalNF/test2/Vertical Net Follow - Test2/2021-03-26 10-06-56 - Vertical Net Follow - Test2 - NetFollow/imageSeries'
# takes all images and convert them to grayscale. 
# return a dictionary that holds all images category by category. 
def load_images_from_folder(folder):
    images = {}
    for filename in os.listdir(folder):
        category = []
        path = folder + "/" + filename
        img = cv2.imread(path)

        if img is not None:
            category.append(img)
        images[filename] = category

    return images

images = load_images_from_folder(folder)  # take all images category by category 


def byteArrayToBitArray(byteArray):
    byteArray = byteArray.astype('uint8')
    x_bit = np.unpackbits(byteArray, axis=0)
    return x_bit


def extract_features(images,featureDescriptor = 'SIFT',binary=False):
    feature_vectors = {}
    descriptor_list = []
    #sift = cv2.xfeatures2d.SIFT_create()
    count = 0
    for key,value in images.items():
        features = []
        count += 1
        print(count)
        for img in value:
            kp, des = getKeyPointsAndFeatures(img,featureDescriptor)

            if binary ==True:
                nr_features = des.shape[0]
                des_temp = np.zeros((des.shape[0],des.shape[1]*8)) 
                for idx in range(0,nr_features):
                    des_temp[idx,:] = byteArrayToBitArray(des[idx,:])
                des = des_temp

            #' if only wanting to store unique rows/descriptors
            des = np.unique(des,axis=0)

            descriptor_list.extend(des)
            features.append(des)

        feature_vectors[key] = features
    return [descriptor_list, feature_vectors]

sifts = extract_features(images)
descriptor_list = sifts[0] 
all_bovw_feature = sifts[1] 


# A k-means clustering algorithm who takes 2 parameter which is number 
# of cluster(k) and the other is descriptors list(unordered 1d array)
# Returns an array that holds central points.
def kmeans(k, descriptor_list):
    kmeans = KMeans(n_clusters = k, n_init=10)
    kmeans.fit(descriptor_list)
    visual_words = kmeans.cluster_centers_ 
    return visual_words

# Takes the central points which is visual words    
visual_words = kmeans(150, descriptor_list) 

print('kmeans found')
print(visual_words[1])
#' need to store these words
#storing the visual words in a binary npy format. This array can be accessed by calling visual_words = np.load("visual_words.npy")
np.save("visual_words.npy",visual_words) 


