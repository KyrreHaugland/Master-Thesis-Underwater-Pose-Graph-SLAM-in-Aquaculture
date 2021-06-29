import cv2
import numpy as np
from oct2py import octave
from oct2py import Oct2Py
import os

oc = Oct2Py()
#Enabling CLAHS function from MATLAB implementation
oc.addpath('/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/code/Experiments/Image Processing Experiment/uwit')

# Image directory
directory_to_store_images = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/VerticalNF/VerticalNF/test2/Vertical Net Follow - Test2/2021-03-26 10-06-56 - Vertical Net Follow - Test2 - NetFollow/imageSeries'
os.chdir(directory_to_store_images)

path = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/VerticalNF/VerticalNF/test2/Vertical Net Follow - Test2/2021-03-26 10-06-56 - Vertical Net Follow - Test2 - NetFollow/Vertical Net Follow - Test2 - NetFollow.mp4'
cap = cv2.VideoCapture(path)
blurSize = 15
framerate = 60
frame_count = 0
imageNr = 0
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    frame_count += 1

    if (frame_count%60 ==0):
        # Our operations on the frame come here
        imageNr += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        clahs = oc.clahs(gray,blurSize,blurSize,nout='max_nout')
        blur = cv2.GaussianBlur(clahs,(blurSize,blurSize),cv2.BORDER_DEFAULT)

        filename = 'VerticalNF_test2_image' +str(imageNr)+'.png'
        cv2.imwrite(filename, blur)

    if ret == False:
        break

cap.release()
cv2.destroyAllWindows()