import numpy as np

def access_data(data_idx):
    #' Dataset 1
    if data_idx == 1:
        TelemetriDataPath = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared/run2_NF_and_calibration/FhSim/KalmanFilterStandardDPSummer2020.csv'
        FhSimPath = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared/run2_NF_and_calibration/FhSim/FhSimTimeLog.csv'
        videoFile = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared/run2_NF_and_calibration/video/2020-10-28 14-42-57 - run2_with_calibration/My Great Game - My Great Capture.mp4'

        #' Video info
        videoStartTimeLocal = '14:42:57' 
        numberOfFrames = 33855
        videoLengthMin = 9  
        videoLengthSec = 24
        videoLengthMiliSec = 0.817 
        videoLength = videoLengthMin*60 + videoLengthSec + videoLengthMiliSec

        return TelemetriDataPath, FhSimPath, videoFile,videoStartTimeLocal ,videoLength
    elif data_idx == 2:
        TelemetriDataPath = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared/run3_NF_and_calibration/FhSim/KalmanFilterStandardDPSummer2020.csv'
        FhSimPath = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared/run3_NF_and_calibration/FhSim/FhSimTimeLog.csv'
        videoFile = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared/run3_NF_and_calibration/video/2020-10-28 15-06-40 - run3_with_calibration/My Great Game - My Great Capture.mp4'

        #' Video info
        videoStartTimeLocal = '15:06:40'  
        numberOfFrames = 60471
        videoLengthMin = 16  
        videoLengthSec = 48
        videoLengthMiliSec = 0.858 
        videoLength = videoLengthMin*60 + videoLengthSec + videoLengthMiliSec

        return TelemetriDataPath, FhSimPath, videoFile,videoStartTimeLocal ,videoLength
        
    #' Dataset 2
    #' - AutonomousFeatures
    elif data_idx == 3:
        TelemetriDataPath = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/AutonomousFeatures/AutonomousFeatures/test1/HenningTrackProgressg_TrialRataran_Nr1_20210326.csv'
        FhSimPath = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/AutonomousFeatures/AutonomousFeatures/test1/FhSimTimeLog.csv'
        videoFile = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/AutonomousFeatures/AutonomousFeatures/test1/Mission Script & Robust NF - Test1/2021-03-26 11-34-28 - Mission Script & Robust NF - Test1 - AutonomousFeaturesNF/Mission Script & Robust NF - Test1 - AutonomousFeaturesNF.mp4'

        #' Video info
        videoStartTimeLocal = '11:34:28'  
        numberOfFrames = 65941
        videoLengthMin = 18  
        videoLengthSec = 19
        videoLengthMiliSec = 0.898 
        videoLength = videoLengthMin*60 + videoLengthSec + videoLengthMiliSec

        return TelemetriDataPath, FhSimPath, videoFile,videoStartTimeLocal ,videoLength
    elif data_idx == 4:
        TelemetriDataPath = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/AutonomousFeatures/AutonomousFeatures/test2/HenningTrackProgressg_TrialRataran_Nr2_20210326.csv'
        FhSimPath = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/AutonomousFeatures/AutonomousFeatures/test2/FhSimTimeLog.csv'
        videoFile = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/AutonomousFeatures/AutonomousFeatures/test2/Mission Script & Robust NF - Test2/2021-03-26 12-07-10 - Mission Script & Robust NF - Test2 - AutonomousFeaturesNF/Mission Script & Robust NF - Test2 - AutonomousFeaturesNF.mp4'

        #' Video info
        videoStartTimeLocal = '12:07:10' 
        numberOfFrames = 29159 
        videoLengthMin = 8  
        videoLengthSec = 6
        videoLengthMiliSec = 0.47 
        videoLength = videoLengthMin*60 + videoLengthSec + videoLengthMiliSec
        print('do something')
        return TelemetriDataPath, FhSimPath, videoFile,videoStartTimeLocal ,videoLength

    #' - NonLinearVelocityControl
    elif data_idx == 7:
        TelemetriDataPath = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/NonLinearVelocityControl/NonLinearVelocityControl/test1/NonLinVelocityCon_TrialRataran_Nr1_20210325.csv'
        FhSimPath = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/NonLinearVelocityControl/NonLinearVelocityControl/test1/FhSimTimeLog.csv'
        videoFile = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/NonLinearVelocityControl/NonLinearVelocityControl/test1/NonLinVelocityCon -Test1/2021-03-25 15-44-39 - NonLinVelocityCon -Test1 - NetFollow/NonLinVelocityCon -Test1 - NetFollow.mp4'

        #' Video info
        videoStartTimeLocal = '15:44:39' 
        numberOfFrames = 21608 
        videoLengthMin = 6  
        videoLengthSec = 0
        videoLengthMiliSec = 0.49 
        videoLength = videoLengthMin*60 + videoLengthSec + videoLengthMiliSec
        print('do something')
        return TelemetriDataPath, FhSimPath, videoFile,videoStartTimeLocal ,videoLength
    

    
    