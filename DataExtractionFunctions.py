import pandas as pd
import numpy as np
import math
from datetime import datetime
from matplotlib import pyplot as plt

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def extractFhSimData(FhSimDataPath):
    fhSimData = pd.read_csv(FhSimDataPath,delimiter = ';')
    return fhSimData

def extractTelemetriData(TelemetriDataPath):
    CSV = pd.read_csv(TelemetriDataPath,delimiter = ';')
    telemetriData = CSV[1:] #removing the first row that is empty
    return telemetriData

#From https://kodify.net/python/math/round-decimals/
def round_decimals_down(number:float, decimals:int=2):
    """
    Returns a value rounded down to a specific number of decimal places.
    """
    if not isinstance(decimals, int):
        raise TypeError("decimal places must be an integer")
    elif decimals < 0:
        raise ValueError("decimal places has to be 0 or more")
    elif decimals == 0:
        return math.floor(number)

    factor = 10 ** decimals
    return math.floor(number * factor) / factor

#From https://www.knowledgehut.com/blog/programming/python-rounding-numbers
def round_decimals_up(number:float, decimals:int=2): 
    multiplier = 10 ** decimals 
    return math.ceil(number * multiplier) / multiplier

def extractCSVinfo(telemetriData,clockStartTime,clockEndTime):
    """[summary]

    Args:
        TelemetriDataPath ([string]): [Path and File name of telemetri data CSV]
        clockStartTimeVideo ([string]): [Start of video recording]
        clockEndTimeVideo ([string]): [End of video recording]

    Returns:
        Time ([ndarray]): [time index]
        RPM ([ndarray]): [raw RPM data]
        EtaMeasured ([ndarray]): [x,y,z measurements]
        NuMeasured ([ndarray]): [u,v,w,r measurements]
        USBLAvail ([ndarray]): [binary variable indicating USBL availability]
        DVLAvail ([ndarray]): [binary variable indicating DVL availability]
        EtaHat ([ndarray]): [Kalman filter estimate of pose]
        NuHat ([ndarray]): [Kalman filter estimate of pose]
        BiasHat ([ndarray]): [Kalman filter bias estimate]
        DVL_beams ([ndarray]): [DVL beam measurements]
        RPM2Tau ([ndarray]): [RPM2Tau]
        CmdTau ([ndarray]): [CmdTau]
    """    
    
    # ' Filtering based on video data:
    DataOnlyRelevant = telemetriData[(telemetriData.Time >= clockStartTime) & (telemetriData.Time <= clockEndTime)]

    #' Extracting data:
    Time                        = np.array([pd.to_numeric(DataOnlyRelevant['Time']).to_numpy()])

    RPM                         = (DataOnlyRelevant[['ROV.26','ROV.27','ROV.28','ROV.29','ROV.30','ROV.31']].apply(pd.to_numeric, errors='coerce', axis=1)).to_numpy()
    EtaMeasured4                = (DataOnlyRelevant[['Pos4DOFMux','Pos4DOFMux.1','Pos4DOFMux.2','Pos4DOFMux.3']].apply(pd.to_numeric, errors='coerce', axis=1)).to_numpy()
    NuMeasured4                 = (DataOnlyRelevant[['Vel4DOFMux','Vel4DOFMux.1','Vel4DOFMux.2','Vel4DOFMux.3']].apply(pd.to_numeric, errors='coerce', axis=1)).to_numpy()
    EtaMeasured6                = (DataOnlyRelevant[['EstPos6DOFMux','EstPos6DOFMux.1','EstPos6DOFMux.2','EstPos6DOFMux.3','EstPos6DOFMux.4','EstPos6DOFMux.5']].apply(pd.to_numeric, errors='coerce', axis=1)).to_numpy()
    NuMeasured6                 = (DataOnlyRelevant[['EstVel6DOFMux','EstVel6DOFMux.1','EstVel6DOFMux.2','EstVel6DOFMux.3','EstVel6DOFMux.4','EstVel6DOFMux.5']].apply(pd.to_numeric, errors='coerce', axis=1)).to_numpy()

    USBLAvail                   = np.array([pd.to_numeric(DataOnlyRelevant['USBL.17']).to_numpy()])
    DVLAvail                    = np.array([pd.to_numeric(DataOnlyRelevant['DVL.14']).to_numpy()])

    EtaHat                      = (DataOnlyRelevant[['KalmanFilter.4','KalmanFilter.5','KalmanFilter.6','KalmanFilter.7']].apply(pd.to_numeric, errors='coerce', axis=1)).to_numpy()  
    NuHat                       = (DataOnlyRelevant[['KalmanFilter.8','KalmanFilter.9','KalmanFilter.10','KalmanFilter.11']].apply(pd.to_numeric, errors='coerce', axis=1)).to_numpy() 
    BiasHat                     = (DataOnlyRelevant[['KalmanFilter','KalmanFilter.1','KalmanFilter.2','KalmanFilter.3']].apply(pd.to_numeric, errors='coerce', axis=1)).to_numpy()

    USBL_raw                    = (DataOnlyRelevant[['USBL','USBL.1','USBL.2']].apply(pd.to_numeric, errors='coerce', axis=1)).to_numpy()

    EtaMeasured4[:,0:2]          = EtaMeasured4[:,0:2]*USBLAvail.T     
    NuMeasured4[:,0:3]           = NuMeasured4[:,0:3]*DVLAvail.T
    EtaMeasured6[:,0:2]          = EtaMeasured6[:,0:2]*USBLAvail.T     
    NuMeasured6[:,0:3]           = NuMeasured6[:,0:3]*DVLAvail.T

    #AbsoluteMeasurements include depth, roll, pitch and yaw
    AbsoluteMeasurements = (DataOnlyRelevant[['ROV.8','ROV.9','ROV.10','ROV.11']].apply(pd.to_numeric, errors='coerce', axis=1)).to_numpy()
    for i in range(0,len(EtaMeasured4)):
        EtaMeasured4[i,3]        = pi_2_pi(EtaMeasured4[i,3])
        EtaHat[i,3]              = pi_2_pi(EtaHat[i,3]) 
        EtaMeasured6[i,3]        = pi_2_pi(EtaMeasured6[i,3]) 
        EtaMeasured6[i,4]        = pi_2_pi(EtaMeasured6[i,4])  
        EtaMeasured6[i,5]        = pi_2_pi(EtaMeasured6[i,5])  
        AbsoluteMeasurements[i,1]= pi_2_pi(AbsoluteMeasurements[i,1])
        AbsoluteMeasurements[i,2]= pi_2_pi(AbsoluteMeasurements[i,2])
        AbsoluteMeasurements[i,3]= pi_2_pi(AbsoluteMeasurements[i,3])


    NetFollowingMode            = np.array([pd.to_numeric(DataOnlyRelevant['Aqueous.23']).to_numpy()])
    DVL_beams                   = (DataOnlyRelevant[['DVL.6','DVL.7','DVL.8','DVL.9']].apply(pd.to_numeric, errors='coerce', axis=1)).to_numpy()
    DVL_beams_flags             = (DataOnlyRelevant[['DVL.10','DVL.11','DVL.12','DVL.13']].apply(pd.to_numeric, errors='coerce', axis=1)).to_numpy()

    RPM2Tau                     = (DataOnlyRelevant[['RPM2Tau','RPM2Tau.1','RPM2Tau.2','RPM2Tau.3','RPM2Tau.4','RPM2Tau.5']].apply(pd.to_numeric, errors='coerce', axis=1)).to_numpy()
    CmdTau                      = (DataOnlyRelevant[['ModeSwitcher','ModeSwitcher.5','ModeSwitcher.2','ModeSwitcher.3','ModeSwitcher.4','ModeSwitcher.2']].apply(pd.to_numeric, errors='coerce', axis=1)).to_numpy()
    
    

    return Time, RPM, EtaMeasured4, NuMeasured4, USBLAvail, DVLAvail, EtaHat, NuHat, BiasHat, DVL_beams, DVL_beams_flags, RPM2Tau, CmdTau, NetFollowingMode, EtaMeasured6, NuMeasured6,AbsoluteMeasurements, USBL_raw

def extractClockTime(fhSimData,telemetriData,videoStartTimeLocal,videoLength):
    """[Returns the clock start time of when video and telemetri started recording, and also returns clock end time of video]

    Args:
        fhSimData ([mp4]): [Video Data]
        telemetriData ([csv]): [Telemetri Data]
        videoStartTimeLocal ([str]): [Video start time]
        VideoLength ([float]): [Video length in seconds]
clockStartTime, clockEndTime, nrCentiSecondVideoRoundedUp, TELEMETRI_STARTS_RECORDING_LAST 
    Returns:
        clockStartTime ([float]): [Start time when both video and telemetri are active]
        clockEndTime ([float]): [End time when both video and telemetri are deactived]
        nrCentiSecondVideoRoundedUp ([float]): [centiseconds when the videoStarts if the telemetri starts before video]
        TELEMETRI_STARTS_RECORDING_LAST ([boolean]): [describing if telemetri starts recording after the video]
    """    
    # ? NOTE : There are several cases that needs to be handled to synchronize the videostream and the telemetri data. 
    # ? The cases are as follows:
    # ? Case I   - Video starts to record AFTER telemetri data logging && Video stream ends BEFORE telemetri logging
    # ? Case II  - Video starts to record AFTER telemetri data logging && Video stream ends AFTER telemetri logging
    # ? Case III - Video starts to record BEFORE telemetri data logging && Video stream ends BEFORE telemetri logging
    # ? Case IV  - Video starts to record BEFORE telemetri data logging && Video stream ends AFTER telemetrfrom datetime import datetimei logging

    #boolean: if 1 then telemetri starts recording after video
    TELEMETRI_STARTS_RECORDING_LAST = 0


    # 'nr desci seconds in both datasets before telemetri data is sampled
    clockStartTimeTelemetri = findClockStartTimeOfTelemetri(telemetriData)
    telemetriStartDesciSecond = int(clockStartTimeTelemetri*10)
    print('TELEMETRISTART in desciseconds:',clockStartTimeTelemetri)

    nrCentiSecondVideoRoundedUp = 0
    
    # ' Extract clock start and end time
    DataWhenStart = fhSimData.loc[fhSimData['LocalTime'].str.contains(videoStartTimeLocal, case=False)]
    no_startVideoData = DataWhenStart.empty

    if (no_startVideoData):
        # ? Handles case III and IV: Video starts to record BEFORE telemetri data logging, .....
        # ? : if video start before data recording, use Telemetri clockStartTime

        clockStartTime = clockStartTimeTelemetri 
        TELEMETRI_STARTS_RECORDING_LAST = 1
        clockStartTimeVideo = 0 
    else:
        # ? Handles case I and II: Video starts to record AFTER telemetri data logging, .....
        clockStartTimeVideo = DataWhenStart.iloc[0,0]

        if (clockStartTimeVideo<=clockStartTimeTelemetri):
            clockStartTime = clockStartTimeTelemetri
            TELEMETRI_STARTS_RECORDING_LAST = 1
        else:
            nrCentiSecondVideoRoundedUp = countCentisecond(clockStartTimeVideo) 
            clockStartTime = round_decimals_up(clockStartTimeVideo, 1)

    #Testing if the end time actually exists in the dataset:
    expectedEndTime = clockStartTimeVideo + videoLength
    
    endTimeExists = fhSimData['Timestep'] == expectedEndTime
    endTimeData = fhSimData[endTimeExists]

    # ' This part handles case 1 and 2
    no_endData = endTimeData.empty
    #case if telemetri data is not as long as video - Meaning not getting expected output
    if (no_endData):
        # ? Case II and IV: ...., Video stream ends AFTER telemetri logging
        clockEndTimeTelemetri = fhSimData.iloc[-1:,0]
        clockEndTimeTelemetri = clockEndTimeTelemetri.iloc[0]

        clockEndTimeVideo = clockEndTimeTelemetri 

        #Rounding down to fit with telemetri dataset
        clockEndTimeVideo = round_decimals_down(clockEndTimeVideo,1)
        print('Using last recorded end time: ',clockEndTimeVideo)

    else:
        # ? Case I and III: ...., Video stream ends BEFORE telemetri logging
        clockEndTimeVideo = expectedEndTime
        #Rounding down to fit with telemetri data 
        clockEndTimeVideo = round_decimals_down(clockEndTimeVideo,1) 
        print('Using expected end time: ', clockEndTimeVideo)
    
    clockEndTime = clockEndTimeVideo
    
    return clockStartTime, clockEndTime, nrCentiSecondVideoRoundedUp, TELEMETRI_STARTS_RECORDING_LAST 


def findClockStartTimeOfTelemetri(telemetriData):
    Time                        = np.array([pd.to_numeric(telemetriData['Time']).to_numpy()])

    USBLAvail                   = np.array([pd.to_numeric(telemetriData['USBL.17']).to_numpy()])
    print(USBLAvail.shape)
    DVLAvail                    = np.array([pd.to_numeric(telemetriData['DVL.14']).to_numpy()])
    print(DVLAvail.shape)
    AbsMeas                     = (telemetriData[['ROV.8','ROV.9','ROV.10','ROV.11']].apply(pd.to_numeric, errors='coerce', axis=1)).to_numpy()

    for index, row in telemetriData.iterrows():
        # if all telemetri data of interrest is available at a certain clocktime
        AbsMeasIdx = list(AbsMeas[index,:])

        if(USBLAvail[0,index]!=0 and DVLAvail[0,index]!=0 and AbsMeasIdx[0]!=0 and AbsMeasIdx[1]!=0 and AbsMeasIdx[2]!=0 and AbsMeasIdx[3]!=0):
            clockStartTimeTelemetri = Time[0,index]
            break

    return clockStartTimeTelemetri

def findStartFrameOfVideo(FhSimData,clockStartTime,videoStartTimeLocal,nrCentiSecondVideoRoundedUp,TELEMETRI_STARTS_RECORDING_LAST,frameRate):
    if TELEMETRI_STARTS_RECORDING_LAST:
        telemetriStartTimeLocal, telemetriStartTimeCentiseconds = findTelemetriStartTimeLocal(clockStartTime,FhSimData)

        # calculate time difference [s] between video and telemetri starting recording
        timeDiff = calculateTimeDifference(videoStartTimeLocal,telemetriStartTimeLocal)+(telemetriStartTimeCentiseconds)
        print('timediff',timeDiff)

        # calculate how many frames this time difference is
        startFrame = math.ceil(frameRate*(timeDiff))

    else:
        startFrame = math.ceil(nrCentiSecondVideoRoundedUp*frameRate)
    return startFrame

def findTelemetriStartTimeLocal(clockStartTime,FhSimData):
    telemetriStartTimeLocal = FhSimData[FhSimData.Timestep==clockStartTime].iloc[0,2]

    DataContainingTelemetriLocalTime = FhSimData.loc[FhSimData['LocalTime']==telemetriStartTimeLocal]
    firstRecordedClockTimeAtTelemetriLocalTime = DataContainingTelemetriLocalTime.iloc[0,0]
    
    #The time when telemetriLocalTime is recorded 
    telemetriStartTimeCentiseconds = clockStartTime - firstRecordedClockTimeAtTelemetriLocalTime 
    print('telemetriStartTimeCentiseconds',telemetriStartTimeCentiseconds)

    return telemetriStartTimeLocal, telemetriStartTimeCentiseconds

def calculateTimeDifference(startTime,endTime):
    endTimeSplit = endTime.split(", ")
    date_format = "%H:%M:%S"

    
    time_start = startTime 
    time_end = endTimeSplit[1]   

    # Then get the difference here.    
    diff = datetime.strptime(time_end, date_format) - datetime.strptime(time_start, date_format)

    return diff.seconds

def countCentisecond(x):
    x_round_down = round_decimals_down(x,1)
    x_centiSeconds = x- x_round_down
    if(x_centiSeconds<5):
        centisecond = (10-x_centiSeconds)/100
    else:
        centisecond = x_centiSeconds/100
    return centisecond

