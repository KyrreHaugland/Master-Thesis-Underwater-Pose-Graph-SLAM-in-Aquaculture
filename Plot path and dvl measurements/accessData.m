function filename = accessData(x)
    % x represents a symbolic number indicating which data
    switch x
        %Dataset1
        case 1
            %loop closure
            filename = '/home/kyrre/Dokumenter/prosjektoppgave/DatasettFishSim/shared/run2_NF_and_calibration/FhSim/KalmanFilterStandardDPSummer2020.csv';
        case 2
            filename = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared/run3_NF_and_calibration/FhSim/KalmanFilterStandardDPSummer2020.csv';
        %Dataset2:
        % - AutonomousFeatures
        case 3
            filename = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/AutonomousFeatures/AutonomousFeatures/test1/HenningTrackProgressg_TrialRataran_Nr1_20210326.csv';
        case 4
            filename = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/AutonomousFeatures/AutonomousFeatures/test2/HenningTrackProgressg_TrialRataran_Nr2_20210326.csv';
        % - BacksteppingDP_withTarget
        case 5
            filename = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/BacksteppingDP_withTarget/BacksteppingDP_withTarget/test1/BacksteppingAqueous_TrialRataran_Nr1_20210325.csv';
        case 6
            filename = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/BacksteppingDP_withTarget/BacksteppingDP_withTarget/test2/BacksteppingAqueous_TrialRataran_Nr2_20210325.csv';
        % - NonLinearVelocityControl
        case 7
            %loop closure
            filename = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/NonLinearVelocityControl/NonLinearVelocityControl/test1/NonLinVelocityCon_TrialRataran_Nr1_20210325.csv';
        case 8
            %loop closure
            filename = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/NonLinearVelocityControl/NonLinearVelocityControl/test2/NonLinVelocityCon_TrialRataran_Nr2_20210325.csv';
        % - VerticalNF 
        case 9
            %loop closure
            filename = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/VerticalNF/VerticalNF/test1/VerticalNetFollowing_TrialRataran_Nr1_20210326.csv';
        case 10
            filename = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/VerticalNF/VerticalNF/test2/VerticalNetFollowing_TrialRataran_Nr2_20210326.csv';
        case 11
            filename = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/VerticalNF/VerticalNF/test3/VerticalNetFollowing_TrialRataran_Nr3_20210326.csv';
        case 12
            filename = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/VerticalNF/VerticalNF/test4/VerticalNetFollowing_TrialRataran_Nr4_20210326.csv';
        case 13
            filename = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/VerticalNF/VerticalNF/test5/VerticalNetFollowing_TrialRataran_Nr5_20210326.csv';
        case 14
            filename = '/home/kyrre/Dokumenter/prosjektoppgave/masteroppgave/Dataset/shared2/VerticalNF/VerticalNF/test6/VerticalNetFollowing_TrialRataran_Nr6_20210326.csv';
        otherwise
            disp('other value')
    end
end