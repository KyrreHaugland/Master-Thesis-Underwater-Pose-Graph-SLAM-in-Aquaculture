clear all; close all; clc;
%addpath('./MSS/FDI/GNC/');
addpath('HelpFunctions')

%% Read in ROV states
filename = accessData(4);
CSV = readtable(filename);
Data = CSV(2:size(CSV,1),:);

%%
Time                        = Data.Time;

RPM                         = [Data.ROV_26,Data.ROV_27,Data.ROV_28,Data.ROV_29,Data.ROV_30,Data.ROV_31];
EtaMeasured                 = [Data.Pos4DOFMux,Data.Pos4DOFMux_1,Data.Pos4DOFMux_2,Data.Pos4DOFMux_3];
NuMeasured                  = [Data.Vel4DOFMux,Data.Vel4DOFMux_1,Data.Vel4DOFMux_2,Data.Vel4DOFMux_3];

USBLAvail                   = Data.USBL_17;
DVLAvail                    = Data.DVL_14;

EtaHat                      = [Data.KalmanFilter_4,Data.KalmanFilter_5,Data.KalmanFilter_6,Data.KalmanFilter_7];
NuHat                       = [Data.KalmanFilter_8,Data.KalmanFilter_9,Data.KalmanFilter_10,Data.KalmanFilter_11];
BiasHat                     = [Data.KalmanFilter,Data.KalmanFilter_1,Data.KalmanFilter_2,Data.KalmanFilter_3];

EtaMeasured(:,1:2)          = EtaMeasured(:,1:2).* USBLAvail; %XY-position is obtained from USBL measurements
NuMeasured(:,1:3)           = NuMeasured(:,1:3).* DVLAvail;

% Map angles to [-pi,pi]
for i=1:length(Time)
    EtaMeasured(i,4)        = ssa(EtaMeasured(i,4),'rad');
    EtaHat(i,4)             = ssa(EtaHat(i,4),'rad');
end

RPM2Tau                     = [Data.RPM2Tau,Data.RPM2Tau_1,Data.RPM2Tau_2,Data.RPM2Tau_3,Data.RPM2Tau_4,Data.RPM2Tau_5];
CmdTau                      = [Data.ModeSwitcher,Data.ModeSwitcher_5,Data.ModeSwitcher_2,Data.ModeSwitcher_3,Data.ModeSwitcher_4,Data.ModeSwitcher_2];

%%

figure(1);clf;
    subplot(221);
        plot(Time,EtaMeasured(:,1));
        hold on;
        plot(Time,EtaHat(:,1),'--');
        hold off;
        legend({'$x_{\rm{meas}}$','$\hat{x}$'},'interpreter','latex');
    subplot(222);
        plot(Time,EtaMeasured(:,2));
        hold on;
        plot(Time,EtaHat(:,2),'--');
        hold off;
        legend({'$y_{\rm{meas}}$','$\hat{y}$'},'interpreter','latex');
    subplot(223);
        plot(Time,EtaMeasured(:,3));
        hold on;
        plot(Time,EtaHat(:,3),'--');
        hold off;
        legend({'$z_{\rm{meas}}$','$\hat{z}$'},'interpreter','latex');
    subplot(224);
        plot(Time,EtaMeasured(:,4));
        hold on;
        plot(Time,EtaHat(:,4),'--');
        hold off;
        legend({'$\psi_{\rm{meas}}$','$\hat{\psi}$'},'interpreter','latex');
    sgtitle('Position');
    
figure(2);clf;
    subplot(221);
        plot(Time,NuMeasured(:,1));
        hold on;
        plot(Time,NuHat(:,1),'--');
        hold off;
        legend({'$u_{\rm{meas}}$','$\hat{u}$'},'interpreter','latex');
    subplot(222);
        plot(Time,NuMeasured(:,2));
        hold on;
        plot(Time,NuHat(:,2),'--');
        hold off;
        legend({'$v_{\rm{meas}}$','$\hat{v}$'},'interpreter','latex');
    subplot(223);
        plot(Time,NuMeasured(:,3));
        hold on;
        plot(Time,NuHat(:,3),'--');
        hold off;
        legend({'$w_{\rm{meas}}$','$\hat{w}$'},'interpreter','latex');
    subplot(224);
        plot(Time,NuMeasured(:,4));
        hold on;
        plot(Time,NuHat(:,4),'--');
        hold off;
        legend({'$r_{\rm{meas}}$','$\hat{r}$'},'interpreter','latex');
    sgtitle('Velocity');

figure(3);clf;
    subplot(221);
        plot(Time,BiasHat(:,1));
        legend({'$\hat{b}_1$',},'interpreter','latex');
    subplot(222);
        plot(Time,BiasHat(:,2));
        legend({'$\hat{b}_2$',},'interpreter','latex');
    subplot(223);
        plot(Time,BiasHat(:,3));
        legend({'$\hat{b}_3$',},'interpreter','latex');
    subplot(224);
        plot(Time,BiasHat(:,4));
        legend({'$\hat{b}_4$',},'interpreter','latex');
    sgtitle('Bias');
    
figure(4);clf;
    subplot(321);
        plot(Time,RPM(:,1));
        legend({'$\tau_1$',},'interpreter','latex');
    subplot(322);
        plot(Time,RPM(:,2));
        legend({'$\tau_2$',},'interpreter','latex');
    subplot(323);
        plot(Time,RPM(:,3));
        legend({'$\tau_3$',},'interpreter','latex');
    subplot(324);
        plot(Time,RPM(:,4));
        legend({'$\tau_4$',},'interpreter','latex');
    subplot(325);
        plot(Time,RPM(:,5));
        legend({'$\tau_5$',},'interpreter','latex');
    subplot(326);
        plot(Time,RPM(:,6));
        legend({'$\tau_6$',},'interpreter','latex');
    sgtitle('Measured RPM');

figure(5);clf;
    subplot(321);
        hold on;
        plot(Time,RPM2Tau(:,1));
        plot(Time,CmdTau(:,1));
        legend({'$\hat{\tau}_1$','$\tau_1$'},'interpreter','latex');
        hold off;
    subplot(322);
        hold on;
        plot(Time,RPM2Tau(:,2));
        plot(Time,CmdTau(:,2));
        legend({'$\hat{\tau}_2$','$\tau_2$'},'interpreter','latex');
        hold off;
    subplot(323);
        hold on;
        plot(Time,RPM2Tau(:,3));
        plot(Time,CmdTau(:,3));
        legend({'$\hat{\tau}_3$','$\tau_3$'},'interpreter','latex');
        hold off;
    subplot(324);
        hold on;
        plot(Time,RPM2Tau(:,4));
        plot(Time,CmdTau(:,4));
        legend({'$\hat{\tau}_4$','$\tau_4$'},'interpreter','latex');
        hold off;
    subplot(325);
        hold on;
        plot(Time,RPM2Tau(:,5));
        plot(Time,CmdTau(:,5));
        legend({'$\hat{\tau}_5$','$\tau_5$'},'interpreter','latex');
        hold off;
    subplot(326);
        hold on;
        plot(Time,RPM2Tau(:,6));
        plot(Time,CmdTau(:,6));
        legend({'$\hat{\tau}_6$','$\tau_6$'},'interpreter','latex');
        hold off;
    sgtitle('Tau');

start = 200; %buffer to to not make the program crash
stop = length(Time);


%creating trajectory and cage net from raw data:
XYZ_coord = [EtaHat(:,1),EtaHat(:,2),EtaHat(:,3)];
DVL_beam = [Data.DVL_6,Data.DVL_7,Data.DVL_8,Data.DVL_9];
DVL_beam = DVL_beam.* DVLAvail;
DVL_Left_Right = [(DVL_beam(:,2)+DVL_beam(:,3))/2,(DVL_beam(:,1)+DVL_beam(:,4))/2];


%TF_left and TF_right gives 1 if the element is an outlier, 0 if it is not.
[DVL_r_rmOutliers_left,TF_left] = rmoutliers(DVL_Left_Right(:,1),'movmedian',25,'SamplePoints',Time);
[DVL_r_rmOutliers_right,TF_right] = rmoutliers(DVL_Left_Right(:,2),'movmedian',25,'SamplePoints',Time);



%want to map the relative measurements to the global reference frame
DVL_beam_orientation_deg = 25;
DVL_beam_orientation_rad = 2*pi*DVL_beam_orientation_deg/360;
phi = DVL_beam_orientation_rad;

R_b_left = [cos(-phi),-sin(-phi),0;sin(-phi),cos(-phi),0;0,0,1];
R_b_right = [cos(phi),-sin(phi),0;sin(phi),cos(phi),0;0,0,1];
V3zero = [0;0;0];
T_b_left = [R_b_left V3zero;0 0 0 1];
T_b_right = [R_b_right V3zero;0 0 0 1];

XYZ_lm_KF = zeros((stop-start)*2,3);
XYZ_lm_DVL = XYZ_lm_KF;


delta_b_x = [-0.01;0;0];


figure('units','normalized','outerposition',[0 0 1 1]);
plot3(EtaHat(:,1),EtaHat(:,2),EtaHat(:,3))
xlabel('x')
ylabel('y')
zlabel('z')
grid on;
hold on;
pbaspect([1.8,1.4,1])
for i=start:stop 
    %KF : pose and rotation matrix
    pos_ROV_KF = [EtaHat(i,1),EtaHat(i,2),EtaHat(i,3)];
    yaw_est_KF = EtaHat(i,4);
    R_w_b_KF = [cos(yaw_est_KF),-sin(yaw_est_KF),0;
        sin(yaw_est_KF),cos(yaw_est_KF),0;
        0,0,1];

    %finding path and landmarks(lm)
    if DVLAvail(i) == 1 && TF_left(i) == 0 && TF_left(i) == TF_right(i)
        %create transformation matrix for kalman and DVL measurements
        T_w_b_KF = [R_w_b_KF pos_ROV_KF.';0 0 0 1];
        %T_w_b_DVL = [R_w_b_DVL pos_ROV_DVL.';0 0 0 1];
        
        %projection of range measurements for kalman and DVL measurements
        l_p_KF = T_w_b_KF*T_b_left*[DVL_Left_Right(i,1);0;0;1];
        r_p_KF = T_w_b_KF*T_b_right*[DVL_Left_Right(i,2);0;0;1];
        
        %store range projections
        XYZ_lm_KF((i-1)*2+1,:) = l_p_KF(1:3).';
        XYZ_lm_KF((i-1)*2+2,:) = r_p_KF(1:3).';
    end
    set(gca, 'Zdir', 'reverse')
    if mod(i,500)==0
        disp(i)
    end
end

folder = '/home/kyrre/Bilder/bilderMasteroppgave/Resultater/MatlabPlot/';

figure(9)
title('2D-view')
plot3(EtaHat(start:stop,1),EtaHat(start:stop,2),EtaHat(start:stop,3))
hold on
plot3(XYZ_lm_KF(1:2:(end-1),1),XYZ_lm_KF(1:2:(end-1),2),XYZ_lm_KF(1:2:(end-1),3),'.','Color','r')
plot3(XYZ_lm_KF(2:2:end,1),XYZ_lm_KF(2:2:end,2),XYZ_lm_KF(2:2:end,3),'.','Color','r')
grid on;
view(0,90)
xlabel('N')
ylabel('E')
zlabel('D')

set(gca, 'Zdir', 'reverse')
%saveas(gcf,'/home/kyrre/Bilder/bilderMasteroppgave/Resultater/MatlabPlot/2DView_shared2_AutonomousFeatures_run2.png')
%}

figure(10)
title('3D-view')
plot3(EtaHat(start:stop,1),EtaHat(start:stop,2),EtaHat(start:stop,3))
hold on
plot3(XYZ_lm_KF(1:2:(end-1),1),XYZ_lm_KF(1:2:(end-1),2),XYZ_lm_KF(1:2:(end-1),3),'.','Color','r')
plot3(XYZ_lm_KF(2:2:end,1),XYZ_lm_KF(2:2:end,2),XYZ_lm_KF(2:2:end,3),'.','Color','r')
grid on;
view(2)
xlabel('N')
ylabel('E')
zlabel('D')

set(gca, 'Zdir', 'reverse')
view(-45,55)
%saveas(gcf,'/home/kyrre/Bilder/bilderMasteroppgave/Resultater/MatlabPlot/3DView_shared2_AutonomousFeatures_run2.png')
