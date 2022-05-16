%% Dataset batch2 recording #3
clear all
close all
addpath("MAT/")
load("ReadData_Batch3_Second.mat")
FSampling_IMU = cast(FSampling_IMU,'double');
FSampling_GPS = cast(FSampling_GPS,'double');

% Batch 3 Second
CalibSamplesIMU = 2920:7600; %samples
CalibSamplesGPS = 56:190; %seconds
MotionSamplesIMU = 8600:43480; %samples (Flat part of second stop)
MotionSamplesGPS = 215:1087; %seconds

OmegaZ = lowpass(OmegaZ,0.5,40,"ImpulseResponse",'iir','Steepness',0.9);
ThetaZ = lowpass(ThetaZ,0.5,40,"ImpulseResponse",'iir','Steepness',0.9);
AlphaX_Motion = lowpass(AlphaX(MotionSamplesIMU),0.5,40,"ImpulseResponse",'iir','Steepness',0.9);
AlphaY_Motion = lowpass(AlphaY(MotionSamplesIMU),0.5,40,"ImpulseResponse",'iir','Steepness',0.9);

ThetaZ_Motion = ThetaZ(MotionSamplesIMU);

plot(AlphaX(MotionSamplesIMU), "LineStyle","-.")
hold on
plot(AlphaX_Motion, "LineWidth",1)
legend("Unfiltered", "Filtered")
title(" Forward Acceleration Filtering")
xlabel("Sample (@40Hz)")
ylabel("Acceleration (m.s^{-2})")
grid on

buttonpress(waitforbuttonpress)


%% Part A) GEOPLOT
geoplot(LAT,LON,'g-*')
buttonpress(waitforbuttonpress)
clear SatNum
%% Part B1) YAW Estimation
%% Magnotemeter Calibration
% [1]http://www.brokking.net/YMFC-32/YMFC-32_document_1.pdf
MagCalData = [MagX(CalibSamplesIMU)', MagY(CalibSamplesIMU)', MagZ(CalibSamplesIMU)'];
[A,b,expmfs] = magcal(MagCalData)
CalMagData = (MagCalData-b)*A;

figure("Name", "Magnetometer Plots (Roll Vs Pitch Axes)")
hold on

scatter(MagX(CalibSamplesIMU), MagY(CalibSamplesIMU), 1, "red", "filled", "MarkerEdgeColor",'red', "MarkerFaceAlpha","0.5")
xlabel("Field Strength_{X} (Gauss)");
ylabel("Field Strength_{Y} (Gauss)");

scatter(CalMagData(:,1) ,CalMagData(:,2), 1, "blue", "filled", "MarkerEdgeColor",'blue',"MarkerFaceAlpha","0.5")
title("Compass Calibration")
xlabel("Field Strength (X Axis) (Gauss)")
ylabel("Field Strength (Y Axis) (Gauss)")
grid on
axis equal
legend("Uncalibrated", "Calibrated")
%===========================================================================
% CHANGE THESE AFTER CALIBRATION
MagMotionData = [MagX', MagY', MagZ'];
AllMagData_Cal = (MagMotionData-b)*A; % Corrected Magnetometer Data for complete dataset


MagXCal = AllMagData_Cal(:,1);
MagYCal = AllMagData_Cal(:,2);

buttonpress(waitforbuttonpress) 

%% Yaw Rate integration and Complimentary Filter (Calibration time)
%%
% [1] https://robotics.stackexchange.com/questions/1717/how-to-determine-the-parameter-of-a-complementary-filter
% [2] https://drive.google.com/file/d/0B9rLLz1XQKmaLVJLSkRwMTU0b0E/view?resourcekey=0-oUq7ThstZRP9gGOzXQz9ZA

% Magnetometer yaw rotation angle (radians)
MagYaw = arrayfun(@(y,x)atan2d(-y, x), MagYCal, MagXCal)'; %Transpose needed to match dimensions

magBias =MagYaw(1)-rad2deg(ThetaZ(1));
MagYaw = wrapTo180(MagYaw-magBias);
ThetaZd = rad2deg(ThetaZ);

% gyro yaw rotation angle
cumYawGyro = wrapTo180(rad2deg(cumtrapz(1/40,OmegaZ))+ThetaZd(1));

% Complimentary Filter (DECIDE TIME CONSTANT)
% 1) VN100 in-run bias (Worst case): 5-7 degrees/hour => 0.1167 degrees per second
% Proposed: Take time constant as half of this: Tc => 0.06 Ref:[2]
% 2) From Tests on bias instability coefficients: zero slope line at 3.4 samples
% Consider: Tc => 3.4/40 = 0.0  850
Tc = 0.06;
alpha = (Tc/(Tc+(1/FSampling_IMU))); % Filter coefficient with a time constant Tc

CompFilterHandle  = @(LPFInp,HPFInp,Tctera)(alpha.*LPFInp)+((1-alpha).*HPFInp);
YawCompFilter     = CompFilterHandle(MagYaw, cumYawGyro,alpha);

figure("Name", "Yaw Rate integration")

subplot(4,1,1)
plot(MagYaw, 'Color',[0.9, 0.8,0.1], 'LineWidth',2);
title("Magnetometer Yaw");
xlabel("Sample (@40Hz)");
ylabel("Yaw Angle (\circ)");

subplot(4,1,2)
plot(cumYawGyro,'Color','red','LineWidth',2)
title("Yaw from Yaw rate integration (Gyro)");
xlabel("Sample (@40Hz)");
ylabel("Yaw Angle (\circ)");

subplot(4,1,3)
plot(YawCompFilter, 'Color','blue','LineWidth',2)
title("Complimentary Filter");
xlabel("Sample (@40Hz)");
ylabel("Yaw Angle (\circ)");

subplot(4,1,4)
plot(ThetaZd, 'Color','green','LineWidth',2);
title("IMU Yaw");
xlabel("Sample (@40Hz)")
ylabel("Yaw Angle (\circ)")

buttonpress(waitforbuttonpress)

%% Part B2) Estimate forward Velocity linear acceleration
%% Windowing for locating locally stagnant regions
% FROM LAB 3: Bias instability ensemble size: X = 83 (0.0004) Y = 212 (0.0005)
% Accel in-run bias: < 0.04 mg < 0.4 m.s^-2


% Look for tolerance constrained acceleration values to define stop points
clf
%Keep looking at last 5 seconds of data and define a band to say
%acceleration is stopped. Rebias the whole data again at this time
bias = 0;
flag = [];
windowSize    = 700;
window = AlphaX_Motion(1:windowSize);
tolerance     = 0.07;
AlphaX_MotionPrior = AlphaX_Motion;
plot(AlphaX_Motion)
legend("Original Alpha_X")


for i = 1+windowSize:length(AlphaX_Motion)
    if(std(window)<tolerance)
        disp("Window standard deviation:")
        std(window)
        flag = [flag, 1]; %#ok<*AGROW>
    else
        flag = [flag,-1];
    end
     window(1:end-1) = window(2:end);
    window(end) = AlphaX_Motion(i);
end

TrueEdge = strfind(flag,[1 -1]);
FalseEdge = [1,strfind(flag,[-1 1])+1];
plot(flag)
biases = [];
for i = 1:length(TrueEdge)
    biases = [biases, mean(AlphaX_Motion(FalseEdge(i):TrueEdge(i)))];
end

for i = 1:length(TrueEdge)-1
    AlphaX_Motion(FalseEdge(i):FalseEdge(i+1)) = AlphaX_Motion(FalseEdge(i):FalseEdge(i+1))-biases(i);
end

VelX_Raw = cumtrapz(1/40, AlphaX(MotionSamplesIMU));
VelX_Adjusted = cumtrapz(1/40, AlphaX_Motion);

subplot(2,1,1)
hold on
plot(AlphaX_MotionPrior)
plot(AlphaX_Motion, "LineWidth",2)
plot(flag, 'LineStyle',"-.")
title(" Linear Acceleration X Axis")
legend("Unadjusted", "Bias Removed", "Locally stagnant regions")
xlabel("Sample (@40Hz)")
ylabel("Acceleration (m.s^{-2})")
grid on
grid minor

subplot(2,1,2)
hold on
plot(VelX_Raw)
plot(VelX_Adjusted)
title("Integrated Velocity Profile X Axis")
legend("Non-ajusted", "Adjusted")
xlabel("Sample (@40Hz)")
ylabel("Velocity (m.s^{-1})")
grid on
grid minor

buttonpress(waitforbuttonpress)

%% Estimate forward velocity through GPS Measurements
%852 seconds before it goes haywire

LATMotion = LAT(MotionSamplesGPS);
LONMotion = LON(MotionSamplesGPS);
UTME_Motion = UTME(MotionSamplesGPS);
UTMN_Motion = UTMN(MotionSamplesGPS);

TimeVec =   MotionSamplesGPS';              % Sampling Times
VelGPS    = GPS_DistanceCalc(LATMotion,LONMotion, TimeVec);

figure("Name", " Velocity integrated from GPS data (Phase 2)")
plot(MotionSamplesGPS(2:end),VelGPS)

grid on
grid minor
title(" Velocity from GPS data")
xlabel("Timestamp from start of recording (s)")
ylabel("Velocity (m.s^{-1})")

buttonpress(waitforbuttonpress)

%% SECTION 3___________________________Dead Reckoning_______________________________
%% 3A) Plotting Adjusted Velocity and GPS Velocity
%Selecting samples before LOS
feasible_range_GPS = 1:473;
feasible_range_upsampled = 1:473*40;

% Interpolate GPS velocity samples to match IMU Velocity samples
VelGPS_upsampled = interp(VelGPS, 40);

figure("Name", "Velocity profiles GPS vs IMU")
hold on
plot(VelGPS_upsampled(feasible_range_upsampled))
plot(VelX_Adjusted(feasible_range_upsampled))
legend("GPS", "Integrated Acceleration")
title("Velocity profiles GPS vs IMU")
xlabel("Sample (@40Hz)")
ylabel("Forward Velocity (m.s^{-1})")
grid on
grid minor
buttonpress(waitforbuttonpress)
%% 3B) Resolving Acceleration into a N-E Coordinate system from the GPS plot

%Segment taken from loop 1 before GPS loses Signal
% Assume GPS north and calibrated ThetaZ = 0 are the same

VelX_Adjusted  = VelX_Adjusted(feasible_range_upsampled);

%Dead Reckining using Yaw angle from IMU, magnetometer and Complementary
%filter

%Resolving Linear velocity into NE directional components
VelN_IMU = []; VelE_IMU = [];
for i= 1:length(VelX_Adjusted)
    rot = [cos(ThetaZ_Motion(i)) -sin(ThetaZ_Motion(i));
            sin(ThetaZ_Motion(i)) cos(ThetaZ_Motion(i))]*[VelX_Adjusted(i);0];
    VelN_IMU = [VelN_IMU, rot(2)];     VelE_IMU = [VelE_IMU, rot(1)];
end

VelN_Mag = []; VelE_Mag = [];
for i= 1:length(VelX_Adjusted)
    rot = [cos(MagYaw(i)) -sin(MagYaw(i));
            sin(MagYaw(i)) cos(MagYaw(i))]*[VelX_Adjusted(i);0];
    VelN_Mag = [VelN_Mag, rot(2)];     VelE_Mag = [VelE_Mag, rot(1)];
end

YawCompFilter_Motion = YawCompFilter(MotionSamplesIMU);
YawCompFilter_Motion = YawCompFilter_Motion(feasible_range_upsampled);
VelN_CompFilter = []; VelE_CompFilter= [];
for i= 1:length(VelX_Adjusted)
    rot = [cos(MagYaw(i)) -sin(MagYaw(i));
            sin(MagYaw(i)) cos(MagYaw(i))]*[VelX_Adjusted(i);0];
    VelN_CompFilter = [VelN_CompFilter, rot(2)];    VelE_CompFilter = [VelE_CompFilter, rot(1)];
end

%Finding resolved displacements
DispN_IMU = cumtrapz(1/40, VelN_IMU);
DispE_IMU = cumtrapz(1/40,VelE_IMU);
DispN_Mag = cumtrapz(1/40, VelN_Mag);
DispE_Mag  = cumtrapz(1/40,VelE_Mag);
DispN_Comp = cumtrapz(1/40, VelN_CompFilter);
DispE_Comp = cumtrapz(1/40,VelE_CompFilter);


UTME_DeadReckoning = UTME_Motion(feasible_range_GPS)-UTME(MotionSamplesGPS(1));
UTMN_DeadReckoning = UTMN_Motion(feasible_range_GPS)-UTMN(MotionSamplesGPS(1));


% scatter((UTME(MotionSamplesGPS)-UTME(MotionSamplesGPS(1),...
%             UTMN(MotionSamplesGPS)-UTMN(MotionSamplesGPS(1)))))

scatter(UTME_DeadReckoning,UTMN_DeadReckoning, 5,'filled')
hold on
scatter(-downsample(DispE_IMU,40),downsample(DispN_IMU,40),5, 'filled')
scatter(-downsample(DispE_Mag,40),downsample(DispN_Mag,40),5,'filled')
scatter(-downsample(DispE_Comp,40),downsample(DispN_Comp,40),5,'filled')
grid on
grid minor
title("Trajectory comparison: IMU Dead Reckoning")
legend("GPS Data", "Bearing: IMU Yaw", "Bearing: Magnetometer Yaw","Complementary Filter Yaw", 'Location','southwest')
xlabel("Easting Position (m)")
ylabel("Northing Position (m)")


 buttonpress(waitforbuttonpress)
%% 3B) Estimating Lateral Acceleration omega.X_dot

OmegaZ_Motion = OmegaZ(MotionSamplesIMU);
OmegaZ_Motion = OmegaZ_Motion(feasible_range_upsampled);
AlphaY_Estimated = OmegaZ_Motion.*VelX_Adjusted(feasible_range_upsampled);

plot(AlphaY_Motion(feasible_range_upsampled))
hold on
plot(AlphaY_Estimated)
title("Lateral Acceleration Comparison")
legend("Observed", "Estimated")
xlabel("Sample (@40 Hz)")
ylabel("Lateral Acceleration (m.s^{-2})")
grid on
grid minor
 buttonpress(waitforbuttonpress)



%% 3C) Calculating displacement of IMU from body frame origin: X_c
yaw_acc = diff(OmegaZ_Motion)
xc= (AlphaY_Motion(1:feasible_range_upsampled(end-1))-AlphaY_Estimated(1:end-1))./yaw_acc;
fin = [];
for i=xc
    if abs(i)<3 %Setting tolerance for acceptable values
        fin = [fin, i];
    end
end
XC = mean(fin);

%% Variable Cleansing
clearvars fin xc
clearvars MagMotionData AllMagData_Cal MagCalData A b expmfs magBias CalMagData MagX MagY MagZ
clearvars CalibSamplesGPS CalibSamplesIMU Tc  CompFilterHandle alpha
clearvars AlphaX_MotionPrior VelX_Raw TrueEdge FalseEdge biases CalMagData
clearvars LATMotion LONMotion LAT LON
clearvars window windowSize  tolerance rot i
clearvars bias asn flag TimeVec i YawCompFilter VelGPS



%% Helper Functions
%% Calculating distance on Geoid

function [vel]= GPS_DistanceCalc(lat, lon, TimeVec) %#ok<*DEFNU>
LatLon = [lat',lon'];
dLatLon = diff(LatLon);
DistDeg = hypot(dLatLon(:,1), dLatLon(:,2));       % Distance (Degree Differences)
% d2m = 39983551.2/360;   %LAT
d2m = 29795778/360; %LON

% Degrees-To-Metres Conversion (Approximate)
DistMtr = DistDeg * d2m;                           % Distance (Metre Differences)
dTime = diff(TimeVec);                                % Sampling Time Differences
vel = DistMtr ./ dTime;      
end

%% Button Press for figure closing
function []  = buttonpress(buttonstate)
 if buttonstate == 1
     close all
 end
end

