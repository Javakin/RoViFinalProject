clear all
close all
clc

% Import formatted data
load('data.mat');


%% Robotics data Treatment

% 1 Point - Error
% Errors
plotGraphError(SlowMarker50Error_1P, MediMarker100Error_1P, FastMarker200Error_1P_1, 2,3,3, 'Single Point Tracking Error');

%% 3 Point - Pose
plotGraphPose(SlowMarker50Pose_1P, MediMarker100Pose_1P, FastMarker150Pose_1P, 2,3,3,'Single Point');


%% 3 Point - Error
plotGraphError(SlowMarker50Error_3P, MediMarker100Error_3P, FastMarker200Error_3P_1, 2,2,1, 'Three Point Tracking Error');

% 3 Point - Pose
plotGraphPose(SlowMarker50Pose_3P, MediMarker100Pose_3P, FastMarker150Pose_3P, 2,3,3,'Three Point');

%% Vision data Treatment 

% Errors
plotGraphError(SlowMarker450Error_Vis, MediMarker500Error_Vis, FastMarker550Error_Vis, 4,2,4, 'Visual Servering - Tracking Error');

% Positional and velocity limits
plotGraphLim(SlowMarker450Limits_Vis, MediMarker500Limits_Vis, FastMarker550Limits_Vis, 1 , 3, 1, 'Visual Servering');



