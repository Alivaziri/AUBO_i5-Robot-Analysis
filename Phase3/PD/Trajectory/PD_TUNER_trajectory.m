clc; clear; close all
%% creating slTuner and configuring it
% creat slTuner interface
TunedBlocks = {'PD1','PD2','PD3','PD4','PD5','PD6'};
ST0 = slTuner('PD_trajectory',TunedBlocks);

% Mark outputs of PD blocks as plant inputs
addPoint(ST0,TunedBlocks)

% Mark joint angles as plant outputs
addPoint(ST0,'plant/THETA');

% Mark refrences signals
RefSignals = {...
    'PD_trajectory/MATLAB Function','PD_trajectory/MATLAB Function1',...
    'PD_trajectory/MATLAB Function2' ,'PD_trajectory/MATLAB Function3',...
    'PD_trajectory/MATLAB Function4','PD_trajectory/MATLAB Function5'};
addPoint(ST0,RefSignals)

%% Defining Input and Outputsand Tuning The System
Controls = TunedBlocks;      % actuator commands
Measurements = 'PD_trajectory/plant/Integrator1/1[THETA]';    % joint angle measurement
options = looptuneOptions('RandomStart',100','UseParallel',true);
TR = TuningGoal.Tracking(RefSignals,Measurements,0.05,0);
ST1 = looptune(ST0,Controls,Measurements,TR,options);

%% Update PD Blocks
writeBlockValue(ST1)