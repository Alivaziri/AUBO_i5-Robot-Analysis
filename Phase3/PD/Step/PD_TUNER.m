clc; clear; close all
%% creating slTuner and configuring it
% creat slTuner interface
TunedBlocks = {'PD1','PD2','PD3','PD4','PD5','PD6'};
ST0 = slTuner('PD_FINALTC2',TunedBlocks);

% Mark outputs of PD blocks as plant inputs
addPoint(ST0,TunedBlocks)

% Mark joint angles as plant outputs
addPoint(ST0,'plant/THETA');

% Mark refrences signals
RefSignals = {...
    'PD_FINALTC2/Signal Builder/1','PD_FINALTC2/Signal Builder/2',...
    'PD_FINALTC2/Signal Builder/3' ,'PD_FINALTC2/Signal Builder/4',...
    'PD_FINALTC2/Signal Builder/5','PD_FINALTC2/Signal Builder/6'};
addPoint(ST0,RefSignals)

%% Defining Input and Outputsand Tuning The System
Controls = TunedBlocks;      % actuator commands
Measurements = 'PD_FINALTC2/plant/Integrator1/1[THETA]';    % joint angle measurement
options = looptuneOptions('RandomStart',100','UseParallel',true);
TR = TuningGoal.StepTracking(RefSignals,Measurements,2,0);
ST1 = looptune(ST0,Controls,Measurements,TR,options);

%% Update PD Blocks
writeBlockValue(ST1)