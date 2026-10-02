%% MAIN File
%  Franka Panda Trajectory Analysis - Run this file to execute all analysis scripts in order

clc; clear; close all;

CPRINTF_PATH  = 'C:\Users\nravikumar\Desktop\MATLAB SCRIPTS\Libraries\Cprintf';
addpath(genpath(CPRINTF_PATH));

% % Start capturing command window
% diary_file = fullfile('C:\Users\nravikumar\Desktop\ANALYSIS 2', 'Panda_Analysis.txt');
% diary(diary_file);
% diary on;

fprintf('FRANKA TRAJECTORY ANALYSIS:\n');

run('LOAD_DATA.m');         % loads CSV + YAML 
run('EE_ERROR_ANALYSIS.m'); % Analysis of EE position, orientation, homing errors
run('JOINT_ANALYSIS.m');    % torque, velocity, jerk, range of motion
run('VISUALIZE_ROBOT_LOG_DATA.m');  % Visualization of the Log datas 
run('VISUALIZE_DATA.m');    % Visualization of the Analysis datas 
run('EE_FORCE_ANALYSIS.m'); % Analysis of Force EE
run('EXCEL_EXPORT.m');      % Exporting the Analysis results
run('SY_EVALUATION.m');     % Additional visualization for detailed evaluation

% diary off;


