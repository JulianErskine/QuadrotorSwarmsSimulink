%% SingleQuadInit.m
% Initialisation Script for the simulation of a single quadrotor.
%
% Sets up simulation parameters, runs ErskineSingleDrone.slx simulink file
% and simulates the drone flight. 
%
% The control modes:
%   1: position-yaw
%   2: velocity-yawrate, 
%   3: acceleration-yawraterate
%   4: attitude-thrust
%   5: thrust-bodyrate
%   6: motor speeds
%
% The control modes can be selected by double clicking on the Quadrotor
% simulink block.
%
% The "DroneID" and "idx_offset" parameters of the simulink block are used for
% multi-drone simulations, and can be set to 1 and 0 respectively 
%
% Author: Julian Erskine
% Address: LS2N, Ecole Centrale de Nantes
% Email: julian.erskine@ls2n.fr
% Last revision: 17-03-2020


%% Setup workspace
close all; clear all; clc;
addpath(genpath(fileparts(which(mfilename))));

%% Simulation Params
disp("Initializing Sim Parameters")
sim_dt = 0.005
Tsim = 10

%% Setup Problem

% set parameters
drone_params = setDefaultDroneParams(1)
g = 9.81;


% Initial States  
p0 = 4*(rand(3, 1) - [0.5;0.5;0])
v0 = [0;0;0];
a0 = [0;0;0];
q0 = [1;0;0;0];
omega0 = [0;0;0];

% Desired states
desired_states = [3;1;3;1]

% plot Simkey
simkey.p = [1:3];
simkey.v = [4:6];
simkey.a = [7:9];
simkey.q = [10:13];

out = sim('ErskineSingleDrone');

plotDroneFlights(out,simkey);