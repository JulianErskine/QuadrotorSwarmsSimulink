%% MultiQuadInit.m
% Initialisation Script for the simulation of a formation of multiple quadrotors.
%
% Sets up simulation parameters, runs ErskineMultiDroneFormation.slx simulink file
% and plots the drone flights. 
%
% The examples here use bearing formation control, but the structure should
% work equally well for other types of control.
%
%
% Author: Julian Erskine
% Address: LS2N, Ecole Centrale de Nantes
% Email: julian.erskine@ls2n.fr
% Last revision: 17-03-2020

%% Preliminaries
clear all
close all
clc
addpath(genpath(fileparts(which(mfilename))));


%% Simulation Params
disp("Initializing Sim Parameters")
sim_dt = 0.005
Tsim = 10
controller_rate = 25;

%% Setup Problem

% Number of drones if formation
nDrones = 3;

% set parameters
drone_params = setDefaultDroneParams(nDrones)
g = 9.81;


% Initial States  
p0 = 4*(rand(3,nDrones) - [0.5;0.5;0])
v0 = zeros(3, nDrones);
a0 = zeros(3, nDrones);
q0 = zeros(4,nDrones); q0(1,:) = ones(1,nDrones);
omega0 = zeros(3,nDrones);

% Sim Key
simkey.p = [1:3];
simkey.v = [4:6];
simkey.a = [7:9];
simkey.q = [10:13];


% camera noise
camera_noise.perp = 0.05;
camera_noise.long = 0.5;
camera_noise.long_bias = 0.5;

load('DroneStateBus');
FormationStateBus = generateFormationStateBus(nDrones);


%% Bearing Target
edges = [1,2; 
  1,3;
  2,1;
  2,3;
  3,1;
  3,2];

T = [0,1,Tsim-5];
pf = p0;

% create desired formation close to initial conditions:
%pf = p0(:,1:nDrones) + 2*(rand(3, nDrones)-[0.5; 0.5; 0]);
% create random desired formation
pf = 3*(rand(3, nDrones)-[0.5; 0.5; 0]);
P = [[p0(:,1:nDrones),p0(:,1:nDrones),pf];pi/3*(rand(1,length(T)*nDrones)-0.5)];

PTS = trajectory_interpolation(P,T,0.01,'step');

for i=1:length(PTS.time)
  Qt = reshape(PTS.data(i,:),[4,nDrones]);
  B(i,:) = reshape(calculate_bearings(edges,Qt(1:3,:),Qt(4,:)), [1,3*length(edges)]);
  [~,~,rigidity(i,1)] = build_bearing_rigidity_matrix(Qt(1:3,:),Qt(4,:),edges);
end

Bd_traj = timeseries(B,PTS.time);
Rigidity_traj = timeseries(rigidity,PTS.time);

% Create futur trajectory reference, used for MPC. Values needed to run,
% but non-essential otherwise.
N_p = 1
dt_p = 0.1
rate_mpc = 25
for i = 1:length(PTS.time)
  for j = 1:N_p
    t = Bd_traj.Time(i) + j*dt_p;
    if t >= Bd_traj.TimeInfo.End
      t = Bd_traj.TimeInfo.End;
    end
    idx = find(round(Bd_traj.Time,3) == round(t,3));
    Bforward(:,j,i) = Bd_traj.Data(idx,:)';
  end
end
Bd_future = timeseries(Bforward,PTS.Time)


%% Run simulation
disp("Running Simulation")
out = sim("ErskineMultiDroneFormation.slx");
disp("Plotting Simulation")
plotResults(out,Bd_traj,Rigidity_traj); pause(0.1);
plotDroneFlights(out,simkey, ['r','b']);
