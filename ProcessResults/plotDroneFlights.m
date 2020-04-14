function [] = plotDroneFlights(out, simkey, varargin)
%plotDroneFlights plots the flight of all Drones in the simulation output
%   out: Simulation output
%   simkey: structure indicating index of position and velocity in the
%   output
%   varargin(1): list of colors for the drones in seperate formations

%% Check if Simulation Output Exists
if exist('out')==0
  disp('Init Complete')
  return
else
  disp('plotting Results')
end

%% Create list of Drones in Sim Output
sim_fields = out.who;
listOfDrones = [];
for i = 1:length(sim_fields)
  if strncmp(sim_fields{i},'Drone',5)
    listOfDrones = [listOfDrones, out.get(sim_fields{i})];
    time = out.get(sim_fields{i}).Time;
    data = squeeze(out.get(sim_fields{i}).Data);
    listOfDrones(length(listOfDrones)).Name = sim_fields{i};
    disp(["Found "+sim_fields{i}])
  end
end
nDrones_total = length(listOfDrones)
time = round(out.get("tout"),3);

%% Set color codes of drones
if length(varargin)
  colors = varargin{1};
  nFormations = length(colors);
  if mod(nDrones_total, nFormations) == 0
    form = 1;
    for i = 1:nDrones_total
      droneColors(i) = string(colors(form));
      if mod(i,(nDrones_total/nFormations)) == 0
        form = form+1;
      end
    end
  else
    error("There must be the same number of Colors as there are formations")
  end
else
  droneColors = []
  for i = 1:nDrones_total
    droneColors = [droneColors, string('k')];
  end
end

%% Plot
% create plot figure and arena
h_areaFig = figure("Name","Drone Arena","Position",[50,50,1250,750]);
hold on; grid on; axis equal; view(-25,20);
h_arenaPlt = plot_arena(6,4,4);

% create list of quadrotors
Quads = [];
h_quads= [];
for i = 1:nDrones_total
  Quads = [Quads,Quadcopter()];
  Quads(i).set_position(listOfDrones(i).Data(1,simkey.p));
  Quads(i).set_orientation(listOfDrones(i).Data(1,simkey.q));
  Quads(i).set_color(char(droneColors(i)));
  h_quads= [h_quads,Quads(i).plotQuadcopter()];
end

% animate simulation
 pause(0.001);
t_step = 0.05;
for t = 0:t_step:max(time)
  delete(h_quads);
  h_quads = [];
  idx = round(find(time==round(t,4)));
  % loop through quads  
  for i = 1:nDrones_total
    Quads(i).set_position(listOfDrones(i).Data(idx, simkey.p));
    Quads(i).set_orientation(listOfDrones(i).Data(idx, simkey.q));
    h_quads= [h_quads,Quads(i).plotQuadcopter()];
  end
  pause(t_step/100);
end



%% END OF PLOTTING SCRIPT
disp("Plotting Script Finished")

end

