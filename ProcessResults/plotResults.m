function [] = plotResults(out, Bd_ts, Rigid_ts)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%% Plot Bearing Trajectory

sim_fields = out.who
listOfControllers = [];
controllerNames =[]
for i = 1:length(sim_fields)
  if ~strncmp(sim_fields{i},'Drone',5) && ~strncmp(sim_fields{i},'tout',4)
    listOfControllers = [listOfControllers, out.get(sim_fields{i})];
    controllerNames = [ controllerNames, string(sim_fields{i})];
    disp(["Found "+sim_fields{i}])
  end
end
controllerNames
nControllers = length(listOfControllers)
time = round(out.get("tout"),3);

endidx = length(time)

if time(endidx) > Bd_ts.TimeInfo.End
  Bd_ts = addsample(Bd_ts,'Data',Bd_ts.Data(Bd_ts.TimeInfo.Length,:),'Time',time(endidx))
end
if time(endidx) > Rigid_ts.TimeInfo.End
  Rigid_ts = addsample(Rigid_ts,'Data',Rigid_ts.Data(Rigid_ts.TimeInfo.Length,:),'Time',time(endidx))
end

hResultsFig = figure('Name',"SimResults", 'Position',[200,200,600,400])
subplot(9,1,1:4); grid on; hold on;
ax = gca;
for i = 1:nControllers
  plot(listOfControllers(i).bearing_error)
end
legend(controllerNames)
title('Bearing Error')
ylabel('Bearing Error Magnitude')
ax.YAxis.Limits(1) = 0;
subplot(9,1,6:9); grid on; hold on;
ax = gca;
for i = 1:nControllers
  plot(Bd_ts)
end
ylabel('Desired Bearing Values')
yyaxis right
plot(Rigid_ts, 'r', 'linewidth',3)
ylabel('Rigidity Eigenvalue')
title('Desired Bearings')
xlabel("Time (s)")

ax.YAxis(2).Color = [1,0,0]
ax.YAxis(2).Limits = [0, round(max(Rigid_ts.Data)+0.05,1)]
% end of function
end

