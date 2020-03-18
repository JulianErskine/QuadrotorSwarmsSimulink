function [FormationStateBus] = generateFormationStateBus(nDrones)
%generateFormationStateBus(nDrones) generates the formation bus required
%for a formation of n drones
%   This function is a prerequisite of the ErskineMultiDroneFormation
%   Simulink model, and must be re-run any time the number of drones in the
%   formation changes.

FormationStateBus = Simulink.Bus;

for i = 1:nDrones
  tmp = Simulink.BusElement;
  tmp.Name = ["drone"+num2str(i)];
  tmp.DataType = "Bus: DroneStateBus";
  FormationStateBus.Elements(i) = tmp;
end


end

