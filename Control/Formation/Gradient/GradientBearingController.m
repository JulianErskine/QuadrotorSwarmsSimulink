function [out] = GradientBearingController(B, Bd, droneStates, nDrones, edges)
%GradientBearingController sets the control velocity and yawrate of each drone.
% 
%   This function implements the decentralized bearing formation control from
%   Schiano F, Franchi A, Zelazo D, and Robuffo Giordano P, "A Rigidity-Based De-centralized 
%   Bearing Formation Controller for Groups of Quadrotor UAVs", IROS 2016.
%   
%

kc = 1;
ky = 1;

u = zeros(3,nDrones);
omega = zeros(1,nDrones);
S = [0 -1 0; 1 0 0; 0 0 0];

drones = struct2cell(droneStates);

% calculate relative yaws
for i = 1:nDrones
  [Ri,y] = build_flat_rotation(drones{i}.q);
  yaw(i) = y;
  R(:,:,i) = Ri;
end
for E = 1:length(edges)
  iYj(E) = yaw(edges(E,2)) - yaw(edges(E,1));
end

% Flatten bearings
for i = 1:length(B)
  fR0 = squeeze(R(:,:,edges(i,1)))';
  oRb = quat2rotm(drones{edges(i,1)}.q');
  B(:,i) = fR0 * oRb * B(:,i);
end

for di = 1:nDrones
  for E = 1:length(edges)
    if di == edges(E,1) %if the drone is observing another
      dj = edges(E,2);
      Bij = B(:,E);
      Bijd = Bd(:,E);
      P = eye(3)-Bij*Bij';
      u(:,di) = u(:,di) - kc*P*Bijd;
      omega(di) = omega(di) + ky*Bij'*S*Bijd;
    elseif di == edges(E,2) % if another drone is observing it
      dj = edges(E,1);
      Bji = B(:,E);
      Bjid = Bd(:,E);
      P = eye(3)-Bji*Bji';
      Rij = Rzmat(iYj(E));
      u(:,di) = u(:,di) + kc*Rij*P*Bjid;
    else % id drone i is not part of the edge
      %do nothing
    end % go to next edges
  end %go to next drone i
end

out = reshape([u;omega],[4*nDrones,1]);
end


function [R, yaw] = build_flat_rotation(q)
R_full = quat2rotm(q');
z = [0;0;1];

if(abs(R_full(3,1))<abs(R_full(3,2)))
  x_proj = [R_full(1:2,1);0];
  y_proj = cross(z,x_proj);
else
  y_proj = [R_full(1:2,2);0];
  x_proj = cross(y_proj,z);
end
R = [x_proj,y_proj,z];
yaw = atan2(x_proj(2),x_proj(1));
end
