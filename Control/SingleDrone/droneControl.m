function [thrust, moments] = droneControl(u, p, v, q, omega, control_type_param, drone_params, DroneID, g)
%droneControl is the main function used to control the drone in
%ErskineSingleDrone.slx
%
%   The modes which may be selected are:
%     1: Position/Yaw - F_W
%     2: Velocity/Yawrate - F_B
%     3: Acceleration/Yawrate - F_B
%     4: Thrust/Attitude - reference quaternion - F_W
%     5: Thrust/BodyRate - F_B
%     6: Thrust/Moments - F_B
%   Note that F_W (world) and F_B (body) indicate the frame of reference
%   for the input.
%
% u: the 4 (5 in the case of thrust/attitude mode) input to the controller
% p: the current measured drone position in F_W
% v: the current measured drone velocity in F_B
% q: the current attitude estimate of the drone in F_W
% omega: the current measured angular velocity of the drone in F_B
%
% control_type_parameter: integer used to select the control mode.
% drone_params: array of structures (see setDefaulDroneParams.m) containing
% the mechanical and control parameters of each drone.
% DroneID: Integer used to select which drone in the drone_params arrays.
% g: Magnitude of gravitational acceleration (assumed to be in the -z_W
% direction).
%
% The position control is a PD controller. The velocity, yawrate, and body rate 
% are P controllers. Translational control has saturated thrust along the
% desired acceleration vector (see documentation of saturateThrust.m)
%
% The attitude controls are PD based, and combine aspects of: 
% Brescianini D, Hehn M, and D'Andrea R. "Nonlinear Quadrocopter Attitude 
% Control Technical Report", ETHZ 2013. and 
% Fresk E, and Nikolakopoulos G. "Full Quaternion Based Attitude Control 
% for a Quadrotor", European Control Conference, 2013.

m = drone_params(DroneID).mechanical.m;
kp_attitude = drone_params(DroneID).control.attitude.kp;
kd_attitude = drone_params(DroneID).control.attitude.kd;

if control_type_param == 1
  % do position control
  % VALIDATED
  kp = drone_params(DroneID).control.position.kp;
  kd = drone_params(DroneID).control.position.kd;
  
  pd = u(1:3);
  yd = u(4);
  %[R,y] = build_flat_rotation(q);
  R = quat2rotm(q);
  ad = kp*(pd-p) + kd*(-R*v);

  f = saturateThrust(ad, drone_params(DroneID).control.limits.thrust.max, m, g);
  thrust = norm(f);
  zd = f/thrust;
  thrust = thrust * zd' * R(:,3);
  moments = fullAttitudeControl(zd,yd,q,omega,kp_attitude,kd_attitude);
  moments(3) = moments(3)/10;
  
  
elseif control_type_param == 2
  % do body velocity and yawrate control
  vd = u(1:3); % desired velocity in flat body frame
  yrd = u(4); % desired yawrate
  
  kp = drone_params(DroneID).control.velocity.kp;
  
  [R,y]= build_flat_rotation(q); % rotation of quadrotor in world frame
  v = quat2rotm(q')*v; % measured velocity in world frame
  v = inv(R)*v; % measured velocity in flat body frame
  ad = kp*(vd - v); % force vector in flat body frame
  fd = saturateThrust(ad,drone_params(DroneID).control.limits.thrust.max, m, g);
  fd = R*fd; %fd in world frame
  thrust = norm(fd);
  moments = fullAttitudeControl(fd,y,q,omega,kp_attitude,kd_attitude); % in world frame
  moments(3) =0.1*(moments(3) + kp_attitude*(yrd-omega(3)));
  
elseif control_type_param == 3
  % do body acceleration and yawrate control
  thrust = 0;
  
  
  
  
elseif control_type_param == 4
  % do thrust and attitude control
  % VALIDATED
  thrust = u(1);
  qd = u(2:5)/norm(u(2:5));
  moments = doAttitudeControl(qd,q,omega,kp_attitude, kd_attitude);
  
  
  
elseif control_type_param == 5
  % do thrust and body rate control
  % VALIDATED
  kp_attitude_rate = drone_params(DroneID).control.attitude_rate.kp;
  thrust = u(1);
  moments = kp_attitude_rate * (u(2:4)-omega);
  
  
elseif control_type_param == 6
  % do body frame thrust wrench control
  %VALIDATED
  thrust = u(1);
  moments = u(2:4);
  
else
  error("The quadrotor control type was not valid")
end

end


%% Auxiliary functions
function moment = doAttitudeControl(qd, q, omega, ka, ko)
%good with ka = 20, ko = 2
qerr = multiplyQuaternions(qd, [q(1);-q(2);-q(3);-q(4)]);
moment = ka*sign(qerr(1))*qerr(2:4) - ko*omega;
end


function moment = reducedAttitudeControl(zd,q,omega,ka,ko)
% point the quadrotor axis from currently at zc towards desrired zd
% normalize inputs
zd = zd/norm(zd);
Rc = quat2rotm(q');
zd = inv(Rc)*zd;
zc = [0;0;1];
dot_vectors = zc'*zd;
if dot_vectors<0.999999
  alpha = acos(dot_vectors)
else
  alpha = 0;
end
k = cross(zc,zd)/norm(cross(zc,zd));
qe = [cos(alpha/2); sin(alpha/2)*k];
qe = qe/norm(qe);
qcmd = multiplyQuaternions(q,qe);
moment = doAttitudeControl(qcmd,q,omega,ka,ko);
end


function moment = fullAttitudeControl(zd,yd,q,omega,ka,ko)
% point the quadrotor axis from currently at zc towards desrired zd
% K frame is flat rotation about z0
% L frame is rotated about new y_K by theta
% normalize inputs
zd = zd/norm(zd);

kzd = Rzmat(yd)*zd;
th_cmd = atan(kzd(1)/kzd(3));

lzd = Rymat(th_cmd)*kzd;
phi_cmd = atan2(-lzd(2), lzd(3));

qcmd = eul2quat(phi_cmd,th_cmd,yd,'zyx');
moment = doAttitudeControl(qcmd,q,omega,ka,ko);
end



function thrust_vector = saturateThrust(ad, fmax, m, g)
%% saturateThrust
% fmax^2 = m(g^2 + a^2) - 2m^2(g*a)*cos(th+pi/2)

if norm(m*(ad + [0;0;g])) > fmax
  % do thrust saturation
  th = atan2(sqrt(ad(1)^2 + ad(2)^2), ad(3));
  b = -2*m^2*g*cos(th + pi/2);
  c = (m*g)^2 - (fmax)^2;
  amax = (-b + sqrt(b*b - 4*c*m^2))/(2*m^2);
  factor = amax/norm(ad);
  thrust_vector = m*(factor*ad + [0;0;g]);
else
  thrust_vector = m*(ad + [0;0;g]);
end

if thrust_vector(3) < m*g/3
  % do downward thrust saturation
end
end
