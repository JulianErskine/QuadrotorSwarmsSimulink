function [Rflat,yaw] = build_flat_rotation(q)
%BUILD_FLAT_ROTATION Summary of this function goes here
%   Detailed explanation goes here
R_full = quat2rotm(q');
z = [0;0;1];

if(abs(R_full(3,1))<abs(R_full(3,2)))
  x_proj = [R_full(1:2,1);0];
  y_proj = cross(z,x_proj);
else
  y_proj = [R_full(1:2,2);0];
  x_proj = cross(y_proj,z);
end

x_proj = x_proj/norm(x_proj);
y_proj = y_proj/norm(y_proj);

Rflat = [x_proj,y_proj,z];
yaw = atan2(x_proj(2),x_proj(1));
end

