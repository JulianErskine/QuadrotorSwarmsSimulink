function q = build_rotation(z,yaw)
z = z/norm(z);
x = [cos(yaw);sin(yaw);0];
x = x/norm(x);
y = cross(z,x);
y = y/norm(y);
x = cross(y,z);
x = x/norm(x);
R = [x,y,z];
q = rotm2quat(R).';
end
