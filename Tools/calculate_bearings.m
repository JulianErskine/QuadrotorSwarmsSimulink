function B = calculate_bearings(E,P,Y)
% calculate_bearings(E,P,Y)
%   Calculates the matrix of bearings for a given edge matrix E, position
%   matrix P, and vector Y
%   Edge vector is E = [Di, Dj
%                      ...
%                       Dk, Dl]
%   Position vector is P= [px1 ... pxn
%                          py1 ... pyn
%                          pz1 ... pzn]
%   Yaw vector is Y = [y1 ... yn]
%
%   Bearing matrix is B = [BxE1     BxEm
%                          ByE1 ... BxEm
%                          BzE1     BxEm]

[m,~] = size(E);
B = zeros(3,m);

% for each edge
for i=1:m
  
  % drone i looks at drone j in world frame
  di = E(i,1);
  dj = E(i,2);
  
  % rotation of drone i in world frame
  ci = cos(Y(di));
  si = sin(Y(di));
  Ri = [ ci, si, 0;
    -si, ci, 0;
    0,  0, 1];
  
  % bearing of drone j in drone i frame
  pij = P(:,dj)-P(:,di);
  if(norm(pij)<0.001)
    disp(("Drone"+di+ " and Drone"+dj+" collided"))
  end
  
  b = Ri'*pij/norm(pij);
  B(:,i) = b;
end


end


