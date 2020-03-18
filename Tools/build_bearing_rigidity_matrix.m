function [BRM,varargout] = build_bearing_rigidity_matrix(P, Y, E, varargin)
% build_bearing_rigidity_matrix(P, Y, E, varargin)
%   Detailed explanation goes here

nEdges = length(E); %number of edges
Drones = unique(E); %list of drones
nDrones = length(unique(E)); %number of drones
BRM = zeros(3*nEdges, 4*nDrones); %bearing rigidity matrix
B = calculate_bearings(E,P,Y);  % bearings of each edges
S = [0,-1,0; 1,0,0; 0,0,0]; %skew matrix of z0

%build bearing rigidity matrix
for i = 1:nEdges
  di = E(i,1);
  dj = E(i,2);
  idx_i = find(Drones==di);
  idx_j = find(Drones==dj);
  
  ci = cos(Y(idx_i));
  si = sin(Y(idx_i));
  Ri = [ ci, si, 0;
    -si, ci, 0;
    0,  0, 1];
  
  dij = norm(P(:,idx_j)-P(:,idx_i));
  if(dij<0.01)
    warning('Drones are very close')
  end
  PP = eye(3) - B(:,i)*B(:,i)';
  
  first_row = 3*(i-1)+1;
  first_col_di = 3*(idx_i-1)+1;
  first_col_dj = 3*(idx_j-1)+1;
  BRM(first_row:first_row+2, first_col_di:first_col_di+2)= -PP*Ri'/dij;
  BRM(first_row:first_row+2, first_col_dj:first_col_dj+2)= PP*Ri'/dij;
  BRM(first_row:first_row+2, 3*nDrones+idx_i)= -S*B(:,i);
end
% calculate properties
sDOF = rank(null(BRM))-5;
eigens = sort(eig(BRM'*BRM));
r_eigen = round(eigens(6),8);
% set outputs
varargout{1} = sDOF;
varargout{2} = r_eigen;
end

