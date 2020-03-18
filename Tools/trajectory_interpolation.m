function [Qinterp] = trajectory_interpolation(Q, T, dt, type)
%trajectory_interpolation(Q,T) Interpolates the states Q for time T
%   Q : [n,pxm]matrix where nxm is the number of states
%   T : [1,p] vector where p is the number of time waypoints
%   dt: timestep
%   type: 'linear', 'cubic', or 'fifth'

% find dimensions
 p = length(T);
 [n,pm] = size(Q);
 m = pm/p;
 
 % time vector
 Ts = min(T):dt:max(T);
 
 % initial loop conditions
 seg = 1;
 idx = 1;
 Qa = Q(:,1:m);
 Qb = Q(:,m+1:2*m);
 ta = T(1);
 tb = T(2);

%for every time step
for i = 1:length(Ts)
  % if a new segment
  if(Ts(i)>T(seg+1))
    seg = seg+1;
    idx = idx + m;
    Qa = Q(:,idx:idx+m-1);
    Qb = Q(:,idx+m:idx+2*m-1);
    ta = T(seg);
    tb = T(seg+1);
  end
  % stop if at end of segment
  alpha = (Ts(i)-ta)/(tb-ta);
  if (alpha > 1)
    alpha=1;
  end
  % choose interp method
  if strcmp(type,'step')
    P = 1;
  elseif strcmp(type,'linear')
    P = alpha;
  elseif strcmp(type,'cubic')
    P = 3*alpha^2 - 2*alpha^3;
  elseif strcmp(type,'fifth')
    P = 10*alpha^3 - 15*alpha^4 + 6*alpha^5;
  end

  Qnow = Qa + P*(Qb - Qa);
  Qt(i,:) = reshape(Qnow,[1,n*m]);
end

Qinterp = timeseries(Qt,Ts);



end


