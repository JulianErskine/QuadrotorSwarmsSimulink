function [q] = eul2quat(rx, ry, rz, order)
%EUL2QUAT Summary of this function goes here
%   Detailed explanation goes here
Qx=[cos(rx/2) sin(rx/2)  0          0];
Qy=[cos(ry/2) 0          sin(ry/2)  0];
Qz=[cos(rz/2) 0          0          sin(rz/2)];
if(strcmpi(order,'xyz')==1)  
%    disp('order is x-->y-->z');
    Q1=multiplyQuaternions(Qy,Qz);  
    Q2=multiplyQuaternions(Qx,Q1);  
    
elseif(strcmpi(order,'xzy')==1)  
 %   disp('order is x-->z-->y');
    Q1=multiplyQuaternions(Qz,Qy);  
    Q2=multiplyQuaternions(Qx,Q1);
    
elseif(strcmpi(order,'yxz')==1)
  %  disp('order is y-->x-->z');
    Q1=multiplyQuaternions(Qx,Qz);  
    Q2=multiplyQuaternions(Qy,Q1);
    
elseif(strcmpi(order,'yzx')==1)
   % disp('order is y-->z-->x');
    Q1=multiplyQuaternions(Qz,Qx);  
    Q2=multiplyQuaternions(Qy,Q1);
    
elseif(strcmpi(order,'zxy')==1)
    %disp('order is z-->x-->y');
    Q1=multiplyQuaternions(Qx,Qy);  
    Q2=multiplyQuaternions(Qz,Q1);
    
elseif(strcmpi(order,'zyx')==1) % same as matlab quatdemo
    %disp('order is z-->y-->x');
    Q1=multiplyQuaternions(Qy,Qx);  
    Q2=multiplyQuaternions(Qz,Q1); 
else
    error('could not recognized')    
end
    w=Q2(1);
    x=Q2(2);
    y=Q2(3);
    z=Q2(4);
    q = [w;x;y;z];
end

