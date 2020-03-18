function [h] = plot_arena(lx, ly, lz)
%plot_arena Plots a rectangular flight environment
% The flight arena spans x=[]
%   Detailed explanation goes here
h(1)=plot3([-lx/2,lx/2],[-ly/2,-ly/2],[0,0],'k','linewidth',2);
h(2)=plot3([-lx/2,lx/2],[ly/2,ly/2],[0,0],'k','linewidth',2);
h(3)=plot3([-lx/2,lx/2],[-ly/2,-ly/2],[lz,lz],'k','linewidth',2);
h(4)=plot3([-lx/2,lx/2],[ly/2,ly/2],[lz,lz],'k','linewidth',2);

h(5)=plot3([-lx/2,-lx/2],[-ly/2,ly/2],[0,0],'k','linewidth',2);
h(6)=plot3([lx/2,lx/2],[-ly/2,ly/2],[0,0],'k','linewidth',2);
h(7)=plot3([-lx/2,-lx/2],[-ly/2,ly/2],[lz,lz],'k','linewidth',2);
h(8)=plot3([lx/2,lx/2],[-ly/2,ly/2],[lz,lz],'k','linewidth',2);

h(9)=plot3([-lx/2,-lx/2],[-ly/2,-ly/2],[0,lz],'k','linewidth',2);
h(10)=plot3([lx/2,lx/2],[-ly/2,-ly/2],[0,lz],'k','linewidth',2);
h(11)=plot3([-lx/2,-lx/2],[ly/2,ly/2],[0,lz],'k','linewidth',2);
h(12)=plot3([lx/2,lx/2],[ly/2,ly/2],[0,lz],'k','linewidth',2);

h(13)=plot3([0,1],[0,0],[0,0],'r','linewidth',4);
h(14)=plot3([0,0],[0,1],[0,0],'g','linewidth',4);
h(15)=plot3([0,0],[0,0],[0,1],'b','linewidth',4);

h(16)=surf([lx/2,lx/2,-lx/2,-lx/2],[-ly/2,ly/2,-ly/2,ly/2],zeros(4),...
  'FaceAlpha',0.25,'FaceColor','k');
end

