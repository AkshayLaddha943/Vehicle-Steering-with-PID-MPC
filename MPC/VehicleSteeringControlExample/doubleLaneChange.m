function [Yref,Zref,Xref] = doubleLaneChange(Vx,time)
% Double lane change maneuver

% Vx   : reference longitudinal velocity
% Xref : X position of the reference trajectory
% Yref : Y position of the reference trajectory
% Zref : yaw angle of the reference trajectory
Xref = Vx*time;
z1 = (2.4/50)*(Xref-27.19)-1.2;
z2 = (2.4/43.9)*(Xref-56.46)-1.2;
Zref = atan(8.1*(1/cosh(z1))^2*(1.2/50)-11.4*(1/cosh(z2))^2*(1.2/43.9));
Yref = 8.1/2*(1+tanh(z1)) - 11.4/2*(1+tanh(z2));
end