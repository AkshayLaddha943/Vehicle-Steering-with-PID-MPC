clc; clear all; close all;


% Here,
% The following vehicle parameter values are used in this code:
%
% * |m| is the total vehicle mass (kg)
% * |Iz| is the yaw moment of inertia of the vehicle (mNs^2).
% * |lf| is the longitudinal distance from the center of gravity to the
% front tires (m).
% * |lr| is the longitudinal distance from center of gravity to the rear
% tires (m).
% * |Cf| is the cornering stiffness of the front tires (N/rad).
% * |Cr| is the cornering stiffness of the rear tires (N/rad).
%
m = 1575;
I = 2875;
a = 1.2;
b = 1.6;
Cf = 19000;
Cr = 33000;

%%
% Specify the longitudinal velocity in m/s.
Vx = 15;

%%
% Specify a state-space model, |vehicle|, of the lateral vehicle dynamics.
A = [0,     1,                      0,      0,                          0,      0;    
     0,     -(Cf+Cr)/(m*Vx),         0,      -Vx-(Cf*a-Cr*b)/(m*Vx),       0,      0; 
     0,     0,                      0,      1,                          0,      0;
     0,     -(Cf*a-Cr*b)/(I*Vx),     0,      -(Cf*a^2+Cr*b^2)/(I*Vx),     0,      0;
     0,     1,                      Vx,      0,                          0,      0
     0,     -1,                     Vx,      0,                          0,      0   ];
 size(A)
 
B = [0,     Cf/m,                   0,      Cf*a/I,                     0,      0   ]';
size(B)
C = [0,     0,                      0,      0,                          1,      0;
     0,     0,                      1,      0,                          0,      0   ];
 
 D = [0 0]';
size(C)
vehicle = ss(A,B,C,D);

T = 15;         % simulation duration
time = 0:0.1:T; % simulation time
plotReference(Vx,time);

% Open Simulink model.
mdl = 'mpcVehicleSteering';
open_system(mdl)

Ts = 0.1;
mpc1 = mpc(vehicle,Ts);
mpc1.IsEconomicMPC = false;
mpc1.PredictionHorizon = 10;
mpc1.ControlHorizon = 2;

mpc1.MV.RateMin = -0.26;
mpc1.MV.RateMax = 0.26;

mpc1.Weights.OV = [0.8 0.3];                            
mpc1.Model.Plant

mpc1.Model.Plant=minreal(mpc1.Model.Plant);
sim(mdl)

%%
% Plot the simulation result.
plotSimulation(logsout,Vx,time)

bdclose(mdl)
