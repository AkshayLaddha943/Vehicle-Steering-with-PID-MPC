% The model that we are using is the bicycle model. 
% The controller is the PID controller.
% Optimization Algorithm used is the Gradient Descent

clc; clear; close all;

get_coefs = false;
PID_coefs = [20,30,25];    % [65, 71, 2] is a good set
dynamic_plot = true;

L = 0.2;        % Rear and front axes distances
v = 0.1;        % Longitudinal velocities
x0 = 0.5;       % Initial position in horizontal direction
y0 = 0;         % Initial position in vertical direction
theta0 = 0;     % Initial robot head angle
delta_min = -pi/4;  % Lower bound for steering wheel
delta_max = pi/4;   % Upper bound for steering whell

%% Setting the params
user_params.x0 = x0;
user_params.y0 = y0;
user_params.theta0 = theta0;
user_params.L = L;
user_params.v = v;
user_params.delta_min = delta_min;
user_params.delta_max = delta_max;

if get_coefs
    %% Optimization
    lowerBound = -10;  
    upperBound = 100;  
    lb = ones(1, 3)*lowerBound;
    ub=ones(1, 3)*upperBound;
    nvar=numel(lb);
    k0=lb+rand(1,nvar).*(ub-lb);
    final_obj_func = @(k) obj_fcn(k, user_params);
    options=optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxIterations',10);
    coefs=fmincon(final_obj_func,k0,[],[],[],[],lb,ub,[],options);
else
    coefs = PID_coefs;
end

%% Display
disp('controller coefficients: ')
disp(coefs)
vars = simulate_plant(coefs, user_params);
sim_plots(vars, dynamic_plot)

%---------------------------------------------------------------------------------------------------
function init_params=initialize(user_params)
%% Problem parameters and variables
%% System
L = user_params.L;
v = user_params.v;

% Initial conditions
x0 = user_params.x0;
y0 = user_params.y0;
theta0 = user_params.theta0;

% Constraints
delta_min = user_params.delta_min;
delta_max = user_params.delta_max;

% Simulation
dt = 0.05;
t_vec = 0;

% Initialization
x_ref = linspace(0, 3.28, 328);
y_ref = 0.6*sin(2*x_ref);
q_mat = [x0, y0, theta0]';
delta_vec = 0;
err_vec = 0;
err_mat = [];

init_params.L = L;
init_params.v = v;
init_params.delta_min = delta_min;
init_params.delta_max = delta_max;
init_params.dt = dt;
init_params.t_vec = t_vec;
init_params.x_ref = x_ref;
init_params.y_ref = y_ref;
init_params.q_mat = q_mat;
init_params.delta_vec = delta_vec;
init_params.err_vec = err_vec;
init_params.err_mat = err_mat;
end

%--------------------------------------------------------------------------------------------------

function cost=obj_fcn(coefs, user_params)
sim_vars = simulate_plant(coefs, user_params);
q_mat = sim_vars.q_mat;
delta_vec = sim_vars.delta_vec;
err_vec = sim_vars.err_vec;
x_ref = sim_vars.x_ref;
y_ref = sim_vars.y_ref;

delta_mean_abs = sum(abs(delta_vec))/numel(delta_vec);
cost = sum(abs(err_vec))/numel(err_vec)+delta_mean_abs;

dist_r2start = sqrt((q_mat(1,end)-x_ref(1))^2+(q_mat(2,end)-y_ref(1))^2);
dist_r2end = sqrt((q_mat(1,end)-x_ref(end))^2+(q_mat(2,end)-y_ref(end))^2);

if dist_r2start < dist_r2end
    cost = 10000 * cost;
end
end

%-----------------------------------------------------------------------------------------------------

function qdot=plant(t, q, delta, params)
L = params.L;
v = params.v;
qdot = v*[cos(delta+q(3)), sin(delta+q(3)), sin(delta)/L]';
end

%------------------------------------------------------------------------------------------------------

function sim_plots(vars, dynamic_plot)
t_vec = vars.t_vec;
q_mat = vars.q_mat;
delta_vec = vars.delta_vec;
err_vec = vars.err_vec;
x_ref = vars.x_ref;
y_ref = vars.y_ref;
L = vars.L;

figure(1)
plot(x_ref, y_ref, 'b', "LineWidth", 8, "DisplayName", "Reference Path")
hold on
plot(q_mat(1,1), q_mat(2,1), 'pb', "DisplayName", "Start")
plot(q_mat(1,end), q_mat(2,end), 'or', "DisplayName", "End")
plot(q_mat(1,:), q_mat(2,:),'g',"LineWidth",3, "DisplayName", "Vehicle Path")
legend show
axis equal
xlabel("x (m)")
ylabel("y (m)")
title("Static trajectory of vehicle, controlled by PID")

figure(2)
plot(t_vec, delta_vec*180/pi)
xlabel("t (sec)")
ylabel("\delta (degree)")
title("Input: Steering wheel")

figure(3)
plot(t_vec, err_vec)
xlabel("t (sec)")
ylabel("Trajectory error (m)")

% Dynamic plot
if dynamic_plot
    for i=1:size(q_mat,2)
        figure(4)
        plot(x_ref, y_ref, 'b', "LineWidth", 20)
        axis equal
        hold on
        plot([q_mat(1,i),q_mat(1,i)+L*cos(q_mat(3,i))],[q_mat(2,i),...
            q_mat(2,i)+L*sin(q_mat(3,i))],...
            'g', "LineWidth",8)
        hold off
        pause(0.00001)
    end
end
end

%------------------------------------------------------------------------------------------

function vars=simulate_plant(coefs, user_params)
init_params=initialize(user_params);
L = init_params.L;
v = init_params.v;
delta_min = init_params.delta_min;
delta_max = init_params.delta_max;
dt = init_params.dt;
t_vec = init_params.t_vec;
x_ref = init_params.x_ref;
y_ref = init_params.y_ref;
q_mat = init_params.q_mat;
delta_vec = init_params.delta_vec;
err_vec = init_params.err_vec;
err_mat = init_params.err_mat;

params.L = L;
params.v = v;
params.dt = dt;

% Controller
kp = coefs(1);
ki = coefs(2);
kd = coefs(3);
%% Simulation Loop
i = 1;
while true
    [err, nearest_arg]=find_distance(q_mat(1,end), q_mat(2,end),...
        x_ref, y_ref);
    if nearest_arg == numel(x_ref)
        break
    end
    
    % Input (Steering wheel)
    coef1 = y_ref(nearest_arg) - q_mat(2,end);
    err_pid = [err, (err-err_vec(end))/dt, dt*trapz(err_vec)]';
    delta = coef1*(kp*err_pid(1)+kd*err_pid(2)+ki*err_pid(3));
    
    % Constraints
    if delta < delta_min
        delta = delta_min;
    elseif delta > delta_max
        delta = delta_max;
    end
    
    % solve state equation
    q_new = solver(t_vec(end), q_mat(:, end), delta, params);
    
    t_vec = [t_vec, t_vec(end)+dt];
    q_mat = [q_mat, q_new];
    delta_vec = [delta_vec, delta];
    err_vec = [err_vec, err];
    err_mat = [err_mat, err_pid];

    % Uregent Stopping
    if i == 10000
        break
    end
    i = i + 1;
end

vars.t_vec = t_vec;
vars.q_mat = q_mat;
vars.delta_vec = delta_vec;
vars.err_vec = err_vec;
vars.x_ref = x_ref;
vars.y_ref = y_ref;
vars.L = L;
end

%--------------------------------------------------------------------------------------------------

function q_new=solver(t, q, delta, params)
dt = params.dt;
method = 'r';    % r: runge-kutta, t: taylor expansion
if method == 't'
    qdot = plant(t, q, delta, params);
    q_new = q+dt*qdot;
elseif method == 'r'
    h = dt;
    y = q;
    k1 = plant(t, y, delta, params);
    k2 = plant(t+h/2, y+h*k1/2, delta, params);
    k3 = plant(t+h/2, y+h*k2/2, delta, params);
    k4 = plant(t+h, y+h*k3, delta, params);
    
    y_new = y + h/6*(k1+2*k2+2*k3+k4);
    
    q_new = y_new;
end
end

%-------------------------------------------------------------------------------------------

function [dist, dist_arg]=find_distance(x, y, x_ref, y_ref)
[dist, dist_arg] = min(sqrt(sum([x-x_ref;y-y_ref].^2)));
end

%%--------------------------------------------------------------------------------------------