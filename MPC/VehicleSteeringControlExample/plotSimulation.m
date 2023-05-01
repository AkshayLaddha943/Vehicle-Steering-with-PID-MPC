function plotSimulation(logsout,Vx,time)
% Plot the simulation results from logged data.

steering = logsout.getElement('steering_angle');
outputs = logsout.getElement('outputs');
Y = outputs.Values.Data(:,1); % Y position
Z = outputs.Values.Data(:,2); % yaw angle

figure('position',[100 100 800 720])

subplot(3,1,1)
[~,Zref] = plotReference(Vx,time);
hold on
plot(outputs.Values.time*Vx,Y,'g','LineWidth',2)
legend('reference','mpc')
grid on

subplot(3,1,2)
plot(time,Zref,'b','LineWidth',2)
hold on
plot(outputs.Values.time,Z,'g','LineWidth',2)
xlabel('time (s)')
ylabel('yaw angle (rad)')
legend('reference','mpc')
grid on

subplot(3,1,3)
plot(time,steering.Values.Data,'g','LineWidth',2)
xlabel('time (s)')
ylabel('steering angle (rad)')
legend('mpc')
grid on



