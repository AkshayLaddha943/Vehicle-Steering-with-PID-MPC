function [Yref,Zref] = plotReference(Vx,time)
% Plot the reference trajectory and obstacles.

%%
Yref = zeros(length(time),1);    % Y position of reference trajectory
Xref = zeros(length(time),1);    % X position of reference trajectory
Zref = zeros(length(time),1);    % Yaw angle of reference trajectory

for ct = 1:length(time)
    [Yref(ct),Zref(ct),Xref(ct)] = doubleLaneChange(Vx,time(ct));
end

%%
% Plot the refrence path to follow.
plot(Xref,Yref,'b','LineWidth',2)
xlim([0,225])
ylim([-5,6])
grid on
hold on
