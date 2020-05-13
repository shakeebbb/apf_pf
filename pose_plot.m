clc
clear all

data = readmatrix('/home/shakeeb/Downloads/open_loop_global_planner.csv');

plot3(data(2:end,6), data(2:end,7), data(2:end,8))
grid
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')

title('UAV Position');