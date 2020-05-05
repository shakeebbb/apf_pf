clc
clear all


%% Static Obstacles

file_path = '/home/shakeeb/pomdp_ws/data_results/';

pt_file_name = 'pt_out_static_obs';
pt_piv_file_name = 'pt_piv_out_static_obs';

pt_data = readmatrix(append(file_path, pt_file_name));
pt_piv_data = readmatrix(append(file_path, pt_piv_file_name));

pt_dist = sqrt(pt_data(:,5).^2 + pt_data(:,6).^2 + pt_data(:,7).^2);
pt_piv_dist = sqrt(pt_piv_data(:,5).^2 + pt_piv_data(:,6).^2 + pt_piv_data(:,7).^2);

pt_pos = pt_data(:, 5:7);
pt_piv_pos = pt_piv_data(:, 5:7);

t_start = 0;
t_stop = 500;

t = (pt_data(:,1) - pt_data(1,1))*1e-9;

plot(pt_dist);
hold on
plot(pt_piv_dist);
ylim([0 2]);

%% Static Obstacle

figure
plot(t(1645:1840) - t(1645), pt_dist(1645:1840));
hold on
plot(t(1645:1840) - t(1645), pt_piv_dist(1645:1840));
ylim([0 2]);
grid

fill_x = [0,40,40,0];
fill_y = [1.5,1.5,2.0,2.0];

fill(fill_x, fill_y, 'r', 'EdgeColor', 'none', 'FaceAlpha', '0.5');

legend('Estimated closest obstacle', 'Observed closest obstacle', 'Obstacle Zone');
xlabel('Time (s)');
ylabel('Distance (m)');


%% No Obstacle

figure
plot(t(654:867) - t(654), pt_dist(654:867));
hold on
plot(t(654:867) - t(654), pt_piv_dist(654:867));
ylim([0 2]);
grid

fill_x = [0,40,40,0];
fill_y = [1.5,1.5,2.0,2.0];

legend('Estimated closest obstacle', 'Observed closest obstacle');
xlabel('Time (s)');
ylabel('Distance (m)');

%% 3D Plot

figure
scatter(pt_pos(1645:1840, 1), pt_pos(1645:1840, 3));

hold on
scatter(pt_piv_pos(1645:1840, 1), pt_piv_pos(1645:1840, 3), 'g');
grid

viscircles([0.0 1.6; 0.18 1.55; -0.18 1.61; 0.08 1.7; -0.1 1.62], [0.1 0.11 0.05 0.07 0.06])
xlabel('x (m)');
ylabel('z (m)');

legend('Estimated closest obstacle', 'Observed closest obstacle');

%% Dynamic Obstacles

file_path = '/home/shakeeb/pomdp_ws/data_results/';

pt_file_name = 'pt_out_dynamic_obs';
pt_piv_file_name = 'pt_piv_out_dynamic_obs';

pt_data = readmatrix(append(file_path, pt_file_name));
pt_piv_data = readmatrix(append(file_path, pt_piv_file_name));

pt_dist = sqrt(pt_data(:,5).^2 + pt_data(:,6).^2 + pt_data(:,7).^2);
pt_piv_dist = sqrt(pt_piv_data(:,5).^2 + pt_piv_data(:,6).^2 + pt_piv_data(:,7).^2);

pt_pos = pt_data(:, 5:7);
pt_piv_pos = pt_piv_data(:, 5:7);

t_start = 0;
t_stop = 500;

t = (pt_data(:,1) - pt_data(1,1))*1e-9;

plot(pt_dist);
hold on
plot(pt_piv_dist);
ylim([0 2]);
grid

%% Distannce vs Time

figure
plot(t(159:330) - t(159), pt_dist(159:330));
hold on
plot(t(159:330) - t(159), pt_piv_dist(159:330));
ylim([0 1]);
grid

fill_x = [0,40,40,0];
fill_y = [0.35,0.35,0.8,0.8];

fill(fill_x, fill_y, 'r', 'EdgeColor', 'none', 'FaceAlpha', '0.5');

legend('Estimated closest obstacle', 'Observed closest obstacle', 'Obstacle Zone');
xlabel('Time (s)');
ylabel('Distance (m)');


%% Position 3D Plot

figure
scatter(pt_pos(1645:1840, 1), pt_pos(1645:1840, 3));

hold on
scatter(pt_piv_pos(1645:1840, 1), pt_piv_pos(1645:1840, 3), 'g');
grid

viscircles([0.0 1.6; 0.18 1.55; -0.18 1.61; 0.08 1.7; -0.1 1.62], [0.1 0.11 0.05 0.07 0.06])
xlabel('x (m)');
ylabel('z (m)');

legend('Estimated closest obstacle', 'Observed closest obstacle');
