clc; clear; close all;

% PLOT Y VALUES THEN X VALUES SO THE AXISES MATCH THE PHYSICAL CRANE
figure()

% MATCH THE PLOT BOUNDS TO THE BOUNDS OF THE OBSTACLE COURSE
axis([-600 600 -410 1000])
hold on
grid on


xlabel('y (mm)')
ylabel('x (mm)')
title('Obstacle Course Layout')

% Plot Start and end values
xstart = -220;
ystart = -400;
scatter(ystart, xstart, 100, [0 0.4470 0.7410], "x");

xgoal = 750;
ygoal = 370;
scatter(ygoal, xgoal, 100,[0.4660 0.6740 0.1880],"x");

% Plot obstacles
scatter(0, 200, 200, [0.6350 0.0780 0.1840], "filled")

obst_1x = [-250 55 55];
obst_1y = [-100 -100 -410];
plot(obst_1y, obst_1x, 'Color', [0.6350 0.0780 0.1840]);

obst2_y = [-500 -500];
obst2_x = [-20 285];
plot(obst2_y, obst2_x, 'Color', [0.6350 0.0780 0.1840]);

obst3_y = [-350 260];
obst3_x = [400 400];
plot(obst3_y, obst3_x, 'Color', [0.6350 0.0780 0.1840]);

obst4_y = [100 405 405];
obst4_x = [0 0 610];
plot(obst4_y, obst4_x, 'Color', [0.6350 0.0780 0.1840])

obst5_y = [0 0];
obst5_x = [550 855];
plot(obst5_y, obst5_x, 'Color', [0.6350 0.0780 0.1840])

legend(["Start Position", "Goal", "Obstacles"])

%% ROUGH TRJECTORY

speeds = [0.1 0.2]*1000;

m1 = [0 -55];
m2 = [570 0];
m3 = [0 825];
m4 = [400 0];

total_dist = abs(m1) + abs(m2) + abs(m3) + abs(m4)

times = abs(m1)./speeds + abs(m2)./speeds + abs(m3)./speeds + abs(m4)./speeds

total_times = [14 7];


t1=total_times(2) * 55/(total_dist(2));
t2=total_times(1) * 570/(total_dist(1));
t3=total_times(2) * 825/(total_dist(2));
t4=total_times(1) * 400/(total_dist(1));

t = [0:0.049:sum(total_times)];

d1 = floor(t1/.049);
d2 = floor(t2/.049);
d3 = floor(t3/.049);
d4 = floor(t4/.049);

traj1 = [linspace(0,m1(1),d1)',linspace(0,m1(2),d1)'] + [xstart, ystart];
traj2 = [linspace(0,m2(1),d2)',linspace(0,m2(2),d2)'] + traj1(end,:);
traj3 = [linspace(0,m3(1),d3)',linspace(0,m3(2),d3)'] + traj2(end,:);
traj4 = [linspace(0,m4(1),d4)',linspace(0,m4(2),d4)'] + traj3(end,:);

traj = [traj1;traj2;traj3;traj4];
writematrix(traj, 'traj.csv');


scatter(traj(:,2), traj(:,1), 'b')


start = [xstart, ystart];
start+m1+m2+m3+m4;





