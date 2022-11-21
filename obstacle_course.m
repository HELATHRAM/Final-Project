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
