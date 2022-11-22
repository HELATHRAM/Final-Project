clc; clear; close all;

%% Plant Properties
L = 1.730; % m  Length from trolley to hook; we can try three lengths for the cable 
g=9.81; % m/s 
wn=sqrt(g/L); % rad/s
x1 = 0.079-0.007; % mm Magnitude of the first oscillation
x3 = 0.068-0.007; % mm Magnitude of the second oscillation
z1 = Damping_Ratio_Log_Decrement(x1,x3); % function to calcuate the damping ratio

%% S-Domain Transfer Function
A=[0 1;-(wn^2) -2*z1*wn];B=[0; 1]; C=[0 -1]; %(-wn^2/g)
D=[0];
[num,den]=ss2tf(A,B,C,D);

TF_s=tf([num],[den]); % plant TF in s-domain - Theta/v

s = tf('s');
G = 1/s + TF_s;

ts = 0.049;
TF_z = c2d(G, ts);

%% Z Domain Controller
z = tf('z', ts);

s = 2;
% C_z = (1/0.0002613) * ((z^3 - 2.98*z^2 + 2.974*z - 0.9939) * ((z^-1) + 1.7783) * z^-s) / ((z-0.0848)*(1+1.7783)^2);
C_z = (1/0.0002613) * ((z^3 - 2.98*z^2 + 2.974*z - 0.9939) * ((z^-1) + 0.5623) * z^-s) / ((z-0.0848)*(1+1.7783)^2);

G_z = TF_z * C_z;
%% Smooth trajectory generation
t_f = 15;
t = [0:ts:t_f];

% yd = sin(t)/5;

% Something is wrong with these equations..

y0 = 0; yf = 1;
vy0 = 0; vyf = 0;
ay0 = 0; ayf = 0;

a0 = y0; a1 = vy0; a2 = ay0/2;

a3 = 1/(2*t_f^3) * (20*yf - 20*y0 - (8*vyf + 12*vy0)*t_f - (3*ay0-ayf)*(t_f^2));
a4 = 1/(2*t_f^4) * (30*yf - 30*y0 + (14*vyf + 16*vy0)*t_f + (3*ay0-2*ayf)*(t_f^2));
a5 = 1/(2*t_f^5) * (12*yf - 12*y0 - (6*vyf + 6*vy0)*t_f - (ay0-ayf)*(t_f^2));

yd=a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;

% yd = t/10;
% yd(150:end) = yd(150);

yd = yd_s(yd, s);


%% Controller Testing
r_y = lsim(C_z, yd, t);

y_max = 0.2;
y_min = -0.2;
% r_y = actuator_limit(r_y, y_min, y_max);

y = lsim(G, r_y, t);

figure()
plot(t, yd);
hold on
% plot(t, r_y)
plot(t, y)
legend(["yd", "y"])
title('Simulated ZPETC')
xlabel('time (s)')
ylabel('m')

figure(2)
subplot (3,1,1)
plot(t, yd);
title('Desired Trajectory y_d')
xlabel('time (s)')
ylabel('m')

subplot (3,1,2)
plot(t, r_y);
title('Control Signal r_y');
xlabel('time (s)')
ylabel('m/s')

subplot (3,1,3)
plot(t, y);
title('Simulated Plant Output y')
xlabel('time (s)')
ylabel('m')

out = lsim(G_z, yd, t);
figure()
hold on
plot(t, yd)
plot(t,out)
title('Combined Transfer Function')
xlabel('t (s)')
ylabel('m')
legend(["y_d", "y"])

%%
% writematrix(round(100*r_y/0.2), 'r_y.csv');


%%
function u = actuator_limit(u, min, max)
    u(u<min)=min;
    u(u>max)=max;
end

function yd = yd_s(yd, s)
    yd = [yd(s+1:length(yd)), ones(1, s)*yd(length(yd))]; 
end

function out = Damping_Ratio_Log_Decrement(x1,x3)
    delta = log(x1/x3);
    out = 1/(sqrt(1+(2*pi/delta)^2));
end