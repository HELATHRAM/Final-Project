clear; close all;

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

G_s = G;
G = G*s;

ts = 0.051;
TF_z = c2d(G, ts)

%% Z Domain Controller
z = tf('z', ts);

s = 3;
% C_z = (1/0.0002613) * ((z^3 - 2.98*z^2 + 2.974*z - 0.9939) * ((z^-1) + 1.7783) * z^-s) / ((z-0.0848)*(1+1.7783)^2);
% C_z = (1/0.0002613) * ((z^3 - 2.98*z^2 + 2.974*z - 0.9939) * ((z^-1) + 0.5623) * z^-s) / ((z-0.0848)*(1+1.7783)^2);]

s=1;
% C_z = (1/0.0001745) * ((z^3 - 2.979*z^2 + 2.973*z - 0.9936) * ((z^-1) + 1.0177) * z^-s) / ((z+0.0702022)*(z+0.98257)*(1+1.0177)^2);

% s=4;
% C_z = (1/0.006844) * (z^3 - 2.979*z^2 + 2.973*z - 0.9936) * (z-1) * (z+1) * z^-s / ((z+0.070275)*4);


% TF_z= (0.0002613*(1/z) + 0.0004425*(1/z^2) - (3.942*10^-5)*(1/z^3)) / (1 - 2.98*(z^-1) + 2.974*(z^-2) - 0.9939*(z^-3))

% C_z = ((1/0.0002879)*(z^3 - 2.979*z^2 + 2.973*z - 0.9939) * ((z^-1)+1.8052)*z^-s) / ( (z-0.0727)*(1+1.8052)^2)

% C_z = 4.4248*(1-2.979*z^-1 + 2.973*z^-2 - 0.9939*z^-3) * z^-1 / (-0.0000378 * ((z^-1)-17.385) * ((z^-1)+4.34269));

% TF_z= (0.0002879*z^-1 + 0.0004988*z^-2 - (3.78*10^-5)*z^-3) / (1 - 2.979*(z^-1) + 2.973*(z^-2) - 0.9939*(z^-3))

C_z = inv(TF_z)*inv(z)

%% Smooth trajectory generation

% yd = sin(t)/5;

y0 = 0;
yf = 0.7;

% tf = yf/0.1;
tf = 20;

t = [0:ts:tf];

yd0 = 0;
ydf = 0;

a0 = y0;
a1 = 0;
a2 = 3/(tf^2) * (yf - y0);
a3 = -2/(tf^3)*(yf-y0);

yd = a0 + a1*t + a2*t.^2 + a3*t.^3;

yd = t/30;
% yd(150:end) = yd(150);

pos = yd;
vel = diff(yd)/ts;
accel =diff(diff(yd))/ts;

yd = [vel vel(end)];

yd = yd_s(yd, s);

% scale = 1.8*(2/pi)*atan(2.7*yf*pi/2);
scale=1;
% C_z = C_z/scale;


% yd = 0 * yd;
% yd(1)=.1;

%% Controller Testing
r_y = lsim(C_z, yd, t);

y_max = 0.2;
y_min = -0.2;
r_y = actuator_limit(r_y, y_min, y_max);

r_y = r_y/scale;

y = lsim(G_s, r_y, t);

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


G_z = TF_z * C_z;
out = lsim(G_s, yd, t);
figure()
hold on
plot(t, yd)
plot(t,out)
title('Combined Transfer Function')
xlabel('t (s)')
ylabel('m')
legend(["y_d", "y"])

fprintf('%.3f, %.3f\n', yf, y(end)/yf);

%%
writematrix(round(100*r_y/0.2), 'r_y.csv');

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