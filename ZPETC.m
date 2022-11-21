% clc; clear; close all;

%% Plant Properties
L = 1.730;  % m  Length from trolley to hook; we can try three lengths for the cable 
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

TF_z = c2d(G,0.049);

%% Z Domain Controller
z = tf('z', 0.049);

s = 2;
C_z = (1/0.0002613) * ((z^3 - 2.98*z^2 + 2.974*z - 0.9939) * ((z^-1) + 1.7783) * z^-s) / ((z-0.0848)*(1+1.7783)^2);

%% Testing trajectory tracking

t = [0:0.049:6];

yd = t/10;
% yd(111:end)=yd(111);

yd = yd_s(yd, s);

r_y = lsim(C_z, yd, t);

y_max = 0.2;
y_min = -0.2;
r_y = actuator_limit(r_y, y_min, y_max);

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