clc; clear; close all;
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
TF_z = c2d(G,0.049)

%% Z Domain Controller
z = tf('z',0.049);
C_z = (1/0.0002613) * (((1- 2.98*(z^-1) +2.974*(z^-2) - 0.9939*(z^-3)) * (z +0.5658) *(z^-1) ) / (((z^-1)-11.79)*(1+0.5658)^2))
G_z =  (((z^-1) +0.5658) *(z +0.5658)*(z^-1)) / ((1+0.5658)^2) %direct method to find y from yd

%% Testing trajectory tracking

t = [0:0.049:10];
yd = (0.1/5)*t
% yd = sin(t)
yd = [yd(3:end) yd(end) yd(end)];

u = lsim(C_z, yd, t);
y = lsim(TF_z, u, t);


figure(1)
plot(t, yd);
hold on
plot(t, u);
hold on 
plot(t, y);
legend(["yd", "u", "y"]);

y2 = lsim(G_z, yd, t);
figure(2)
subplot (3,1,1)
plot(t, yd);
subplot (3,1,2)
plot(t, y2);
subplot (3,1,3)
plot(t, u);


%%
function out = Damping_Ratio_Log_Decrement(x1,x3)
    delta = log(x1/x3);
    out = 1/(sqrt(1+(2*pi/delta)^2));
end
