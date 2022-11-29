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

%% Coming up with the gain term
syms z a0 a1 a2 a3 a4 a5
eq=(3.8786+1.4365*(z+z^-1)-0.15*(z^2+z^-2))*(a0+a1*(z+z^-1)+a2*(z^2+z^-2)+a3*(z^3+z^-3)+a4*(z^4+z^-4)+a5*(z^5+z^-5))
x=vpa (expand(eq))

C=[3.8786, 2.873, -0.3, 0, 0, 0;
 1.4365, 3.7286,1.4365,-0.15, 0, 0;
 -0.15, 1.4365, 3.8786, 1.4365, -0.15, 0;
 0, -0.15, 1.4365, 3.8786, 1.4365, -0.15;
 0, 0, -0.15,1.4365, 3.8786,1.4365;
 0, 0, 0, -0.15,1.4365,3.8786 ]
D=[1; 0; 0; 0; 0; 0]
alpha=linsolve(C,D)

a0=(0.4253/2);
a1=-0.2135;
a2=0.1217;
a3=-0.0670;
a4=0.0353;
a5=-0.0157;

%% Z Domain Controller
z = tf('z', 0.049);
C_z =((z^-7)*((2*a0)+a1*(z+z^-1)+a2*(z^2+z^-2)+a3*(z^3+z^-3)+a4*(z^4+z^-4)+a5*(z^5+z^-5))*((z^3 - 2.98*z^2 + 2.974*z - 0.9939) * ((z^-1) + 1.77) )) / ((z-0.0848))

tot=z^-7*((2*a0)+a1*(z+z^-1)+a2*(z^2+z^-2)+a3*(z^3+z^-3)+a4*(z^4+z^-4)+a5*(z^5+z^-5))*(1+1.6934*z-0.15086*z^2)*(1+1.6934*z^-1-0.15086*z^-2);
% tot = C_z*TF_z;

%% Testing trajectory tracking
t = [0:0.049:10];
yd = 0.1*t;
yd(150:end)=yd(150);

yd = yd_s(yd);


u = lsim(C_z, yd, t);
% u = actuator_limit(u, -.2, .2);

y = lsim(G, u*5000, t);
% y = lsim(tot, yd, t);
plot(t, yd);
hold on
plot(t, u)
plot(t, y)
legend(["yd", "u", "y"])

figure(2)
subplot (3,1,1)
plot(t, yd);
title('yd')
subplot (3,1,2)
plot(t, u);
title('u');
subplot (3,1,3)
plot(t, y);
title('y')


%%
function u = actuator_limit(u, min, max)
    u(u<min)=min;
    u(u>max)=max;
end

function yd = yd_s(yd)
    s=7;
    yd = [yd(s+1:length(yd)), ones(1, s)*yd(length(yd))]; 

end

function out = Damping_Ratio_Log_Decrement(x1,x3)
    delta = log(x1/x3);
    out = 1/(sqrt(1+(2*pi/delta)^2));
end
