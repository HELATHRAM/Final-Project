clc; close all; clear all
%%  System Parameters
L1 = 1.73;   % m  Length from trolley to hook; we can try three lengths for the cable 
g=9.81; % m/s 
wn=sqrt(g/L1)/(2*pi);
x1 = .079-.007; % mm Magnitude of the first oscillation
x3 = .068-.007; % mm Magnitude of the second oscillation
z = Damping_Ratio_Log_Decrement(x1,x3); % function to calcuate the damping ratio
A=[0 1; -(wn^2) -2*z*wn];
B=[0; 1]; 
C=[0 -1/L1];
D=[0];

%% S Domain
[num,den]=ss2tf(A,B,C,D);
TF_s=tf([num],[den]) % plant TF in s-domain
s = tf('s');
Tf = ((s*-wn^2)/g)/(s^2+2*z+wn*s+wn^2)
G = (1/s)+TF_s
G_inv = G^-1 % NON CAUSAL??

%% Z Domain
TF_z = c2d(G, 0.049) %change TF in s to z with including zero order hold; maybe 0.049 
Inv_Mo = 1/(TF_z)

[num1, den1] = tfdata(TF_z); 
num1 = cell2mat(num1);
den1 = cell2mat(den1);
roots(num1)

%%
yd=200;
v_z=TF_z*200
syms z
x=(19.55*z^2 - 38.9*z + 19.49)/(z^3 - 2.98*z^2 + 2.974*z - 0.9939);
iztrans(x)

function out = Damping_Ratio_Log_Decrement(x1,x3)
    delta = log(x1/x3);
    out = 1/(sqrt(1+(2*pi/delta)^2));
end
