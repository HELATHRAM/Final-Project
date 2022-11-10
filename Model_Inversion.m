clc; close all; clear all
%% 
% System Parameters
%mh = 0.658+0.005; % kg  Mass of hook
%mp = 0.070; % kg Mass of Payload
L1 = 1.730;   % m  Length from trolley to hook; we can try three lengths for the cable 
%L2 = 0.70;  % m  Length from eye of hook to payload center
g=9.81;
wn=sqrt(g/L1);
x1 = 79-7; % mm Magnitude of the first oscillation
x3 = 68-7; % mm Magnitude of the second oscillation
z = Damping_Ratio_Log_Decrement(x1,x3)

s = tf('s');
TF=((1/L1)*s)/(s^2+2*z*wn*s+wn^2);
% TFD=c2d(TF,0.049);
yvt=(1/s)-(TF*L1);
TFD_yvt=c2d(yvt,0.049)
Inv_Mo=1/(TFD_yvt)
[num, den]=tfdata(Inv_Mo); 
num = cell2mat(num);
den = cell2mat(den);
roots(den)
mag=sqrt(0.9767^2+0.1778^2)
yd=200;
z = tf('z');
ydz=200*z/(z-1)
vt=TFD_yvt*ydz
vt2=TFD_yvt*200
syms z
x=(4.165*z^3-8.138*z^2+4.105*z)/(z^4-3.98*z^3+5.954*z^2-3.968*z+0.9939)
vtD=partfrac(x,z)
simplify(vtD)
syms k z
iztrans(vtD, k)
x2=(15.43*z^2 - 30.68*z + 15.37)/( z^3 - 2.98*z^2 + 2.974*z - 0.9939);
iztrans(x2)
% 
% syms n;
% answer = symsum((153700*r4^n - 306800*r4*r4^n + 154300*r4^n*r4^2)/(29800*r4^2 - 59480*r4 + 29817), r4 in root(z3^3 - (149*z3^2)/50 + (1487*z3)/500 - 9939/10000, z3)) - (153700*kroneckerDelta(n, 0))/9939; 
% n=0;
% eval(answer)



% ydz=c2d(yd,0.049)
% vt=yvt*200
% vtD=c2d(vt,0.049)
% % iztrans(vtD)
% syms z
% x=(4.165*z^2-8.138*z+4.105)/(z^3-2.98*z^2+2.974*z-0.9939)
% iztrans(x)
% simplify(iztrans(x))
% syms z 
% vtDD=partfrac(vtD,z)


% y=yt-theta*L1
% yt=(1/s)*vt; %intergrator 

function out = Damping_Ratio_Log_Decrement(x1,x3)
    delta = log(x1/x3);
    out = 1/(sqrt(1+(2*pi/delta)^2));
end