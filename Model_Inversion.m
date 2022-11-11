clc; close all; clear all
%%  System Parameters
L1 = 1.730;   % m  Length from trolley to hook; we can try three lengths for the cable 
g=9.81; % m/s 
wn=sqrt(g/L1);
x1 = 79-7; % mm Magnitude of the first oscillation
x3 = 68-7; % mm Magnitude of the second oscillation
z = Damping_Ratio_Log_Decrement(x1,x3); % function to calcuate the damping ratio
A=[0 1; -(wn^2) -2*z*wn];
B=[0; 1]; 
C=[0 -1/L1]; %(-wn^2/g)
D=[0];
[num,den]=ss2tf(A,B,C,D);
TF_s=tf([num],[den]); % plant TF in s-domain
s = tf('s');
TF_z=c2d(((1/s)-(TF_s*L1)),0.049) %change TF in s to z with including zero order hold; maybe 0.049 
Inv_Mo=1/(TF_z);
[num1, den1]=tfdata(Inv_Mo); 
num = cell2mat(num1);
den = cell2mat(den1);
roots(den)
yd=200;
v_z=TF_z*200
syms z
x=(19.55*z^2 - 38.9*z + 19.49)/(z^3 - 2.98*z^2 + 2.974*z - 0.9939);
iztrans(x)







% z = tf('z');
% iztrans(v_z)
% syms z
% x=(4.165*z^3-8.138*z^2+4.105*z)/(z^4-3.98*z^3+5.954*z^2-3.968*z+0.9939)
% vtD=partfrac(x,z)
% simplify(vtD)
% syms k z
% iztrans(vtD, k)
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
% s = tf('s');
% TF1=((1/L1)*s)/(s^2+2*z*wn*s+wn^2);
% TFD=c2d(TF,0.049);
% yvt=(1/s)-(TF1*L1);
% TFD_yvt=c2d(yvt,0.049)
% y=yt-theta*L1
% yt=(1/s)*vt; %intergrator 
% mag=sqrt(0.9767^2+0.1778^2)
% TF_zd=c2d(yd,0.1)
% z = tf('z');
% ydz=200*z/(z-1)
% vt=TFD_yvt*ydz

function out = Damping_Ratio_Log_Decrement(x1,x3)
    delta = log(x1/x3);
    out = 1/(sqrt(1+(2*pi/delta)^2));
end
