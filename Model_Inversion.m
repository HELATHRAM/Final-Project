clc; close all; clear all
%%  System Parameters
L1 = 1.730;  % m  Length from trolley to hook; we can try three lengths for the cable 
g=9.81; % m/s 
wn=sqrt(g/L1); % rad/s
x1 = 0.079-0.007; % mm Magnitude of the first oscillation
x3 = 0.068-0.007; % mm Magnitude of the second oscillation
z1 = Damping_Ratio_Log_Decrement(x1,x3); % function to calcuate the damping ratio
%% S-Domain
A=[0 1;-(wn^2) -2*z1*wn];B=[0; 1]; C=[0 -1]; %(-wn^2/g)
D=[0];
[num,den]=ss2tf(A,B,C,D);
TF_s=tf([num],[den]); % plant TF in s-domain - Theta/v
s = tf('s'); %Tf = ((s*-1*(wn^2))/g)/(s^2+2*z*wn*s+wn^2) % Theta / V
roots(den);
G = (1/s)+(TF_s);
G_inv = G^-1; % NON CAUSAL?? 
%figure(1); %pzplot(G_inv);
%% Z-Domain
TF_half=c2d(TF_s,0.049);
[num2, den2]=tfdata(TF_half); 
num = cell2mat(num2);
den = cell2mat(den2);
roots(den);

TF_z=c2d(((1/s)+(TF_s)),0.049); %change TF in s to z with including zero order hold; maybe 0.049 
[Ad,Bd,Cd,Dd]=ssdata(TF_z)
Inv_Mo=1/(TF_z) %TF1=((-1/L1)*s)/(s^2+2*z*wn*s+wn^2);
%figure (2); %pzplot(Inv_Mo);
[num1, den1]=tfdata(Inv_Mo); 
num = cell2mat(num1);
den = cell2mat(den1);
roots(den)
mag=sqrt(0.9951^2+0.0823^2);
syms z
Inv_Mo2=partfrac((z^3 - 2.98*z^2 + 2.974*z - 0.9939)/(0.09774*z^2 - 0.1945*z + 0.09744),z);
% simplify(Inv_Mo2)
%before yd; this is for Inv_Model
iztrans(simplify(Inv_Mo2));
simplify(iztrans(simplify(Inv_Mo2)))

%% Velocity Calc 
z = tf('z');
yd=200*(z/(z-1));% in z domain %yd=200; % in the time domain 
v_z=Inv_Mo*yd;

syms z k %after yd; this is for v_z
iztrans((200*z^4 - 596.1*z^3 + 594.8*z^2 - 198.8*z)/(0.09774*z^3 - 0.2923*z^2 + 0.292*z - 0.09744),z,k);
%% Simulation 
% sys1 = tf(num,den,0.049) % this has the sampling rate / discrete 
% % yd=200; 
% % sam_rate=0.049:10
% input=0:0.049:yd    %it's non causal system  % Your input signal
% lsim(sys1,input,0.049)
% Rz=symmul(Inv_Mo,yd)
% rz=invztrans(Rz)
%% Zero Phase Error Tracking  
Q=(((z^3 - 2.98*z^2 + 2.974*z - 0.9939)*((z^-1)+1.7784))/((z-0.0848)*(1+1.7784)^2))
%R= Q/(z^2); 
%yd(k+2)
ydz=[];
yd=[0:10:200]';
for k=1:1:19;
ydz(k)=yd(k+2)
end 

R=lsim (Q,ydz,0.045)
Y=lsim(TF_z,R)

% Yd=[];
% for k=1:1:200, 
%Yd(k)=Yd*10
% end 
% Ts=0.01; 
% N=200;
% t=[0:(N/2-1)]'*Ts;
% Yd=4*t.^2.*(3-4*t).*(t<0.5)+(t>=0.5 & t<1);
% Yd=[Yd;flipud(Yd)]; Yd=[Yd;Yd];
% kf=length(Yd); kk=[0:kf-1]';
% R1=lsim(R,[Yd(3:kf);Yd(1:2)])
% Y=lsim(TF_z,R);

syms z
x=(-0.001416*z^2 - 0.002398*z + 0.0002136)/(z^3 - 2.98*z^2 + 2.974*z - 0.9939);
iztrans(x,k);

function out = Damping_Ratio_Log_Decrement(x1,x3)
    delta = log(x1/x3);
    out = 1/(sqrt(1+(2*pi/delta)^2));
end
