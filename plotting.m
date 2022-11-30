clc;close all;%clear;

nopause=readtable('GT_BridgeCrane_Data.csv');
    
nopause(492:end,:)=[];

headers=nopause.Properties.VariableNames';


figure()
subplot(3,1,1)
plot(nopause.TimeY_dir_sec_,nopause.YActualVelocity_mm_sec_/1000);
title('Bridge Crane Y Velocity')
xlabel('time (s)')
ylabel('m/s')

subplot (3,1,2)
plot(nopause.TimeY_dir_sec_,((nopause.YCranePosition_mm_-nopause.YCranePosition_mm_(1))+nopause.YPayloadDeflection_mm_)/1000);
title('Bridge Crane Payload Position')
xlabel('time (s)')
ylabel('m')

subplot(3,1,3)
plot(nopause.TimeY_dir_sec_,nopause.YPayloadDeflection_mm_/1000);
title('Bridge Crane Payload Deflection')
xlabel('time (s)')
ylabel('m')