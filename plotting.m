clc;close all;%clear;

pause=readtable('GT_BridgeCrane_Data_with pause.csv');
nopause=readtable('GT_BridgeCrane_Data_without pause.csv');

pause(281:end,:)=[];
nopause(223:end,:)=[];

headers=pause.Properties.VariableNames';

figure()
subplot(3,1,1)
plot(pause.TimeY_dir_sec_,pause.YActualVelocity_mm_sec_/1000);

subplot (3,1,2)
plot(pause.TimeY_dir_sec_,((pause.YCranePosition_mm_-pause.YCranePosition_mm_(1))+pause.YPayloadDeflection_mm_)/1000);

subplot(3,1,3)
plot(pause.TimeY_dir_sec_,pause.YPayloadDeflection_mm_/1000);



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