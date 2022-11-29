clc; clear; close all;
tf = 15;
t = [0:0.051:tf];

y0 = 0;
yf = 1;

yd0 = 0;
ydf = 0;

a0 = y0;
a1 = 0;
a2 = 3/(tf^2) * (yf - y0);
a3 = -2/(tf^3)*(yf-y0);

ydes = a0 + a1*t + a2*t.^2 + a3*t.^3;

plot(t,ydes)

%%
figure()

ydd0 = 0;
yddf = 0;

a0 = y0; a1 = yd0; a2 = ydd0/2;

a3 = 1/(2*tf^3) * ( 20*yf - 20*y0 - (8*ydf + 12*yd0)*tf - (3*ydd0 - yddf)*tf^2 );
a4 = 1/(2*tf^4) * ( 30*y0 - 30*yf + (14*ydf + 16*yd0)*tf + (3*ydd0 - 2*yddf)*tf^2 );
a5 = 1/(2*tf^5) * ( 12*yf - 12*y0 - (6*ydf + 6*yd0)*tf - (ydd0 - yddf)*tf^2 );

ydes = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;

plot(t,ydes)

