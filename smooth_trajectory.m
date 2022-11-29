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

yd = a0 + a1*t + a2*t.^2 + a3*t.^3;

plot(t,yd)

%%
