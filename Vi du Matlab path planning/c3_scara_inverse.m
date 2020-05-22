a1 = 0.5;
a2 = 0.5;
theta1 = 5*pi/180;
theta2 = 0*pi/180;
theta4 = 0*pi/180;
d3 = 0.1;
d4 = 0.1;

ox = 0.75;
oy = 0.25;
oz = 0;

c2 = (ox^2 + oy^2 - a1^2 - a2^2)/(2*a1*a2);
theta2 = atan2(c2,sqrt(1-c2^2));
%theta2 = atan2(-sqrt(1-c2^2),c2)
s2 = sin(theta2);


theta1 = atan2(ox,oy) - atan2(a1+a2*c2,a2*s2);
% theta1 = atan2(oy,ox) - atan2(a2*s2,a1+a2*c2);
d3 = -(oz + d4);

theta1 = theta1*180/pi
theta2 = theta2*180/pi
