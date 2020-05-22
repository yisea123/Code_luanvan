close all
% q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
t0 = 0; tf = 1;
t = t0:0.01:tf;
q1 = 10 + 105*t.^2 - 70*t.^3;
q2 = 10 + 350*t.^3 - 525*t.^4 + 210*t.^5;

a1 = 1; a2 = 1;
x = a1*cos(q1*pi/180) + a2*cos((q1+q2)*pi/180); 
y = a1*sin(q1*pi/180) + a2*sin((q1+q2)*pi/180); 

h = figure
plot(t,q1,'r', t,q2,'b')
legend('q1','q2')
grid on
set(h,'Position',[10 10 300 300]);

h = figure
plot(t,x,'r', t,y,'b')
legend('x','y')
grid on
set(h,'Position',[10 10 300 300]);

h = figure
plot(x,y,'.b')
xlabel('x(m)'); ylabel('y(m)')
grid on
set(h,'Position',[10 10 300 300]);
