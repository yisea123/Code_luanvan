close all
% X(t) = a0 + a1*t + a2*t^2 + a3*t^3 
X0  = 1; Xf  = -1;
X0d = 0;  Xfd = 0;
t0  = 0;  tf  = 1;

A = [1  t0  t0^2  t0^3;   
     0  1   2*t0  3*t0^2;  
     1  tf  tf^2  tf^3;
     0  1   2*tf  3*tf^2];
 
b = [X0; X0d; Xf; Xfd];

a = inv(A)*b
a0 = a(1); a1 = a(2); a2 = a(3); a3 = a(4);

t = t0:0.01:tf;
Xt   = a0 + a1*t + a2*t.^2 + a3*t.^3;
Xtd  = a1 + 2*a2*t + 3*a3*t.^2;
Xtd  = Xtd /2; 
Xtdd = 2*a2 + 6*a3*t;
Xtdd = Xtdd /10;

L1 = 1; L2 = 1;
Yt = 1.5;

c2 = (Xt.^2 + Yt^2 - L1^2 - L2^2)/(2*L1*L2);
s2 = -sqrt(1 - c2.^2);
theta2 = atan2(s2,c2);
theta1 = atan2(Yt,Xt) - atan2(L2.*s2,L1+L2.*c2);


plot(t,Xt,'r', t,Xtd,'b')
grid on
legend('X', 'Xdot')

figure
plot(t,theta1,'r', t,theta2,'b')
grid on
legend('\theta1', '\theta2')

figure
X = L1*cos(theta1) + L2*cos(theta1+theta2);
Y = L1*sin(theta1) + L2*sin(theta1+theta2);
plot(X,Y,'.r')
xlabel('x(m)'); ylabel('y(m)')


