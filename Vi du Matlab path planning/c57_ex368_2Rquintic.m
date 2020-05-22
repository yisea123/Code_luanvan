close all
% X(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
X0  = 0.41122; Xf  = -0.0282;
X0d = 0;  Xfd = 0;
X0dd = 0; Xfdd = 0;
t0  = 0;  tf  = 10;


A = [1  t0  t0^2  t0^3    t0^4     t0^5;
     0  1   2*t0  3*t0^2  4*t0^3   5*t0^4;
     0  0   2     6*t0    12*t0^2  20*t0^3;
     1  tf  tf^2  tf^3    tf^4     tf^5;
     0  1   2*tf  3*tf^2  4*tf^3   5*tf^4;
     0  0   2     6*tf    12*tf^2  20*tf^3];
 
b = [X0; X0d; X0dd; Xf; Xfd; Xfdd];

a = inv(A)*b
a0 = a(1); a1 = a(2); a2 = a(3);
a3 = a(4); a4 = a(5); a5 = a(6);

t = t0:0.2:tf;
Xt   = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
Xtd  = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
Xtd  = Xtd; 
Xtdd = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;
Xtdd = Xtdd;

L1 = 0.25; L2 = 0.25;
Yt = -0.259982*Xt + 0.3705;

c2 = (Xt.^2 + Yt.^2 - L1^2 - L2^2)/(2*L1*L2);
s2 = -sqrt(1 - c2.^2);
theta2 = atan2(s2,c2);
theta1 = atan2(Yt,Xt) - atan2(L2.*s2,L1+L2.*c2);


plot(t,Xt,'r', t,Xtd,'g',t,Xtdd,'b')
grid on
legend('X', 'Xdot', 'Xddot')

figure
plot(t,theta1,'r', t,theta2,'b')
grid on
legend('\theta1', '\theta2')

figure
X = L1*cos(theta1) + L2*cos(theta1+theta2);
Y = L1*sin(theta1) + L2*sin(theta1+theta2);
plot(X,Y,'.b'); grid on
xlabel('x(m)'); ylabel('y(m)')

