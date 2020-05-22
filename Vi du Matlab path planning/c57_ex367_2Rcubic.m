close all
% X(t) = a0 + a1*t + a2*t^2 + a3*t^3 
X0  = 0.41122; Xf  = -0.0282;
X0d = 0;  Xfd = 0;
t0  = 0;  tf  = 10;

A = [1  t0  t0^2  t0^3;   
     0  1   2*t0  3*t0^2;  
     1  tf  tf^2  tf^3;
     0  1   2*tf  3*tf^2];
 
b = [X0; X0d; Xf; Xfd];

a = inv(A)*b
a0 = a(1); a1 = a(2); a2 = a(3); a3 = a(4);

t = t0:0.1:tf;
Xt   = a0 + a1*t + a2*t.^2 + a3*t.^3;
Xtd  = a1 + 2*a2*t + 3*a3*t.^2;
Xtd  = Xtd; 
Xtdd = 2*a2 + 6*a3*t;
Xtdd = Xtdd;

L1 = 0.25; L2 = 0.25;
Yt = -0.259982*Xt + 0.3705;

c2 = (Xt.^2 + Yt.^2 - L1^2 - L2^2)/(2*L1*L2);
s2 = -sqrt(1 - c2.^2);
theta2 = atan2(s2,c2);
theta1 = atan2(Yt,Xt) - atan2(L2.*s2,L1+L2.*c2);

h = figure;
plot(t,Xt,'r', t,Xtd,'g',t,Xtdd,'b')
grid on
legend('X', 'Xdot', 'Xddot')
set(h,'Position',[10 10 300 300]);

h = figure
plot(t,theta1,'r', t,theta2,'b')
grid on
legend('\theta1', '\theta2')
set(h,'Position',[10 10 300 300]);

X = L1*cos(theta1) + L2*cos(theta1+theta2);
Y = L1*sin(theta1) + L2*sin(theta1+theta2);
h = figure
plot(X,Y,'.r'); grid on
xlabel('x(m)'); ylabel('y(m)')
xlim([-0.2 0.6]);
ylim([0 0.5]);
set(h,'Position',[10 10 300 300]);

save 'c57_ex367.mat' Xt Yt theta1 theta2

figure
grid on
hold on
plot(X(1),Y(1),'.r')
xlim([-0.2 0.6]);
ylim([0 0.5]);
N = max(size(X)); 
i = 1;
STEP = 1;
while i<(N-STEP),
plot(X(i:(i+STEP)),Y(i:(i+STEP)),'.r')
pause(0.1);
i = i + STEP;
end

