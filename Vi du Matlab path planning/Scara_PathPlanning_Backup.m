close all
% Thi?t k? qu? ??o s? d?ng ???ng b?c 5
% X(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5

% Chi?u dài 2 kh?p c?a robot
L1 = 3; L2 = 2; 

% Nh?p to? ?? 2 ?i?m ??u cu?i t? bàn phím
point = input('Give coord[ X0 Y0 Xf Yf]: '); 
X0 = point(1);Y0 = point(2); Xf = point(3); Yf = point(4);

% L?a ch?n các ?i?u ki?n ??u
X0d = 0; Xfd = 0;
X0dd = 0; Xfdd = 0;

t0 =0; tf = 10;


A = [1  t0  t0^2  t0^3    t0^4     t0^5;
     0  1   2*t0  3*t0^2  4*t0^3   5*t0^4;
     0  0   2     6*t0    12*t0^2  20*t0^3;
     1  tf  tf^2  tf^3    tf^4     tf^5;
     0  1   2*tf  3*tf^2  4*tf^3   5*tf^4;
     0  0   2     6*tf    12*tf^2  20*tf^3];
b = [X0; X0d; X0dd; Xf; Xfd; Xfdd];

a = inv(A)*b;
a0 = a(1); a1 = a(2); a2 = a(3);
a3 = a(4); a4 = a(5); a5 = a(6);

t = t0:0.1:tf;
Xt   = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
Xtd  = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
Xtdd = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;


Yt = (Yf - Y0)/(Xf-X0)*(Xt-X0)+Y0;
% tau = (Xt-X0)/(Xf-X0);
% Yt = (1-tau).^2*Y0+2*(1-tau).*tau*Y1+tau.^2*Yf;


c2 = (Xt.^2 + Yt.^2 - L1^2 -L2^2)/(2*L1*L2);
theta2 = atan2(sqrt(1-c2.^2),c2);
theta1 = atan2(Yt,Xt) - atan2(L2*sin(theta2),L1+L2*cos(theta2));


X = L1*cos(theta1) + L2*cos(theta1+theta2);
Y = L1*sin(theta1) + L2*sin(theta1+theta2);
X_med = L1*cos(theta1);
Y_med = L1*sin(theta1);
x = [];
y = [];
x1 = [];
y1 = [];
for i=1:20
    temp = [0; X_med(1+5*i)];
    x = [x temp];
    temp = [0; Y_med(1+5*i)];
    y = [y temp];
    temp  = [X_med(1+5*i);X(1+5*i)];
    x1 = [x1 temp];
    temp  = [Y_med(1+5*i);Y(1+5*i)];
    y1 = [y1 temp];
end
for i=1:20
    plot(x(:,i)',y(:,i)','b')
    hold on
    plot(x1(:,i)',y1(:,i)','r')
end


figure
plot(t,Xt,'r', t,Xtd,'g',t,Xtdd,'b')
grid on
legend('X', 'Xdot', 'Xddot')

figure
plot(t,theta1,'r',t,theta2,'b')
grid on
legend('\theta1', '\theta2','\theta3')











