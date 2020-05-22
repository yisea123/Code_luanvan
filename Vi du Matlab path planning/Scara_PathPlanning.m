close all
% Thiet ke quy dao su dung duong bac 5
% X(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5

% Chieu dai 2 khop cua robot
L1 = 2; L2 = 2; 

% Nhap toa do 2 diem dau cuoi tu ban phim
point = input('Give coord[ X0 Y0 Xf Yf]: '); 
X0 = point(1);Y0 = point(2); Xf = point(3); Yf = point(4);

% Lua chon cac dieu kien dau
X0d = 0; Xfd = 0;
X0dd = 0; Xfdd = 0;
t0 =0; tf = 1;


% Tính toán các h? s? cho ???ng cong
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

% Tinh toan cac gia tri vi tri, thoi gian va van toc cua bien X
t = t0:0.01:tf;
Xt   = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
Xtd  = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
Xtdd = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;

% Phuong trinh duong thang cua Y theo bien X
Yt = (Yf - Y0)/(Xf-X0)*(Xt-X0)+Y0;


% Phuong trinh dong hoc nguoc
c2 = (Xt.^2 + Yt.^2 - L1^2 -L2^2)/(2*L1*L2);
theta2 = atan2(sqrt(1-c2.^2),c2);
theta1 = atan2(Yt,Xt) - atan2(L2*sin(theta2),L1+L2*cos(theta2));


% Tinh toan lai X,Y tu phuong trinh dong hoc thuan
X = L1*cos(theta1) + L2*cos(theta1+theta2);
Y = L1*sin(theta1) + L2*sin(theta1+theta2);
X_med = L1*cos(theta1);
Y_med = L1*sin(theta1);



%Ve do thi kiem chung ket qua
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











