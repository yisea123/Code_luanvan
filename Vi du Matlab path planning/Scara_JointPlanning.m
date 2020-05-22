close all

% theta(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
L1 = 2; L2 = 2; d4 = 1;

% Nhap diem dau va cuoi tu ban phim
point = input('Give coord[ X0 Y0 Xf Yf]: '); 
X0 = point(1);Y0 = point(2); Xf = point(3); Yf = point(4);

%Tinh cac gia tri bien khop tai diem dau
c2 = (X0^2 + Y0^2 - L1^2 -L2^2)/(2*L1*L2);
theta2_0 = atan2(sqrt(1-c2^2),c2)*sign((rand(1)-0.5));
theta1_0 = atan2(Y0,X0) - atan2(L2*sin(theta2_0),L1+L2*cos(theta2_0));

%Tinh cac gia tri bien khop tai diem cuoi
c2 = (Xf^2 + Yf^2 - L1^2 -L2^2)/(2*L1*L2);
theta2_f = atan2(sqrt(1-c2^2),c2)*sign((rand(1)-0.5));
theta1_f = atan2(Yf,Xf) - atan2(L2*sin(theta2_f),L1+L2*cos(theta2_f));


% Phuong trinh duong cong theta1 theo thoi gian
theta1_0d = 0; theta1_fd = 0;
theta1_0dd = 0; theta1_fdd = 0;

t0 =0; tf = 1;

A = [1  t0  t0^2  t0^3    t0^4     t0^5;
     0  1   2*t0  3*t0^2  4*t0^3   5*t0^4;
     0  0   2     6*t0    12*t0^2  20*t0^3;
     1  tf  tf^2  tf^3    tf^4     tf^5;
     0  1   2*tf  3*tf^2  4*tf^3   5*tf^4;
     0  0   2     6*tf    12*tf^2  20*tf^3];
b = [theta1_0; theta1_0d; theta1_0dd; theta1_f; theta1_fd; theta1_fdd];

a = inv(A)*b;
a0 = a(1); a1 = a(2); a2 = a(3);
a3 = a(4); a4 = a(5); a5 = a(6);

t = t0:0.001:tf;
theta1   = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
theta1_d  = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
theta1_dd = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;


% Phuong trinh duong cong theta2 theo thoi gian
b = [theta2_0; theta1_0d; theta1_0dd; theta2_f; theta1_fd; theta1_fdd];
a = inv(A)*b;
a0 = a(1); a1 = a(2); a2 = a(3);
a3 = a(4); a4 = a(5); a5 = a(6);
theta2   = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
theta2_d  = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
theta2_dd = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;




% Phuong trinh dong hoc thuan tinh toan toa do X,Y
X = L1*cos(theta1) + L2*cos(theta1+theta2);
Y = L1*sin(theta1) + L2*sin(theta1+theta2);
Xd = zeros(1,length(X));
X_med = L1*cos(theta1);
Y_med = L1*sin(theta1);

% Tinh van toc cua X 
for i = 1:(length(X)-1)
    Xd(i) = (X(i+1)-X(i))/0.1;
end


% Ve do thi kiem tra ket qua
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
plot(t,Xd,'r',t,X,'g')
grid on
legend('Xd','X')

figure
plot(t,theta1,'r', t,theta1_d,'g',t,theta1_dd,'b')
grid on
legend('theta1', 'theta1dot', 'theta1ddot')

figure
plot(t,theta2,'r', t,theta2_d,'g',t,theta2_dd,'b')
grid on
legend('theta2', 'theta2dot', 'theta2ddot')

% figure
% K=2000;
% theta1 = (rand(1,K))*pi;
% theta2 = (rand(1,K)-0.5)*pi;
% x_temp = L1*cos(theta1)+L2*cos(theta1+theta2);
% y_temp = L1*sin(theta1)+L2*sin(theta1+theta2);
% plot(x_temp,y_temp,'.b')


