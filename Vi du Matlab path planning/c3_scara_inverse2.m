clear all

a1 = 0.5;
a2 = 0.5;
theta4 = 0*pi/180;
d3 = 0.1;
d4 = 0.1;

% Inverse
q1 = []; q2 = [];
for i=0:180,
    ox = 0.2 + 0.2*cos(i*pi/180);
    oy = 0.0 + 0.2*sin(i*pi/180);
    oz = -0.1; 

    c2 = (ox^2 + oy^2 - a1^2 - a2^2)/(2*a1*a2);
    theta2 = atan2(sqrt(1-c2^2),c2);
    %theta2 = atan2(-sqrt(1-c2^2),c2)
    s2 = sin(theta2);
    theta1 = atan2(oy,ox) - atan2(a2*s2,a1+a2*c2);
    %d3 = -(oz + d4);
    q1 = [q1 theta1];
    q2 = [q2 theta2];
end  

% Forward
x = []; y = []; z = [];
for i=1:length(q1),
    s1 = sin(q1(i)); c1 = cos(q1(i));
    s2 = sin(q2(i)); c2 = cos(q2(i));
    s4 = sin(theta4); c4 = cos(theta4);

    A1 = [c1 -s1 0 a1*c1;
          s1  c1 0 a1*s1;
          0   0  1   0;
          0   0  0   1];
    A2 = [c2 -s2 0 a2*c2;
          s2  c2 0 a2*s2;
          0   0  1   0;
          0   0  0   1];
    A3 = [1  0  0  0;
          0  1  0  0;
          0  0  1  d3;
          0  0  0  1];
    A4 = [c4 -s4 0 0;
          s4  c4 0 0;
          0   0  1 d4;
          0   0  0 1];  
  
    A = A1*A2*A3*A4;  

    x = [x A(1,4)];
    y = [y A(2,4)];
    z = [z A(3,4)];
end

figure
plot(q1,'r')
hold on
plot(q2,'b')

figure
hold on
grid on
for i=1:length(q1),
    plot(x(i),y(i),'.');
    pause(0.1);
end    

      
      
    
