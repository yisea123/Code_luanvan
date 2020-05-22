a1 = 0.5;
a2 = 0.5;
theta1 = 0*pi/180;
theta2 = 0*pi/180;
theta4 = 0*pi/180;
d3 = 0.1;
d4 = 0.1;

x = [];
y = [];
z = [];

for i=-180:180,
    theta2 = i*pi/180;
    s1 = sin(theta1); c1 = cos(theta1);
    s2 = sin(theta2); c2 = cos(theta2);
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
plot(x,y,'.')
grid on
      
      
      
      
      
      
      
      
      
      