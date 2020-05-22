phi
theta1 = i*pi/180;
s1 = sin(theta1); c1 = cos(theta1);
s2 = sin(theta2); c2 = cos(theta2);
s3 = sin(theta3); c3 = cos(theta3);

Rz = [c1 -s1 0 ;
      s1  c1 0 ;
      0   0  1];
Ry = [ c2 0 -s2;
       0  1  0 ;
      -s2 1 c2]; 
Rx = [c3 -s3 0 ;
      s3  c3 0 ;
      0   0  1];  