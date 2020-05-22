R10 = [0  0  1;
       1  0  0;
       0  1  0]; 
  
Rf0 = [1  0  0;
       0  1  0;
       0  0  1]; 
       
R1f = Rf0'*R10;
phi = acos((trace(R1f) - 1)/2);
%k   = (R1f - R1f')/(2*sin(phi))
k   = [R1f(3,2)-R1f(2,3); 
       R1f(1,3)-R1f(3,1); 
       R1f(2,1)-R1f(1,2)]/(2*sin(phi))
phi = phi*180/pi
       