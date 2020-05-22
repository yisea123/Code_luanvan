clc
% q1(t) = a0 + a1*t + a2*t^2 + a3*t^3
% q2(t) = b0 + b1*t + b2*t^2
% q3(t) = c0 + c1*t + c2*t^2
% q4(t) = d0 + d1*t + d2*t^2 + d3*t^3 + d4*t^4
q0   = 10; q1 = 20; q2 = 30; q3 = 35; q4  = 45;
q0d  = 0;                             q4d = 0;
q0dd = 0;                             q4dd = 0;

t0 = 0; t1 = 0.4; t2 = 0.7; t3 = 0.9; t4 = 1;
% from t0 to t1
A = [1  t0  t0^2  t0^3;
     0  1   2*t0  3*t0^2;
     0  0   2     6*t0;
     1  t1  t1^2  t1^3];    
b = [q0; q0d; q0dd; q1];
a = inv(A)*b
a0 = a(1); a1 = a(2); a2 = a(3); a3 = a(4); 

% from t1 to t2
q1d = a1 + 2*a2*t1 + 3*a3*t1^2;
A = [1  t1  t1^2;
     0  1   2*t1;
     1  t2  t2^2];    
b = [q1; q1d; q2];
a = inv(A)*b
b0 = a(1); b1 = a(2); b2 = a(3); 

% from t2 to t3
q2d = b1 + 2*b2*t2;
A = [1  t2  t2^2;
     0  1   2*t2;
     1  t3  t3^2];    
b = [q2; q2d; q3];
a = inv(A)*b
c0 = a(1); c1 = a(2); c2 = a(3); 

% from t3 to t4
q3d = c1 + 2*c2*t3;
A = [1  t3  t3^2  t3^3    t3^4;   
     0  1   2*t3  3*t3^2  4*t3^3;
     1  t4  t4^2  t4^3    t4^4;
     0  1   2*t4  3*t4^2  4*t4^3;
     0  0   2     6*t4    12*t4^2];
 
b = [q3; q3d; q4; q4d; q4dd];

a = inv(A)*b
d0 = a(1); d1 = a(2); d2 = a(3); d3 = a(4); d4 = a(5); 

qt  = [];
qtd = [];
for t = t0:0.01:(t1-0.01),
  q = a0 + a1*t + a2*t^2 + a3*t^3;
  qd = a1 + 2*a2*t + 3*a3*t^2;
  qt = [qt q];
  qtd = [qtd qd];
end
for t = t1:0.01:(t2-0.01),
  q = b0 + b1*t + b2*t^2;
  qd = b1 + 2*b2*t;
  qt = [qt q];
  qtd = [qtd qd];
end
for t = t2:0.01:(t3-0.01),
  q = c0 + c1*t + c2*t^2;
  qd = c1 + 2*c2*t;
  qt = [qt q];
  qtd = [qtd qd];
end
for t = t3:0.01:t4,
  q = d0 + d1*t + d2*t^2 + d3*t^3 + d4*t^4;
  qd = d1 + 2*d2*t + 3*d3*t^2 + 4*d4*t^3;
  qt = [qt q];
  qtd = [qtd qd];
end


t = t0:0.01:t4;
plot(t,qt,'r', t,qtd/5,'b')
grid on
legend('q', 'qdot/5')