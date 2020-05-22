clc
% q1(t) = a0 + a1*t + a2*t^2
% q2(t) = c0 + c1*t
% q3(t) = b0 + b1*t + b2*t^2 + b3*t^3
q0  = 0; qf  = 60;
q0d = 0;  qcd = 50; qfd = 0;
t0  = 0; t1 = 0.4; t2 = 0.7; tf  = 1;

a0 = q0
a1 = 0;
a2 = qcd/(2*t1)

c0 = q0 - 0.5*t1*qcd
c1 = qcd

A = [1  t2  t2^2  t2^3;
     0  1   2*t2  3*t2^2;
     1  tf  tf^2  tf^3;
     0  1   2*tf  3*tf^2];
 
b = [c0 + c1*t2; c1; qf; qfd];

a = inv(A)*b
b0 = a(1); b1 = a(2); b2 = a(3); b3 = a(4);

qt  = [];
qtd = [];
for t = t0:0.01:(t1-0.01)
  q = a0 + a1*t + a2*t^2;
  qd = a1 + 2*a2*t;
  qt = [qt q];
  qtd = [qtd qd];
end
for t = t1:0.01:(t2-0.01)
  q = c0 + c1*t;
  qd = c1;
  qt = [qt q];
  qtd = [qtd qd];
end
for t = t2:0.01:tf
  q = b0 + b1*t + b2*t^2 + b3*t^3;
  qd = b1 + 2*b2*t + 3*b3*t^2;
  qt = [qt q];
  qtd = [qtd qd];
end
      
t = t0:0.01:tf;
plot(t,qt,'r', t,qtd/2,'g')
grid on
legend('q', 'qdot/2')