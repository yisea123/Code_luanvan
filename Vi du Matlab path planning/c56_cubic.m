% q(t) = a0 + a1*t + a2*t^2 + a3*t^3 
q0  = 0; qf  = 120;
q0d = 0;  qfd = 0;
t0  = 0;  tf  = 10;

A = [1  t0  t0^2  t0^3;   
     0  1   2*t0  3*t0^2;  
     1  tf  tf^2  tf^3;
     0  1   2*tf  3*tf^2];
 
b = [q0; q0d; qf; qfd];

a = inv(A)*b
a0 = a(1); a1 = a(2); a2 = a(3); a3 = a(4);

t = t0:0.01:tf;
qt   = a0 + a1*t + a2*t.^2 + a3*t.^3;
qtd  = a1 + 2*a2*t + 3*a3*t.^2;
qtd  = qtd; 
qtdd = 2*a2 + 6*a3*t;
qtdd = qtdd;
plot(t,qt,'r', t,qtd,'g', t,qtdd, 'b')
grid on
legend('q', 'qdot', 'qddot')