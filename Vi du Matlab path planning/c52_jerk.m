% q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 + a6*t^6 + a7*t^7
q0   = 10; qf  = 45;
q0d  = 0;  qfd = 0;
q0dd = 0; qfdd = 0;
q03d = 0; qf3d = 0;
t0   = 0;  tf  = 1;

A = [1  t0  t0^2  t0^3    t0^4     t0^5     t0^6      t0^7;
     0  1   2*t0  3*t0^2  4*t0^3   5*t0^4   6*t0^5    7*t0^6;
     0  0   2     6*t0    12*t0^2  20*t0^3  30*t0^4   42*t0^5;
     0  0   0     6       24*t0    60*t0^2  120*t0^3  210*t0^4;
     1  tf  tf^2  tf^3    tf^4     tf^5     tf^6      tf^7;
     0  1   2*tf  3*tf^2  4*tf^3   5*tf^4   6*tf^5    7*tf^6;
     0  0   2     6*tf    12*tf^2  20*tf^3  30*tf^4   42*tf^5;
     0  0   0     6       24*tf    60*tf^2  120*tf^3  210*tf^4];
 
b = [q0; q0d; q0dd; q03d; qf; qfd; qfdd; qf3d];

a = inv(A)*b
a0 = a(1); a1 = a(2); a2 = a(3); a3 = a(4); 
a4 = a(5); a5 = a(6); a6 = a(7); a7 = a(8);

t = t0:0.01:tf;
qt   = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5 + a6*t.^6 + a7*t.^7;
qtd  = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4 + 6*a6*t.^5 + 7*a7*t.^6;
qtd  = qtd /2; 
qtdd = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3 + 30*a6*t.^4 + 42*a7*t.^5;
qtdd = qtdd / 10;
qt3d = 6*a3 + 24*a4*t + 60*a5*t.^2 + 120*a6*t.^3 + 210*a7*t.^4;
qt3d = qt3d / 100;
plot(t,qt,'r', t,qtd,'g', t,qtdd,'b', t,qt3d,'m')
grid on
legend('q', 'qdot/2','q2dot/10','q3dot/100')