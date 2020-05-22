% q(t) = a0 + a1*(t-t0) +a2*(t-t0)^2 + a3*(t-t0)^3
q0  = 10; qf  = 45;
q0d = 0;  qfd = 0;
t0  = 0;  tf  = 1;
% q0  = 10; qf  = 45;
% q0d = 12;  qfd = 0;
% t0  = 0;  tf  = 2;

a0 = q0
a1 = q0d
a2 = -(3*q0 -3*qf - 2*t0*q0d - t0*qfd + 2*tf*q0d + tf*qfd) / (tf- t0)^2
a3 = (2*q0 -2*qf - t0*q0d - t0*qfd + tf*q0d + tf*qfd) / (tf- t0)^3

t = t0:0.01:tf;
qt   = a0 + a1*(t-t0) + a2*(t-t0).^2 + a3*(t-t0).^3;
qtd  = a1 + 2*a2*(t-t0) + 3*a3*(t-t0).^2;
qtd  = qtd /2; 
qtdd = (2*a2 + 6*a3*(t-t0));
qtdd = qtdd / 10;
plot(t,qt,'r', t,qtd,'g', t,qtdd,'b' )
legend('q', 'qdot','qddot')
grid on