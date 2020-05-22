% q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
q0  = 10; qf  = 45;
q0d = 0;  qfd = 0;
q0dd = 0; qfdd = 0;
t0  = 0;  tf  = 1;

A = [1  t0  t0^2  t0^3    t0^4     t0^5;
     0  1   2*t0  3*t0^2  4*t0^3   5*t0^4;
     0  0   2     6*t0    12*t0^2  20*t0^3;
     1  tf  tf^2  tf^3    tf^4     tf^5;
     0  1   2*tf  3*tf^2  4*tf^3   5*tf^4;
     0  0   2     6*tf    12*tf^2  20*tf^3];
 
b = [q0; q0d; q0dd; qf; qfd; qfdd];

a = inv(A)*b;
a0 = a(1); a1 = a(2); a2 = a(3);
a3 = a(4); a4 = a(5); a5 = a(6);

t = t0:0.001:tf;
qt   = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
qtd  = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
qtd  = qtd /2; 
qtdd = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;
qtdd = qtdd / 10;
qt3d = 6*a3 + 24*a4*t + 60*a5*t.^2;
qt3d = qt3d / 100;
plot(t,qt,'r', t,qtd,'g', t,qtdd,'b', t,qt3d,'m')
grid on
legend('q', 'qdot','q2dot','q3dot')
pre_pulse =[];
for i =1:1000
    a = (qt(i+1)-qt(i))*100000/360;
    pre_pulse = [pre_pulse a];
end
temp = 0;
pulse = zeros(1,length(pre_pulse));
for i =1:length(pre_pulse)
    if pre_pulse(i) >= 0.65
        k = pre_pulse - ceil(pre_pulse);
        pulse(i) = ceil(pre_pulse(i));
    else
        pulse(i) = floor(pre_pulse(i));
        k = pre_pulse - floor(pre_pulse);
    end
    temp = temp + k;
    if temp > 1
        pulse(i) = pulse(i) + 1;
        temp = temp -1;
    end
end
t = 0:0.001:0.999;
figure
plot(t,pulse,'r')
