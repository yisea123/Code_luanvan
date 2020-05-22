k = 1/sqrt(3)*[1;1;1];
theta = 120*pi/180;
s_ = sin(theta);
c_ = cos(theta);
v_ = 1 - cos(theta);

R_k_theta = [k(1)^2*v_ + c_, k(1)*k(2)*v_ - k(3)*s_, k(1)*k(3)*v_ + k(2)*s_;
             k(1)*k(2)*v_ + k(3)*s_, k(2)^2*v_ + c_, k(2)*k(3)*v_ - k(1)*s_; 
             k(1)*k(3)*v_ - k(2)*s_, k(2)*k(3)*v_ + k(1)*s_, k(3)^2*v_ + c_]  