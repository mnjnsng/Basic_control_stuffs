Z_a = -1.05273;
Z_d = -0.0343;
M_a = -2.3294;
M_q = -1.03341;
M_d = -1.1684;
V = 329.127;
w_a = 2*pi*13;
zeta_a = 0.6;

A = [0 1 0 0 0;
    0 Z_a V*Z_a 0 V*Z_d;
    0 M_a/(V*Z_a) M_q (M_d-M_a*Z_d/Z_a) 0;
    0 0 0 0 1;
    0 0 0 -w_a^2 -2*zeta_a*w_a];
B = [0; 0;0;0;w_a^2];

% Full state observation
C = eye(5);
D = [0;0;0;0;0];

% Partial observation
C2 = [1 0 0 0 0;
      0 1 0 0 0;
      0 0 1 0 0;
      0 0 0 1 0];
D2 = zeros(4,1);

sysc = ss(A,B,C,D);
sysd = c2d(sysc,0.2);
Phi = sysd.A;
Gamma = sysd.B;

Q = diag([1,0,0,0,0]);
R = 1;
[K,S,CLP] = lqr(sysc,Q,R);
[Kd,Sd,CLPd] = dlqr(Phi,Gamma,Q,R);
