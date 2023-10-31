syms du y ytm1;
syms w1 w2 w3;

G = [0.3 0 0; 1.01 0.3 0; 1.507 1.01 0.3];
lambda = 0.7;

f = [0.5*du+1.7*y-0.7*ytm1; 
     0.85*du+2.19*y-1.19*ytm1;
     1.095*du+2.533*y-1.533*ytm1];


K = inv(G'*G+lambda*eye(3))*G';
w = [w1;w2;w3];

K*(w-f);
H = inv(G'*G+lambda*eye(3));

th = 1;