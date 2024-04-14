
alpha = sym('alpha','real');
beta = sym('beta','real');
gamma = sym('gamma','real');

q = [alpha;beta;gamma];

r_BF_inB = [...
    - sin(beta + gamma) - sin(beta);...
  sin(alpha)*(cos(beta + gamma) + cos(beta) + 1) + 1;...
  -cos(alpha)*(cos(beta + gamma) + cos(beta) + 1)];


% determine the foot point Jacobian J_BF_inB=d(r_BF_inB)/dq
J_BF_inB = [diff(r_BF_inB,alpha),diff(r_BF_inB,beta),diff(r_BF_inB,gamma)];

% what generalized velocity dq do you have to apply in a configuration q = [0;60°;-120°]
% to lift the foot in vertical direction with v = [0;0;-1m/s];

new_J = double(subs(J_BF_inB,{alpha,beta,gamma},{0,60,-120}))
dq = inv(new_J)*[0;0;-1]
