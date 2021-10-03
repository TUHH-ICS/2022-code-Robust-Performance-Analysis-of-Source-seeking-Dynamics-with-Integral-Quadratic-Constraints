function [G]=define_G_two_mass_model_wrapped(dim,kd,kp)
% This function defines a closed-loop vehicle model that has flocking force
% as input and position and velocity as output
% General LTI vehicle models should be in the form 1/s*G_vel where G_vel is
% a velocity tracking controller.

%% Generic second order vehicle: mass with friction in 1D
k=1; b=10;
A=[0,   0,  1,      0;...
   0,   0,  0,      1;...
   -k,  k,  -kd-b,  b;...
   k,   -k, b,      -kd-b];
B=[0;0;kp;0];
C=[1,0,0,0];
D=0;
% Higher dimensional vehicles
A_hat=kron(A,eye(dim));
B_hat=kron(B,eye(dim));
C_hat=kron(C,eye(dim));
D_hat=kron(D,eye(dim));

G=ss(A_hat,B_hat,C_hat,D_hat);
end