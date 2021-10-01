function [G_veh]=define_G_mass_with_friction_LPV_affine(dim,kd,mass,kp)
% This function defines a closed-loop vehicle model that has flocking force
% as input and position and velocity as output
% General LTI vehicle models should be in the form 1/s*G_vel where G_vel is
% a velocity tracking controller.

%% Generic second order vehicle: mass with friction in 1D
inv_mass1=1/mass(1);
A1=[0 1; 0 -kd*inv_mass1]; 
B1=[0;1*inv_mass1*kp];

inv_mass2=1/mass(2);
A2=[0 1; 0 -kd*inv_mass2]; 
B2=[0;1*inv_mass2*kp];

C=[1 0];
D=0;
% Higher dimensional vehicles
A_hat1=kron(A1,eye(dim));
A_hat2=kron(A2,eye(dim));

B_hat1=kron(B1,eye(dim));
B_hat2=kron(B2,eye(dim));

C_hat=kron(C,eye(dim));
D_hat=kron(D,eye(dim));

G_veh.G1=ss(A_hat1,B_hat1,C_hat,D_hat);
G_veh.G2=ss(A_hat2,B_hat2,C_hat,D_hat);
end