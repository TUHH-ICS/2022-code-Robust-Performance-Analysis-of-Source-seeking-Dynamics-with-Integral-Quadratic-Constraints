function [G_veh]=define_G_quad_LPV_wrapped(dim,kp,kd,mass)
% This function defines a closed-loop quadrotor model that has flocking
% force as input and it preserves the integral action

% Define double integrator to generate references
if dim~=2
    error('Currently just works with dimension=2')
end

inv_mass_1=(1/mass(1));
A1=[0 1; 0 -kd(1)*inv_mass_1]; 
B1=[0;1*inv_mass_1*kp(1)];

inv_mass_2=(1/mass(2));
A2=[0 1; 0 -kd(2)*inv_mass_2]; 
B2=[0;1*inv_mass_2*kp(2)];

% Higher dimensional vehicles
A_hat1=kron(A1,eye(dim));
A_hat2=kron(A2,eye(dim));

B_hat1=kron(B1,eye(dim));
B_hat2=kron(B2,eye(dim));

C_hat=eye(2*dim);
D_hat=zeros(2*dim,dim);


G_double_int_1=ss(A_hat1,B_hat1,C_hat,D_hat);
G_double_int_2=ss(A_hat2,B_hat2,C_hat,D_hat);

G_quad_cl=get_quad_G_cl_LPV(mass);

% Compare the wrapped plant with the generic vehicle model
% sigma(G_double_int)
% hold on
% sigma(G,'r')

G_veh.G1=G_quad_cl.G1*G_double_int_1;
G_veh.G2=G_quad_cl.G2*G_double_int_2;
end