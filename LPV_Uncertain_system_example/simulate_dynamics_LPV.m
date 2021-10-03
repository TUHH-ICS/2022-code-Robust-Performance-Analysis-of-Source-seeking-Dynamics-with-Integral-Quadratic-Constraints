% This script is used to plot the robustness of the source-seeking
% algorithms to fields with increasing condition number
close all
clear
clc
save_data=1;
%% Setup optimization

% Sector bounds
m=1; % lower bound on the sector
L=20;  % Upper bound on the sector

% Optimization tolerences
tolerences.cvx_tol=1e-3;
tolerences.bisect_tol=1e-3;
tolerences.cond_tol=1e8;

% Mass with friction dynamics
kp=1;
kd=1;
mass=[0.9,1.1];
dim=2;% spatial dimension (of positions and velocities)
G_veh=define_G_mass_with_friction_LPV_affine(dim,kd,mass,kp);


%% Numerically simulate the dynamics
% Define the underlying field for dim=2
range=50;
%y_min=range*(-1+2*rand(dim,1));
y_min=-0.95*[range;range];
switch(dim)
    case 1
        k=min(m,L);
        grad_field=@(y) k*(y-y_min);
    case 2
        x = linspace(-1.5*range,1.5*range);
        y = linspace(-1.5*range,1.5*range);
        [X,Y] = meshgrid(x,y);
        Z = 0.5*(m*(X-y_min(1)).^2+L*(Y-y_min(2)).^2);
        grad_field=@(y) [m,0;0,L]*(y-y_min);
end
% Simulation parameters
sim_time=50;
dt=0.001;
time_steps=sim_time/dt;

% Mass with friction dynamics
pos_ic=10*(-1+2*rand(dim,1));
vel_ic=2*(-1+2*rand(dim,1));
x_ic=[pos_ic;vel_ic];
[trajs]= simulate_source_seek_LPV(G_veh,x_ic,grad_field,time_steps,dt);     

%% Compare the obtained numerical decay with the theoretical decay
x_eqm=trajs.x(:,end);
time=dt*(1:time_steps);
e=trajs.x(:,:)-x_eqm;
e_norms=sum(e.^2);
%% Save data
save(['.\data\LPV_trajectories'])
%%
plot_trajectories_LPV
