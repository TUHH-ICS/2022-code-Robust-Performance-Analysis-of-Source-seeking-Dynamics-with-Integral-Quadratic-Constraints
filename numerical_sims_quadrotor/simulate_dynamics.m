% This script numerically simulates the dynamics
close all
clear
clc
%% Vehicle Dynamics
% Select a Vehicle model from the following choices
% 1. Mass with friction
% 2. Quadrotor dynamics
% 3. Example from Scherer and Weiland
% 4. Example that shows the benefit of non-causal multipliers
Veh_mod=2;
switch(Veh_mod)
    case 1
        % Mass with friction dynamics
        addpath(genpath('.\vehicles\mass_with_friction'))
        rng(1)
        c_damp=1+rand;mass=1+rand;step_size=1;
        dim=2;% spatial dimension (of positions and velocities)
        G_veh=define_G_mass_with_friction_wrapped(dim,c_damp,mass,step_size);
    case 2
        % Quadrotor dynamics
        addpath(genpath('.\vehicles\quadrotor'))
        dim=2;% spatial dimension (of positions and velocities)
        % Current implementation only supports dim=2 for quadrotors
        kp=1;kd=18;
        G_veh=define_G_quad_wrapped(dim,kp,kd);  
    case 3
        % Example from Scherer, Weiland LMI notes (not a vehicle)
        a=1; % Choose between 0.2 to 2          
        G_veh=ss(tf([1,-a,0],[1,1,2,1]));
        dim=1;
    case 4
        % Example of a vehicle type model that shows benefit of non-causal
        % multipliers
        G_veh=-tf([1,-1],[1,1,25,0]);
end

% Sector bounds
m=1; % lower bound on the sector
L=10;  % Upper bound on the sector


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
        Z = 1*(X-y_min(1)).^2+2*(Y-y_min(2)).^2;
        grad_field=@(y) [m,0;0,L]*(y-y_min);
end
% Simulation parameters
sim_time=100;
dt=0.001;
time_steps=sim_time/dt;
switch(Veh_mod)
    case 1
        % Mass with friction dynamics
        pos_ic=10*(-1+2*rand(dim,1));
        vel_ic=2*(-1+2*rand(dim,1));
        x_ic=[pos_ic;vel_ic];
        [trajs]= simulate_source_seek(G_veh,x_ic,grad_field,time_steps,dt);        
    case 2
        % Quadrotor dynamics
        x_ic=0;
        [trajs]= simulate_source_seek_quad(G_veh,grad_field,time_steps,dt);
end
%% Compare the obtained numerical decay with the theoretical decay
x_eqm=trajs.x(:,end);
time=dt*(1:time_steps);
e=trajs.x(:,:)-x_eqm;
e_norms=sum(e.^2);
%% Save data
save(['.\data\kp_',num2str(kp),'_kd_',num2str(kd)])
