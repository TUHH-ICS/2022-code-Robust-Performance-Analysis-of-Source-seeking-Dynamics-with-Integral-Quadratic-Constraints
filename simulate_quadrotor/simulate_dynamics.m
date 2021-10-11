%---------------------------------------------------------------------------------------------------
% For Paper
% "Robust Performance Analysis of Source-Seeking Dynamics with Integral Quadratic Constraints"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script is used to generate data and plot the results for the LTI
% quadrotor example presented in Fig.6 in the above paper.  

% This script numerically simulates the dynamics of a quadrotor for optimal
% gains selected using the circle criterion and zames falb multipliers
close all
clear
clc

%% Quadrotor dynamics
addpath(genpath('..\vehicles\quadrotor'))
% Sector bounds
m=1; % lower bound on the sector
L=5;  % Upper bound on the sector

%% Numerically simulate the dynamics
% Define the underlying field for dim=2
range=50;
%y_min=range*(-1+2*rand(dim,1));
y_min=-0.95*[range;range];

x = linspace(-1.5*range,1.5*range);
y = linspace(-1.5*range,1.5*range);
[X,Y] = meshgrid(x,y);
Z = 1*(X-y_min(1)).^2+2*(Y-y_min(2)).^2;
grad_field=@(y) [m,0;0,L]*(y-y_min);

% Simulation parameters
sim_time=100;
dt=0.001;
time_steps=sim_time/dt;
% Quadrotor dynamics
x_ic=0;
for i=1:2
    switch i        
        case 1 % Design with CC
            kp=1;kd=18;            
        case 2 % Design with ZF
            kp=1;kd=9; 
    end
    dim=2;% spatial dimension (of positions and velocities)
    % Current implementation only supports dim=2 for quadrotors
    G_veh=define_G_quad_wrapped(dim,kp,kd);
    [trajs]= simulate_source_seek_quad(G_veh,grad_field,time_steps,dt);
    % Compare the obtained numerical decay with the theoretical decay
    x_eqm=trajs.x(:,end);
    time=dt*(1:time_steps);
    e=trajs.x(:,:)-x_eqm;
    e_norms=sum(e.^2);
    % Save data
    save(['.\data\kp_',num2str(kp),'_kd_',num2str(kd)])
end
%% Plot data
plot_data

