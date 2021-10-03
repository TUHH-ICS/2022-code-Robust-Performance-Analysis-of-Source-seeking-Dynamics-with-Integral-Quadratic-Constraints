% This script is used to plot the robustness of the source-seeking
% algorithms to fields with increasing condition number
close all
clear
clc
save_data=1;
%% Setup optimization

% Sector bounds
m=1; % lower bound on the sector
L=1:1:3;  % Upper bound on the sector
n_L=length(L);

% Optimization tolerences
tolerences.cvx_tol=1e-3;
tolerences.bisect_tol=1e-3;
tolerences.cond_tol=1e8;

% % Mass with friction dynamics
% addpath(genpath('..\vehicles\two_mass_vehicle_model'))
% rng(1)
% kp=1;
% kd=1;
% dim=2;% spatial dimension (of positions and velocities)
% G_veh=define_G_two_mass_model_wrapped(dim,kd,kp);

% Quadrotor dynamics 
addpath(genpath('..\vehicles\quadrotor'))
dim=2;% spatial dimension (of positions and velocities)
% Current implementation only supports dim=2 for quadrotors
kp=1;kd=5;
G_veh=define_G_quad_wrapped(dim,kp,kd);       

% Multiplier class
% Select a multiplier class from the following choices
% 1. Circle criterion
% 2. Full block circle criterion
% 3. Zames Falb multipliers
multiplier_flag=[1,601,602,603];
for i=1:4
    switch multiplier_flag(1,i)
    case 1
        multiplier_class.id=1;
        save_path=['.\data\mult_flag_CC_',num2str(multiplier_flag(1,i))];
    case 601  
        multiplier_class.id=6;
        multiplier_class.rho=-1;
        multiplier_class.psi_order=1;
        multiplier_class.odd_flag=0;
        multiplier_class.causal_flag=-1; % 1: causal, -1:anti-causal, 0:non-causal
        save_path=['.\data\mult_flag_order_1_anti_causal_',num2str(multiplier_flag(1,i))];
    case 602  
        multiplier_class.id=6;
        multiplier_class.rho=-1;
        multiplier_class.psi_order=2;
        multiplier_class.odd_flag=0;
        multiplier_class.causal_flag=-1; % 1: causal, -1:anti-causal, 0:non-causal
        save_path=['.\data\mult_flag_order_2_anti_causal_',num2str(multiplier_flag(1,i))];
    case 603  
        multiplier_class.id=6;
        multiplier_class.rho=-1;
        multiplier_class.psi_order=3;
        multiplier_class.odd_flag=0;
        multiplier_class.causal_flag=-1; % 1: causal, -1:anti-causal, 0:non-causal
        save_path=['.\data\mult_flag_order_3_anti_causal_',num2str(multiplier_flag(1,i))];    
    end    
    alpha_lims=[0,10]; % Initial range for the bisection algorithm
    [alpha_best]=sweep_L(G_veh,m,L,alpha_lims,tolerences,multiplier_class);
    if save_data==1 
        save(save_path); 
    end
end
% Generate example quadratic fields (Linear feedback) to test conservatism
alpha_best=zeros(1,n_L);
for i=1:n_L
    H=[m,0;0,L(1,i)];
    G_cl=feedback(G_veh,H,-1);
    alpha_best(1,i)=-1*max(real(pole(G_cl)));
end
save('.\data\lb_lin');
%% Plot data
plot_data
%% Functions
% This functions sweeps L and finds the best covergence rate estimate by
% running a bisection algorithm for each fixed L
function [alpha_best]=sweep_L(G_veh,m,L,alpha_lims,tolerences,multiplier_class)
    n_L=size(L,2);    
    alpha_best=zeros(1,n_L);    
    for j=1:n_L
            if j>1 && alpha_best(1,j-1)==-1
                alpha_best(1,j)=-1;            
            else       
                L_curr=L(j);
                [alpha_best(1,j),~]=bisection_exponent(G_veh,m,L_curr,alpha_lims,tolerences,multiplier_class);
            end
    end    
end