% This script is used to plot the robustness of the source-seeking
% algorithms to fields with increasing condition number
close all
clear
clc
addpath('..\analysis_scripts')
save_data=0;
%% Setup optimization

% Sector bounds
m=1; % lower bound on the sector
L=1:0.2:9;  % Upper bound on the sector
n_L=length(L);
cvx_tol=1e-3;
bisect_tol=1e-3;
cond_tol=1e8;

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
multiplier_flag=[1,60,61,59];
for i=1:4
    switch multiplier_flag(1,i)
    case 1
        multiplier_class.id=1;
        save_path=['.\data\mult_flag_CC_',num2str(multiplier_flag(1,i))];
    case 60  
        multiplier_class.id=6;
        multiplier_class.rho=-1;
        multiplier_class.psi_order=1;
        multiplier_class.odd_flag=0;
        multiplier_class.causal_flag=0; % 1: causal, -1:anti-causal, 0:non-causal
        save_path=['.\data\mult_flag_non_causal_',num2str(multiplier_flag(1,i))];
    case 61  
        multiplier_class.id=6;
        multiplier_class.rho=-1;
        multiplier_class.psi_order=1;
        multiplier_class.odd_flag=0;
        multiplier_class.causal_flag=1; % 1: causal, -1:anti-causal, 0:non-causal
        save_path=['.\data\mult_flag_causal_',num2str(multiplier_flag(1,i))];
    case 59  
        multiplier_class.id=6;
        multiplier_class.rho=-1;
        multiplier_class.psi_order=1;
        multiplier_class.odd_flag=0;
        multiplier_class.causal_flag=-1; % 1: causal, -1:anti-causal, 0:non-causal
        save_path=['.\data\mult_flag_anti_causal_',num2str(multiplier_flag(1,i))];
    end    
    alpha_lims=[0,10]; % Initial range for the bisection algorithm
    [alpha_best]=sweep_L(G_veh,m,L,alpha_lims,cond_tol,cvx_tol,bisect_tol,multiplier_class);
    save(save_path);
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
function [alpha_best]=sweep_L(G_veh,m,L,alpha_lims,cond_tol,cvx_tol,bisect_tol,multiplier_class)
    n_L=size(L,2);    
    alpha_best=zeros(1,n_L);    
    for j=1:n_L
            if j>1 && alpha_best(1,j-1)==-1
                alpha_best(1,j)=-1;            
            else       
                L_curr=L(j);
                [alpha_best(1,j),~]=bisection_exponent(G_veh,m,L_curr,alpha_lims,cond_tol,cvx_tol,bisect_tol,multiplier_class);
            end
    end    
end
