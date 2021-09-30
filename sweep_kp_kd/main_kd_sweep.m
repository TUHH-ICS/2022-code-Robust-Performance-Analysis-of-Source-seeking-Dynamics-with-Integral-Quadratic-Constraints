% This script is used to reproduce data for Example 7_20 from Scherer and
% Weiland's LMI notes. It is an exercise with an odd sector bounded
% non-linearity
close all
clear
clc
addpath('..\analysis_scripts')
addpath(genpath('..\vehicles\quadrotor'))
%% Setup optimization

% Sector bounds
m=1; % lower bound on the sector
L=10;  % Upper bound on the sector

% Optimization properties
cvx_tol=1e-3;
bisect_tol=1e-3;
cond_tol=100000000;


kp=1;
kd=[1:0.5:14,16:2:30];



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
    [alpha_best]=sweep_kp_kd(m,L,kp,kd,alpha_lims,cond_tol,cvx_tol,bisect_tol,multiplier_class);
    save(save_path);
end
%% Plot data
plot_data
%% Functions
% This functions sweeps kp,kd and finds the best covergence rate estimate 
% by running a bisection algorithm for each fixed kp,kd
function [alpha_best]=sweep_kp_kd(m,L,kp,kd,alpha_lims,cond_tol,cvx_tol,bisect_tol,multiplier_class)    
    n_p=length(kp);
    n_d=length(kd);
    alpha_best=zeros(n_p,n_d);   
    for i=1:n_p
        for j=1:n_d        
            kp_curr=kp(i);
            kd_curr=kd(j);
            % Quadrotor dynamics 
            % Current implementation only supports dim=2 for quadrotors
            dim=2;% spatial dimension (of positions and velocities)
            G_veh=define_G_quad_wrapped(dim,kp_curr,kd_curr); 
            [alpha_best(i,j),~]=bisection_exponent(G_veh,m,L,alpha_lims,cond_tol,cvx_tol,bisect_tol,multiplier_class);        
        end        
    end   
end