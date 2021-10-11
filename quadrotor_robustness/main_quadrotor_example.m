%---------------------------------------------------------------------------------------------------
% For Paper
% "Robust Performance Analysis of Source-Seeking Dynamics with Integral Quadratic Constraints"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script is used to generate data and plot the results for the LTI
% quadrotor example presented in Fig.4 in the above paper.  
close all
clear
clc
addpath('..\analysis_scripts')
save_data=1;
%% Setup optimization

% Sector bounds
m=1; % lower bound on the sector
L=1:0.2:9;  % Upper bound on the sector
n_L=length(L);

% Optimization tolerences
tolerences.cvx_tol=1e-3; % Tolerence for definiteness in LMIs
tolerences.bisect_tol=1e-3; % Tolerence in alpha for the bisect-algorithm 
tolerences.cond_tol=1e8; % Tolerence for bounding the cond no of positive def variables

% Define quadrotor dynamics 
addpath(genpath('..\vehicles\quadrotor'))
dim=2;% spatial dimension (of positions and velocities)
% Current implementation only supports dim=2 for quadrotors
kp=1;kd=5; % gains for the pre-filter
G_veh=define_G_quad_wrapped(dim,kp,kd);       

% Run the analysis for different cases defined in the multiplier structure
% with the following properties:

% id: 
% This determines the kind of multiplier used with the following choices
%     1. Circle criterion
%     6. Zames Falb multipliers with analysis LMIs for LTI systems
%     7. Zames Falb multipliers with analysis LMIs for LPV systems

% rho:
% This is valid only for Zames Falb multipliers and is the pole location
% for the basis functions parameterizing the multiplier

% psi_order:
% This is valid only for Zames Falb multipliers and is the order of the
% multiplier that is being searched over

% odd_flag:
% This is valid only for Zames Falb multipliers and is set to one if the
% non-linearity under consideration is odd and is set to 0 otherwise

% causal_flag:
% This is valid only for Zames Falb multipliers. It should be set to 1 if
% restricting the search to causal multipliers, set to -1 is restricting
% the search to anti-causal multipliers and set to 0 is searching over
% general non-causal multipliers which includes causal and non-causal
% parts.
    
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
function [alpha_best]=sweep_L(G_veh,m,L,alpha_lims,tolerences,multiplier_class)
% This functions sweeps L and finds the best covergence rate estimate by
% running a bisection algorithm for each fixed L
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
