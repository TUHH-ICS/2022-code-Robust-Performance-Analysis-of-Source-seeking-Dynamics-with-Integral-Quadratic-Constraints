%---------------------------------------------------------------------------------------------------
% For Paper
% "Robust Performance Analysis of Source-Seeking Dynamics with Integral Quadratic Constraints"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script is used to generate data and plot the results for the LPV
% examples presented in Fig. 8 and Fig. 9 in the above paper.  


close all
clear
clc
save_data=1;
addpath(genpath('..\analysis_scripts'))
%% Setup optimization

% Sector bounds
m=1; % lower bound on the sector

% Optimization tolerences
tolerences.cvx_tol=1e-3; % Tolerence for definiteness in LMIs
tolerences.bisect_tol=1e-3; % Tolerence in alpha for the bisect-algorithm 
tolerences.cond_tol=1e8; % Tolerence for bounding the cond no of positive def variables


for eg=1:2
    example=eg;
    % Examples
    % 1. LPV example: Mass with varying kd
    % 2. Uncertain quadrotor with varying quadrotor mass
    switch example
        case 1
            % % Mass with friction dynamics
            kp=[1,1];
            % Varying parameter: 1: mass, 2: kd
            varying_param=2;
            switch varying_param
                case 1               
                    kd=[1,1];
                    mass=[0.8,1.2];
                case 2                     
                    kd=[0.8,1.2];
                    mass=[1,1];
            end
            L=1:2:30;  % Upper bound on the sector
            n_L=length(L);
            dim=2;% spatial dimension (of positions and velocities)
            addpath(genpath('..\vehicles\LPV_models'))
            G_veh=define_G_mass_with_friction_LPV_affine(dim,kd,mass,kp);
            fig_name=' LPV mass with varying kd';
            save_folder='.\data_mass_friction';
        case 2
            % Quadrotor dynamics 
            addpath(genpath('..\vehicles\quadrotor'))
            addpath(genpath('..\vehicles\LPV_models'))
            kp=[1,1]; kd=[5,5]; % Pre-filter gains
            mass=[0.2,2];
            L=1:0.5:10;  % Upper bound on the sector
            n_L=length(L);
            dim=2;% spatial dimension (of positions and velocities)
            % Current implementation only supports dim=2 for quadrotors
            G_veh=define_G_quad_LPV_wrapped(dim,kp,kd,mass); 
            fig_name=' quad with two modes';
            save_folder='.\data_quadrotor';
    end

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
    
    multiplier_flag=[70,71,69,702,703,704,705];
    for i=1:7
        switch multiplier_flag(1,i)        
        case 70  
            multiplier_class.id=7;
            multiplier_class.rho=-1;
            multiplier_class.psi_order=1;
            multiplier_class.odd_flag=0;
            multiplier_class.causal_flag=0; % 1: causal, -1:anti-causal, 0:non-causal
            save_path=[save_folder,'\mult_flag_non_causal_',num2str(multiplier_flag(1,i))];
        case 71  
            multiplier_class.id=7;
            multiplier_class.rho=-1;
            multiplier_class.psi_order=1;
            multiplier_class.odd_flag=0;
            multiplier_class.causal_flag=1; % 1: causal, -1:anti-causal, 0:non-causal
            save_path=[save_folder,'\mult_flag_causal_',num2str(multiplier_flag(1,i))];
        case 69  
            multiplier_class.id=7;
            multiplier_class.rho=-1;
            multiplier_class.psi_order=1;
            multiplier_class.odd_flag=0;
            multiplier_class.causal_flag=-1; % 1: causal, -1:anti-causal, 0:non-causal
            save_path=[save_folder,'\mult_flag_anti_causal_',num2str(multiplier_flag(1,i))];
        case 702  
            multiplier_class.id=7;
            multiplier_class.rho=-1;
            multiplier_class.psi_order=2;
            multiplier_class.odd_flag=0;
            multiplier_class.causal_flag=0; % 1: causal, -1:anti-causal, 0:non-causal
            save_path=[save_folder,'\mult_flag_non_causal_',num2str(multiplier_flag(1,i))];
        case 703  
            multiplier_class.id=7;
            multiplier_class.rho=-1;
            multiplier_class.psi_order=3;
            multiplier_class.odd_flag=0;
            multiplier_class.causal_flag=0; % 1: causal, -1:anti-causal, 0:non-causal
            save_path=[save_folder,'\mult_flag_non_causal_',num2str(multiplier_flag(1,i))];
        case 704  
            multiplier_class.id=7;
            multiplier_class.rho=-1;
            multiplier_class.psi_order=4;
            multiplier_class.odd_flag=0;
            multiplier_class.causal_flag=0; % 1: causal, -1:anti-causal, 0:non-causal
            save_path=[save_folder,'\mult_flag_non_causal_',num2str(multiplier_flag(1,i))];
        case 705  
            multiplier_class.id=7;
            multiplier_class.rho=-1;
            multiplier_class.psi_order=5;
            multiplier_class.odd_flag=0;
            multiplier_class.causal_flag=0; % 1: causal, -1:anti-causal, 0:non-causal
            save_path=[save_folder,'\mult_flag_non_causal_',num2str(multiplier_flag(1,i))];    
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
        G_cl1=feedback(G_veh.G1,H,-1);
        G_cl2=feedback(G_veh.G2,H,-1);
        alpha1=-1*max(real(pole(G_cl1)));
        alpha2=-1*max(real(pole(G_cl2)));
        alpha_best(1,i)=min(alpha1,alpha2);
    end
    save([save_folder,'\lb_lin']);
    %% Plot data
    plot_data
end
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