% This script is used to plot the robustness of the source-seeking
% algorithms to fields with increasing condition number
close all
clear
clc
addpath('..\analysis_scripts')
save_data=1;
%% Setup optimization
example=3;
switch example
    case 1
        % Example from Chen and Wen(not a vehicle): the optimal multiplier method
        % for nonlinear robustness analysis
        G_veh=tf([3,3],[1,1,25,0,0]);
        % Sector bounds
        m=1; % lower bound on the sector
        L=1:0.1:3;  % Upper bound on the sector
        n_L=length(L);
    case 2
        % Example from Chen and Wen(not a vehicle): the optimal multiplier method
        % for nonlinear robustness analysis
        G_veh=tf([1,1],[1,1,25,0,0]);
        % Sector bounds
        m=1; % lower bound on the sector
        L=1:0.1:3;  % Upper bound on the sector
        n_L=length(L);
    case 3
        % Example from Scherer and Weiland, LMI notes in control
        a=1;
        G_veh=ss(tf([1,-a,0],[1,1,2,1]));            
        % Sector bounds
        m=0; % lower bound on the sector
        L=0.1:0.1:2;  % Upper bound on the sector
        n_L=length(L);
    case 4
        % Example from Veenman, Scherer and Koroglu (not a vehicle)
        A=[-0.4,    -1;...
            1,      0];
        B=[-0.2,    -1,     -0.25;...
            0,      0,      0];
        C=[1,   0;...
           0,   1;...
           0,   0];
        D=zeros(3);D(3,2)=1;
        G_veh=ss(A,B,C,D);
        % Sector bounds
        m=1; % lower bound on the sector
        L=1:0.1:3;  % Upper bound on the sector
        n_L=length(L);
    case 5
        % Example from Fetzer and Scherer page 3389 Example 4.9 (not a vehicle)
        A=[-4,      -3,     0;...
            2,      0,      0;...
            -1,     -1,     -2];
        B=-[0,   4,      1,      3;...
           2,   0,      3,      1;...
           1,   0,      3,      1];
        C=[-0.1,    -0.2,      1;...
           -1,      -0.3,      0.1;...
           -0.2,    0.1,       1;...
           0.1,    -0.2,       0.2;];
        D=zeros(4);
        G_veh=ss(A,B,C,D);
        % Sector bounds
        m=1; % lower bound on the sector
        L=1:0.1:3;  % Upper bound on the sector
        n_L=length(L);
end


% Optimization tolerences
tolerences.cvx_tol=1e-3;
tolerences.bisect_tol=1e-3;
tolerences.cond_tol=1e8;


% Multiplier class
% Select a multiplier class from the following choices
% 1. Circle criterion
% 2. Full block circle criterion
% 3. Zames Falb multipliers
multiplier_flag=[1,60,61,59];
for i=1:4
    alpha_best=zeros(1,size(L,2));
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
    G_cl=feedback(G_veh,L(1,i),-1);
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
