%---------------------------------------------------------------------------------------------------
% For Paper
% "Robust Performance Analysis of Source-Seeking Dynamics with Integral Quadratic Constraints"
% by Adwait Datar and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
% Author(s): Adwait Datar
%---------------------------------------------------------------------------------------------------
% This script is used to plot the data generated by the
% main_quadrotor_example.m to produce Fig. 4 from the above paper
clear
clc
%% Convergence rate estimates for different multiplier
figure()
plot_data_perf('.\data\mult_flag_CC_1','ro')
hold on    
plot_data_perf('.\data\mult_flag_causal_61','g--')
plot_data_perf('.\data\mult_flag_anti_causal_59','b-.')
plot_data_perf('.\data\mult_flag_non_causal_60','c')
plot_data_perf('.\data\lb_lin','k*')
legend('CC(P_1=0,P_3=0)','ZF causal (P_1=0)','ZF anti-causal (P_3=0)','ZF','Example fields')
xlabel('L')
ylabel('\alpha')
ylim([0,0.5])
title('Convergence rates(exponents) for Quadrotor')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function []=plot_data_perf(file,plot_style)
    save_path=['.\',file];
    data=load(save_path,'alpha_best','L');
    alpha_best=data.alpha_best;
    L=data.L;    
    plot(L,alpha_best(1,:),plot_style,'LineWidth',1)    
end
