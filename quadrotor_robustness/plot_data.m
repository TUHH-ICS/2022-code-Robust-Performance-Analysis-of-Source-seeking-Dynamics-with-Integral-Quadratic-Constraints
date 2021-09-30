%% Plot results for Example 7 20 from Scherer and Weilands LMI notes
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
savefig('quadrotor_robustness')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function []=plot_data_perf(file,plot_style)
    save_path=['.\',file];
    data=load(save_path,'alpha_best','L');
    alpha_best=data.alpha_best;
    L=data.L;    
    plot(L,alpha_best(1,:),plot_style,'LineWidth',1)    
end
