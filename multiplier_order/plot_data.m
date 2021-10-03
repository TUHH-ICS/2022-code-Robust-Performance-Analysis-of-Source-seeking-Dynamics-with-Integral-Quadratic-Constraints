%% Plot results for Example 7 20 from Scherer and Weilands LMI notes
clear
clc
%% Convergence rate estimates for different multiplier
figure()
plot_data_perf('.\data\mult_flag_CC_1','ro')
hold on    
plot_data_perf('.\data\mult_flag_order_1_anti_causal_601','g--')
plot_data_perf('.\data\mult_flag_order_2_anti_causal_602','b-.')
plot_data_perf('.\data\mult_flag_order_3_anti_causal_603','c')
plot_data_perf('.\data\lb_lin','k*')
legend('CC(order=0)','ZF anti causal (order=1)','ZF anti causal (order=2)','ZF anti causal (order=3)','Example fields')
xlabel('L')
ylabel('\alpha')
%ylim([0,0.5])
title('Convergence rates(exponents) for mass with friction')
savefig('multiplier_order_comparison')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function []=plot_data_perf(file,plot_style)
    save_path=['.\',file];
    data=load(save_path,'alpha_best','L');
    alpha_best=data.alpha_best;
    L=data.L;    
    plot(L,alpha_best(1,:),plot_style,'LineWidth',1)    
end
