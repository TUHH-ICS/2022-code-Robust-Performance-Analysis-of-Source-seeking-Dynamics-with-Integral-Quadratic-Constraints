%% Plot results for Example 7 20 from Scherer and Weilands LMI notes
clear
clc
%% Convergence rate estimates for different multiplier
figure()
%plot_data_perf('.\data\mult_flag_CC_1','ro')
   
plot_data_perf('.\data\mult_flag_order_1_non_causal_701','g--')
hold on 
plot_data_perf('.\data\mult_flag_order_2_non_causal_702','b-.')
plot_data_perf('.\data\mult_flag_order_3_non_causal_703','c')
plot_data_perf('.\data\lb_lin','k*')
legend('ZF non-causal (order=1)','ZF non-causal (order=2)','ZF non-causal (order=3)','Example fields')
xlabel('L')
ylabel('\alpha')
ylim([0,0.5])
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
