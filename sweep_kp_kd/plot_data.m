%% Plot results parameter sweep
clear
clc
%% Performance curves
id=1; % represents the fixed value of kp
figure()
plot_data_perf(id,'.\data\mult_flag_CC_1','ro')
hold on    
plot_data_perf(id,'.\data\mult_flag_causal_61','g*')
plot_data_perf(id,'.\data\mult_flag_anti_causal_59','b-.')
plot_data_perf(id,'.\data\mult_flag_non_causal_60','c')
legend('CC($P_1=0,P_3=0$)','ZF causal ($P_1=0$)','ZF anti-causal ($P_3=0$)','ZF')
xlabel('$\frac{k_d}{k_p}$')
ylabel('$\alpha$')
ylim([0,0.2])
title('Convergence rates(exponents)')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function []=plot_data_perf(id,file,line)
    save_path=['.\',file];
    data=load(save_path,'alpha_best','kd','kp');
    alpha_best=data.alpha_best;
    kd=data.kd;
    kp=data.kp;
    kd_kp_ratio=kd./kp(id);
    plot(kd_kp_ratio,alpha_best(id,:),line,'LineWidth',1)    
end