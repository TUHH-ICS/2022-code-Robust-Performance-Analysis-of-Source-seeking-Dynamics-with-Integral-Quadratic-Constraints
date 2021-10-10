%% Plot results 
% save_folder='.\data_mass_friction';
% fig_name=' LPV mass with varying kd';

% fig_name=' quad with two modes';
% save_folder='.\data_quadrotor';
            
% Convergence rate estimates for different multiplier
figure()
plot_data_perf([save_folder,'\mult_flag_causal_71'],'g--')
hold on    
plot_data_perf([save_folder,'\mult_flag_anti_causal_69'],'b-.')
plot_data_perf([save_folder,'\mult_flag_non_causal_70'],'c')
plot_data_perf([save_folder,'\mult_flag_non_causal_702'],':')
plot_data_perf([save_folder,'\mult_flag_non_causal_703'],':')
plot_data_perf([save_folder,'\mult_flag_non_causal_704'],':')
plot_data_perf([save_folder,'\mult_flag_non_causal_705'],':')
plot_data_perf([save_folder,'\lb_lin'],'k*')
legend('ZF causal (P_1=0)','ZF anti-causal (P_3=0)','ZF order 1','ZF order 2','ZF order 3','ZF order 4','ZF order 5','Example fields')
xlabel('L')
ylabel('\alpha')
ylim([0,0.5])
title(['Convergence rates(exponents)',fig_name])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function []=plot_data_perf(file,plot_style)
    save_path=['.\',file];
    data=load(save_path,'alpha_best','L');
    alpha_best=data.alpha_best;
    L=data.L;    
    plot(L,alpha_best(1,:),plot_style,'LineWidth',1)    
end
