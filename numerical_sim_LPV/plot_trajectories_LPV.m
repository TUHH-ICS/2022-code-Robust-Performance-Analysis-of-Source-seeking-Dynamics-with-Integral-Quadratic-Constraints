%% Plot
clear
clc
%%
ZF_data=load('.\data\LPV_trajectories');
% Plot trajectories on a linear scale
figure()
plot(ZF_data.time,ZF_data.trajs.y(1,:))
hold on
plot(ZF_data.time,ZF_data.trajs.y(2,:))
legend('x ZF','y ZF')
xlabel('time')
ylabel('x')

% Plot trajectories on a linear scale
figure()
plot(ZF_data.time,ZF_data.e_norms)
legend('e norm CC','e norm ZF')
xlabel('time')
ylabel('pos error')

% Plot trajectories on a logarithmic scale and compare with the estimated
% rate
%LMI_rate=0.454545;
Fit = polyfit(ZF_data.time(1,1:40000),log(ZF_data.e_norms(1,1:40000)),1); % x = x data, y = y data, 1 = order of the polynomial i.e a straight line 
alpha_num=-Fit(1)/2;
LMI_rate=0.214844;
estimate=log(ZF_data.e_norms(1,1))-2*LMI_rate*ZF_data.time;
fit=Fit(2)+Fit(1)*ZF_data.time;
figure()
plot(ZF_data.time,log(ZF_data.e_norms))
hold on
plot(ZF_data.time,fit)
plot(ZF_data.time,estimate)
ylim([-50,50])
legend('actual','best_fit','e norm ZF')
xlabel('time')
ylabel('ln(pos error)')
%% Plot trajectories with the scalar field
figure()
dim=2;
switch dim
    case 1        
        plot(ZF_data.time,ZF_data.trajs.x(1,:))
        legend('Trajectory CC','Trajectory ZF')
    case 2                
        plot(ZF_data.trajs.y(1,:),ZF_data.trajs.y(2,:))
        hold on
        plot(ZF_data.trajs.y(1,1),ZF_data.trajs.y(2,1),'o')
        plot(ZF_data.trajs.y(1,end),ZF_data.trajs.y(2,end),'X')
        contour(ZF_data.X,ZF_data.Y,ZF_data.Z,20)
        legend('Trajectory CC','Trajectory ZF')        
end