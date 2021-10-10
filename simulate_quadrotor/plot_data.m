%% Plot
clear
clc
%%
ZF_data=load('.\data\kp_1_kd_9');
CC_data=load('.\data\kp_1_kd_18');
sub_sample=100;
% Plot trajectories on a linear scale
figure()
plot(CC_data.time(1:sub_sample:end),CC_data.trajs.y(1,1:sub_sample:end),'--')
hold on
plot(CC_data.time(1:sub_sample:end),CC_data.trajs.y(2,1:sub_sample:end),'--')
plot(ZF_data.time(1:sub_sample:end),ZF_data.trajs.y(1,1:sub_sample:end))
plot(ZF_data.time(1:sub_sample:end),ZF_data.trajs.y(2,1:sub_sample:end))
legend('x CC','y CC','x ZF','y ZF')
xlabel('time')
ylabel('x')

% Plot trajectories on a linear scale
figure()
plot(CC_data.time,CC_data.e_norms)
hold on
plot(ZF_data.time,ZF_data.e_norms)
legend('e norm CC','e norm ZF')
xlabel('time')
ylabel('pos error')

% Plot trajectories on a logarithmic scale
figure()
plot(CC_data.time,log(CC_data.e_norms))
hold on
plot(ZF_data.time,log(ZF_data.e_norms))
ylim([-50,50])
legend('e norm CC','e norm ZF')
xlabel('time')
ylabel('ln(pos error)')
%% Plot trajectories with the scalar field
figure()
dim=2;
switch dim
    case 1
        plot(CC_data.time,CC_data.trajs.x(1,:))
        hold on
        plot(ZF_data.time,ZF_data.trajs.x(1,:))
        legend('Trajectory CC','Trajectory ZF')
    case 2
        plot(CC_data.trajs.y(1,:),CC_data.trajs.y(2,:))
        hold on
        plot(ZF_data.trajs.y(1,:),ZF_data.trajs.y(2,:))
        
        plot(CC_data.trajs.y(1,1),CC_data.trajs.y(2,1),'o')
        plot(CC_data.trajs.y(1,end),CC_data.trajs.y(2,end),'X')
        contour(CC_data.X,CC_data.Y,CC_data.Z,20)
        legend('Trajectory CC','Trajectory ZF')        
end