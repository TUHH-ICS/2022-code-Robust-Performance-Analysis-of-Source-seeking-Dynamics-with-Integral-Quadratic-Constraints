function [trajs]= simulate_source_seek_LPV(G_veh,x_ic,grad_field,time_steps,dt)
% This function simulates the feedback loop (G,grad)
    [A1,B1,C1,D1]=ssdata(G_veh.G1);
    [A2,B2,C2,D2]=ssdata(G_veh.G2);
    nx=size(A1,1);
    [ny,nu]=size(D1);
    x=zeros(nx,time_steps);
    y=zeros(ny,time_steps);
    u=zeros(ny,time_steps);    
    if D1~=zeros(ny,nu)
        error('Vehicle model not strictly proper. (Necessary to avoid algebraic loop)')
    end
    % Initial condition
    x(:,1)=x_ic;    
    y(:,1)=C1*x_ic;
    for i=1:(time_steps-1)
        u(:,i)=-grad_field(y(:,i));
        rand_no=rand();
        [A,B,C,D]=convex_comb(G_veh.G1,G_veh.G2,rand_no);
%         prob_sys1=0;
%         if rand_no<prob_sys1
%             A=A1;B=B1;C=C1;D=D1;
%         elseif rand_no>=prob_sys1
%             A=A2;B=B2;C=C2;D=D2;
%         end
        x(:,i+1)=(eye(nx)+dt*A)*x(:,i)+dt*B*u(:,i);
        y(:,i+1)=C*x(:,i);
    end
    trajs=struct;
    trajs.x=x;
    trajs.y=y;
    trajs.u=u;
end
function [A,B,C,D]=convex_comb(G1,G2,lambda)
[A1,B1,C1,D1]=ssdata(G1);
[A2,B2,C2,D2]=ssdata(G2);
A=convex_combine(A1,A2,lambda);
B=convex_combine(B1,B2,lambda);
C=convex_combine(C1,C2,lambda);
D=convex_combine(D1,D2,lambda);
end
function [A]=convex_combine(A1,A2,lambda)
if lambda>1 || lambda<0
    error('lambda must be betweeen 0 and 1')
end
A=lambda*A1+(1-lambda)*A2;
end