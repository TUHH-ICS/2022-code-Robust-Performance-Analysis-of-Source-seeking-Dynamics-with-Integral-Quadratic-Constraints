function G_quad_wrapped=get_quad_G_cl_LPV(m)
% This function defines the quadrotor model, designs a tracking controller
% and returns wrapped up closed loop of the quadrotor that takes as input
% positions and velocities and outputs quadrotor positions.
% -------------------------------------------------------------------------
    %% Define the model of the quadrocopter
    g = 9.81;   % Gravity constant
    %m = 0.640;  %  Mass of the Quadrocopter
    A = [ 0  1  0  0  0  0  0  0  0  0  0  0   ;
          0  0  0  0  0  0  0  0 -g  0  0  0   ;
          0  0  0  1  0  0  0  0  0  0  0  0   ;
          0  0  0  0  0  0  0  0  0  0  g  0   ;
          0  0  0  0  0  1  0  0  0  0  0  0   ;
          0  0  0  0  0  0  0  0  0  0  0  0   ;
          0  0  0  0  0  0  0  1  0  0  0  0   ;
          0  0  0  0  0  0  0  0  0  0  0  0   ;
          0  0  0  0  0  0  0  0  0  1  0  0   ;
          0  0  0  0  0  0  0  0  0  0  0  0   ;
          0  0  0  0  0  0  0  0  0  0  0  1   ;
          0  0  0  0  0  0  0  0  0  0  0  0 ] ;
    
    % Note the transpose at the end of B
    B1 = [ 0  0  0  0  0 1/m(1) 0  0  0  0  0  0   ;
          0  0  0  0  0  0  0   1  0  0  0  0   ;
          0  0  0  0  0  0  0   0  0  1  0  0   ;
          0  0  0  0  0  0  0   0  0  0  0  1 ]';
    % Note the transpose at the end of B
    B2 = [ 0  0  0  0  0 1/m(2) 0  0  0  0  0  0   ;
          0  0  0  0  0  0  0   1  0  0  0  0   ;
          0  0  0  0  0  0  0   0  0  1  0  0   ;
          0  0  0  0  0  0  0   0  0  0  0  1 ]';
    C = eye(12);  
    D = zeros(12,4);
    P1 = ss(A,B1,C,D);
    P2 = ss(A,B2,C,D);
    ref_dim=6; % Position and velocity references
    %% Design a tracking controller
    % Hinf design
    %G_quad_wrapped = hinf_design_2DOF_four_block(P,ref_dim);

    G_quad_wrapped.G1=lqr_design_with_static_FF(P1,ref_dim);
    G_quad_wrapped.G2=lqr_design_with_static_FF(P2,ref_dim);
    
end