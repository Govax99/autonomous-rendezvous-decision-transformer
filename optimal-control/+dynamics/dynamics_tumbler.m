function dx = dynamics_tumbler(time, state, parameters)
%DYNAMICS_TUMBLER Summary of this function goes here
%   Detailed explanation goes here
[~,J_T,~,OM] = dynamics.set_parameters(parameters);
OM_IL_L = [0; 0; OM];

q_LT = state(1:4)./norm(state(1:4));
w_IT_T = state(5:7);

R_LT = quat.quat2rotm(q_LT);
w_LT_L = R_LT * w_IT_T - OM_IL_L;

A_T = quat.quat_kin_matrix(w_LT_L);

dx = [
    1/2*A_T*q_LT; ...
    J_T\(-cross(w_IT_T, J_T*w_IT_T)); ...
    ];
end

