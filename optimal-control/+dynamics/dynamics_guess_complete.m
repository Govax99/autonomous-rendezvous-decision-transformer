function [dx, f, tau] = dynamics_guess_complete(time, state, parameters, control)
% parameters (I don't know if there is a better way to pass them) -> some
% may change with state (orientation of chaser)
[J_C,J_T,m_C,OM] = dynamics.set_parameters(parameters);
OM_IL_L = [0; 0; OM];

p_LC_L = state(1:3);
v_LC_L = state(4:6);
q_LC = state(7:10)./norm(state(7:10));
w_IC_C = state(11:13);
q_LT = state(14:17)./norm(state(14:17));
w_IT_T = state(18:20);

% manage control
if (nargin < 3)
    f = zeros(3,1);
    tau = zeros(3,1);
elseif (isa(control, 'function_handle'))
    c = control(state);
    f = c(1:3);
    tau = c(4:6);
else
    f = control(1:3);
    tau = control(4:6);
end


R_LC = quat.quat2rotm(q_LC);
w_LC_L = R_LC * w_IC_C - OM_IL_L;

A_C = quat.quat_kin_matrix(w_LC_L);

R_LT = quat.quat2rotm(q_LT);
w_LT_L = R_LT * w_IT_T - OM_IL_L;

A_T = quat.quat_kin_matrix(w_LT_L);

dx = [v_LC_L; ...
    1/m_C*(2*OM*v_LC_L(2) + 3*OM^2*p_LC_L(1) + f(1)); ...
    1/m_C*(-2*OM*v_LC_L(1) + f(2)); ...
    1/m_C*(-OM^2*p_LC_L(3) + f(3)); ...
    1/2*A_C*q_LC; ...
    J_C\(-cross(w_IC_C, J_C*w_IC_C) + tau); ...
    1/2*A_T*q_LT; ...
    J_T\(-cross(w_IT_T, J_T*w_IT_T)); ...
    ];

end
