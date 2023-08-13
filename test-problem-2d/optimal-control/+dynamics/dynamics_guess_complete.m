function [dx, f, tau] = dynamics_guess_complete(time, state, parameters, control)
%DESCRIPTION state-space form of chaser and target dynamics (can handle control input
% as matlab function, useful for PID control)
% (linear dynamics in L-frame, angular dynamics in C-frame, T-frame)
%
% INPUT:
%    time                current integration time
%    state               current total state [p_LC_L, v_LC_L, q_LC, w_IC_C, q_LT, w_IT_T]
%    parameters          structure containing parameters for the dynamics
%    control             control action [f, tau]
%
% OUTPUT:
%	 dx  	             derivative of state
%    f                   control force (in L-frame)
%    tau                 control torque (in C-frame)
%
% LEGEND FRAMES:
%    L-frame             LVLH, local vertical-local horizontal frame, center on target cm
%    C-frame             relative frame, attached to chaser object, center on chaser cm
%    T-frame             relative frame, attached to target object, center on target cm
%
% MONOGRAM NOTATION:
%    p_XY_Z              p: symbol of physical quantity,
%                        X: "measured from",
%                        Y: target point/frame,
%                        Z: "expressed in" frame
[J_C,~,m_C,OM] = dynamics.set_parameters(parameters);

p = state(1:2);
v = state(3:4);
theta_c = state(5);
w_c = state(6);
theta_t = state(7);
w_t = state(8);

% manage control
if (nargin < 3)
    f = zeros(2,1);
    tau = 0;
elseif (isa(control, 'function_handle'))
    c = control(state);
    f = c(1:2);
    tau = c(3);
else
    f = control(1:2);
    tau = control(3);
end


w_lc = w_c - OM; %because orientation is w.r.t lvlh frame
w_lt = w_t - OM;

dx = [v; ...
    1/m_C*(2*OM*v(2) + 3*OM^2*p(1) + f(1)); ...
    1/m_C*(-2*OM*v(1) + f(2)); ...
    w_lc; ...
    1/J_C*tau; ...
    w_lt; ...
    0; ...
    ];

end
