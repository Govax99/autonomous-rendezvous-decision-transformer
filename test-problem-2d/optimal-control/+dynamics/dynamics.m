function [dx, y] = dynamics(time, state, parameters, control)
%DESCRIPTION state-space form of chaser dynamics for ode integration
% (linear dynamics in L-frame, angular dynamics in C-frame)
%
% INPUT:
%    time                current integration time
%    state               current chaser state [p_LC_L, v_LC_L, q_LC, w_IC_C]
%    parameters          structure containing parameters for the dynamics
%    control             control action [f, tau]
%
% OUTPUT:
%	 dx  	             derivative of state
%    y                   additional output info for optimal control
%
% LEGEND FRAMES:
%    L-frame             LVLH, local vertical-local horizontal frame, center on target cm
%    C-frame             relative frame, attached to chaser object, center on chaser cm
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


f = control(1:2);
tau = control(3);


w_lc = w_c - OM; %because orientation is w.r.t lvlh frame

dx = [v; ...
    1/m_C*(2*OM*v(2) + 3*OM^2*p(1) + f(1)); ...
    1/m_C*(-2*OM*v(1) + f(2)); ...
    w_lc; ...
    1/J_C*tau; ...
    ];



y.p = p;
y.v = v;
y.w_c = w_c;
y.w_lc = w_lc;
y.theta_c = theta_c;

y.u = control;
y.f = f;
y.tau = tau;

end

