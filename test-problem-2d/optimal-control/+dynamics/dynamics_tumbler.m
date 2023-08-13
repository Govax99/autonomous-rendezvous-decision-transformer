function dx = dynamics_tumbler(time, state, parameters)
%DESCRIPTION state-space form of tumbling-target dynamics for ode integration
% (linear dynamics in L-frame, angular dynamics in T-frame)
%
% INPUT:
%    time                current integration time
%    state               current chaser state [q_LT, w_IT_T]
%    parameters          structure containing parameters for the dynamics
%
% OUTPUT:
%	 dx  	             derivative of state
%
% LEGEND FRAMES:
%    L-frame             LVLH, local vertical-local horizontal frame, center on target cm
%    T-frame             relative frame, attached to target object, center on target cm
%
% MONOGRAM NOTATION:
%    p_XY_Z              p: symbol of physical quantity,
%                        X: "measured from",
%                        Y: target point/frame,
%                        Z: "expressed in" frame
[~,~,~,OM] = dynamics.set_parameters(parameters);

theta_t = state(1);
w_t = state(2);

w_lt = w_t - OM;

dx = [
    w_lt; ...
    0; ...
    ];
end

