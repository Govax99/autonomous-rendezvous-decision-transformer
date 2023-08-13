function s = check_success(x,parameters)
%DESCRIPTION given the berthing point (in parameters), check if the current
%state x is in berthing position
%
% INPUT:
%    state               current total state [p_LC_L, v_LC_L, q_LC, w_IC_C, q_LT, w_IT_T]
%    parameters          structure containing parameters for the dynamics
%
% OUTPUT:
%	 s                   boolean, true if in berthing position
%
[~,~,~,OM,pberth] = dynamics.set_parameters(parameters);

tol = 1e-6;
p_LC_L_final = x(1:3);
v_LC_L_final = x(4:6);
q_LC_final = x(7:10);
w_IC_C_final = x(11:13);
q_LT_final = x(14:17);
w_IT_T_final = x(18:20);

p_LC_L_check = quat.rotate(pberth, q_LT_final);

OM_IL_L = [0 0 OM]'; % orbital frame ang. velocity
R_LC_final = quat.quat2rotm(q_LC_final);
w_LC_L_final = R_LC_final * w_IC_C_final - OM_IL_L; % ang. velocity of line of sight chaser-target
v_LC_L_check = cross(w_LC_L_final, p_LC_L_check); % chaser must have this velocity to keep up with rotation
err = norm(p_LC_L_final - p_LC_L_check) + norm(v_LC_L_final - v_LC_L_check) + norm(q_LC_final - q_LT_final) + norm(w_IC_C_final - w_IT_T_final);
s = err < tol;
end

