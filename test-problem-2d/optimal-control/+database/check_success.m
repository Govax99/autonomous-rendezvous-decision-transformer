function s = check_success(x,parameters)
%DESCRIPTION given the berthing point (in parameters), check if the current
%state x is in berthing position
%
% INPUT:
%    state               current total state [p, v, theta_c, omega_c, theta_t, omega_t]
%    parameters          structure containing parameters for the dynamics
%
% OUTPUT:
%	 s                   boolean, true if in berthing position
%
[~,~,~,OM,pberth] = dynamics.set_parameters(parameters);

tol = 1e-6;
p_final = x(1:2);
v_final = x(3:4);
th_c_final = x(5); %w.r.t L
w_c_final = x(6); %expressed in I
th_t_final = x(7);
w_t_final = x(8);

p_check = support.rotm(th_t_final)*pberth;



w_lc_l_final = w_c_final - OM; % ang. velocity of line of sight chaser-target
v_check = cross([0 0 w_lc_l_final], [p_check; 0]); % chaser must have this velocity to keep up with rotation
v_check = v_check(1:2);
err = norm(p_final - p_check) + norm(v_final - v_check) + abs(th_c_final - th_t_final) + abs(w_c_final - w_t_final);
s = err < tol;
end

