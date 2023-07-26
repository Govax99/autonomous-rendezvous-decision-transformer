function [x_init,x_final] = generate_boundaries(options,parameters)
%DESCRIPTION generate initial and final states for the rendezvous maneuver
%
% INPUT:
%    options             contain range limitation in which the states get sampled
%    parameters          structure containing parameters for the dynamics
%
% OUTPUT:
%	 x_init              initial state of the maneuver (chaser+target)
%	 x_final             final state of the maneuver (chaser+target)
%
    [~,~,~,OM,pberth] = dynamics.set_parameters(parameters);
    
    xpos_lim   = options.xpos_lim;
    ypos_lim   = options.ypos_lim;
    w_lim     = options.w_lim;
    th_lim  = options.th_lim;
    rng shuffle;
    % ----- FINAL STATE DEFINITION ----- %
    th_t_final = (th_lim(2) - th_lim(1))*rand + th_lim(1);
    th_c_final = th_t_final; % chaser must have same orientation
    
    p_final = pberth; % final position in front of target (local rendezvous configuration)
    p_final = support.rotm(th_t_final)*p_final; % final position state (orbital frame)
    
    w_t_final = (w_lim(2) - w_lim(1))*rand + w_lim(1);
    w_c_final = w_t_final; % chaser must have same ang. velocity
    
    omega_lc_l_final = omega_c_final - OM; % ang. velocity of line of sight chaser-target
    v_final = cross([0 0 omega_lc_l_final], [p_check; 0]); % chaser must have this velocity to keep up with rotation
    v_final = v_final(1:2);
    x_final = [p_final; v_final; th_c_final; w_c_final]; %; q_LT_final; w_IT_T_final not needed anymore
    
    % ----- INITIAL STATE DEFINITION ----- %
    xpos = (xpos_lim(2) - xpos_lim(1))*rand + xpos_lim(1);
    ypos = (ypos_lim(2) - ypos_lim(1))*rand + ypos_lim(1);
    p_c_initial = [xpos ypos zpos];
    v_c_initial = [0 0 0];
    th_c_initial = 0;
    w_c_initial = 0;
    x_init = [p_c_initial, v_c_initial, th_c_initial, w_c_initial]';
    
end
