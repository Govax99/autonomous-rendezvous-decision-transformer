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
    zpos_lim   = options.zpos_lim;
    wx_lim     = options.wx_lim;
    wy_lim     = options.wy_lim;
    wz_lim     = options.wz_lim;
    qaxis_lim  = options.qaxis_lim;
    qtheta_lim = options.qtheta_lim;
    rng shuffle;
    % ----- FINAL STATE DEFINITION ----- %
    qv = (qaxis_lim(:,2) - qaxis_lim(:,1)).*rand(3,1) + qaxis_lim(:,1);
    qtheta = (qtheta_lim(2) - qtheta_lim(1))*rand + qtheta_lim(1);
    q_LT_final = quat.euler2quat(qv, qtheta); % final orientation of tumbling object (variable)
    q_LC_final = q_LT_final; % chaser must have same orientation
    
    p_LC_C_final = pberth; % final position in front of target (local rendezvous configuration)
    p_LC_L_final = quat.rotate(p_LC_C_final, q_LT_final); % final position state (orbital frame)
    
    wx = (wx_lim(2) - wx_lim(1))*rand + wx_lim(1);
    wy = (wy_lim(2) - wy_lim(1))*rand + wy_lim(1);
    wz = (wz_lim(2) - wz_lim(1))*rand + wz_lim(1);
    w_IT_T_final = [wx wy wz]'; % final ang. velocity of tumbling object (variable)
    w_IC_C_final = w_IT_T_final; % chaser must have same ang. velocity
    
    OM_IL_L = [0 0 OM]'; % orbital frame ang. velocity
    R_LC_final = quat.quat2rotm(q_LC_final);
    w_LC_L_final = R_LC_final * w_IC_C_final - OM_IL_L; % ang. velocity of line of sight chaser-target
    v_LC_L_final = cross(w_LC_L_final, p_LC_L_final); % chaser must have this velocity to keep up with rotation
    x_final = [p_LC_L_final; v_LC_L_final; q_LC_final; w_IC_C_final]; %; q_LT_final; w_IT_T_final not needed anymore
    
    % ----- INITIAL STATE DEFINITION ----- %
    xpos = (xpos_lim(2) - xpos_lim(1))*rand + xpos_lim(1);
    ypos = (ypos_lim(2) - ypos_lim(1))*rand + ypos_lim(1);
    zpos = (zpos_lim(2) - zpos_lim(1))*rand + zpos_lim(1);
    p_LC_L_initial = [xpos ypos zpos];
    v_LC_L_initial = [0 0 0];
    q_LC_initial = [0 0 0 1];
    w_IC_C_initial = [0 0 0];
    x_init = [p_LC_L_initial, v_LC_L_initial, q_LC_initial, w_IC_C_initial]';
    
end

