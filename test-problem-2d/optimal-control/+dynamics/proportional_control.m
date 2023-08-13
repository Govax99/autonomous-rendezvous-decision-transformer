function u = proportional_control(state, xref, parameters)
%DESCRIPTION proportional-differential PD control for linear dynamics and
%proportional P for angular dynamics (+ saturation of forces and toruqes)
%
% INPUT:
%    state               current chaser state [p_LC_L, v_LC_L, q_LC, w_IC_C]
%    xref                set point that the controller must follow
%    parameters          structure containing parameters for the controller
%
% OUTPUT:
%    u                   control action [f, tau]
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
    [~,~,~,~,~,kP_tr,kD_tr,kP_rot,u_lim] = dynamics.set_parameters(parameters);
    p = state(1:2);
    v = state(3:4);
    w_c = state(6);

    u = zeros(3,1);
    u(1:2) = kP_tr*(xref(1:2) - p) + kD_tr*v;
    
    u(3) = kP_rot*w_c;
    
    for i = 1:3
        if (abs(u(i)) > u_lim(i))
            u(i) = sign(u(i));
        end
    end
end

