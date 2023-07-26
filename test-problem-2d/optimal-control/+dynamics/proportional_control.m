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
    p_LC_L = state(1:3);
    v_LC_L = state(4:6);
    %q_LC = state(7:10);
    w_IC_C = state(11:13);

    u = zeros(6,1);
    u(1:3) = kP_tr*(xref(1:3) - p_LC_L) + kD_tr*v_LC_L;
    
    u(4:6) = kP_rot*w_IC_C;
    
    for i = 1:6
        if (abs(u(i)) > u_lim(i))
            u(i) = sign(u(i));
        end
    end
end

