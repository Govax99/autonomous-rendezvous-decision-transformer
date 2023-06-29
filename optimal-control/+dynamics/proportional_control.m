function u = proportional_control(state, xref, parameters)
    [~,~,~,~,~,kP_tr,kD_tr,kP_rot,u_lim] = dynamics.set_parameters(parameters);
    p_LC_L = state(1:3);
    v_LC_L = state(4:6);
    %q_LC = state(7:10);
    w_IC_C = state(11:13);

    u = zeros(6,1);
    u(1:3) = kP_tr*(xref(1:3) - p_LC_L) + kD_tr*v_LC_L;
    %q_err = q_LC - xref(7:10);
    %q_err = 2 * atan2(norm(q_err(1:3)),q_err(4)) * q_err(1:3)/norm(q_err(1:3));
    %%u(4:6) = 0.1*eye(3)*q_err + 2*eye(3)*w_IC_C;

    %OM = 0.005;
    %OM_IL_L = [0; 0; OM];
    %R_LC = quad2rotm(q_LC);
    %w_LC_L = R_LC * w_IC_C - OM_IL_L;
    %u(4:6) = 1*eye(3)*w_LC_L; -> maybe need to be rotated again?
    u(4:6) = kP_rot*w_IC_C;
    for i = 1:6
        if (abs(u(i)) > u_lim(i))
            u(i) = sign(u(i));
        end
    end
end

