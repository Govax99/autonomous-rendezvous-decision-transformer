function guess_traj = generate_guess_solution(x_init,x_final,parameters,t_F)
%GENERATE_GUESS_SOLUTION Summary of this function goes here
%   Detailed explanation goes here

    % ----- TIME ----- %
    N = 1000;
    tspan = linspace(t_F, 0, N);

    pcontr = @(state) dynamics.proportional_control(state, x_init, parameters);
    opts = odeset('RelTol',1e-6,'AbsTol',1e-6);
    [~, x] = ode45(@(t, x) dynamics.dynamics_guess(t, x, parameters, pcontr), tspan, x_final, opts);
    x = x'; % better if column are single time
    x = x(:,end:-1:1);
    tspan = tspan(end:-1:1);
    
    % second pass to obtain the control actions
    u = zeros(6, length(tspan));
    for i = 1:length(tspan)
        [~, f, tau] = dynamics.dynamics_guess(tspan(i), x(:, i), parameters, pcontr);
        u(:,i) = [f; tau];
    end

    x_init_oc = zeros(13,1);
    x_init_oc(1:3,1) = x(1:3,1);
    x_init_oc(4:6,1) = x(4:6,1);
    x_init_oc(7:10,1) = x(7:10,1);
    x_init_oc(11:13,1) = x(11:13,1);

    % BUILD GUESS TRAJECTORY
    guess_traj.t = tspan;
    guess_traj.x = x;
    guess_traj.u = u;
    guess_traj.x_init_oc = x_init_oc;
    %guess_traj.x_final_oc = x_final;
    guess_traj.t_F = t_F;
end

