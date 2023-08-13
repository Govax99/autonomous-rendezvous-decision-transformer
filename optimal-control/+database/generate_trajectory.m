function [trajectory,tsolve] = generate_trajectory(options, parameters)
%DESCRIPTION given certain options and parameters generate a trajectory,
%using PD control for initial guess and Yop optimal control direct-solver
%
% INPUT:
%    options             contain coefficients for reward assignment
%    parameters          structure containing parameters for the dynamics
%
% OUTPUT:
%    trajectory          structure containing relevant trajectory data from o.c.
%    tsolve              computational time spent in the solution (in seconds)
%
[x_init,x_final] = database.generate_boundaries(options,parameters);

tsolve = NaN;
t_F = 35;
max_tries = 3;
tol = 0.5;
flag = 0;
for i = 1:max_tries
    guess_traj = database.generate_guess_solution(x_init,x_final,parameters,t_F);
    if (norm(x_init([1:6, 11:13]) - guess_traj.x_init_oc([1:6, 11:13])) < tol) % 1:6,11:13 don't count orientation
        flag = 1;
        break;
    end
    t_F = t_F + 5;
end

% discard initial guesses that do not connect arrive near enough generated
% initial and final points
if (flag == 0)
    trajectory.success = -1;
    t = guess_traj.t;
    x = guess_traj.x;
    u = guess_traj.u;
    trajectory.objective = NaN;
else
    sol = database.solve_optimal_control(guess_traj, parameters, options);
    tsolve = sol.Stats.t_proc_total;
    trajectory.objective = sol.NumericalResults.Objective;

    if (sol.Stats.success ~= 1)
        trajectory.success = -2;
    else
        trajectory.success = 1;
    end
    % simulate state of tumbler
    t = sol.NumericalResults.Independent;
    x = sol.NumericalResults.State;
    % after solution of optimal control is necessary a normalization of quaternions
    for i = 1:size(x,2)
        x(7:10,i) = x(7:10,i)/norm(x(7:10,i));
    end
    u = sol.NumericalResults.Control;
end
% compute tumbling object trajectory
opts = odeset('RelTol',1e-6,'AbsTol',1e-6);
x_tumbler_final = x_final(7:13,1);
[~, x_tumbler] = ode45(@(t,s) dynamics.dynamics_tumbler(t,s,parameters), t(end:-1:1), x_tumbler_final, opts);
x_tumbler = x_tumbler';
x_tumbler = x_tumbler(:,end:-1:1);
%trajectory.x_final = [x_final; x_tumbler_final];

%trajectory.x_init = [guess_traj.x_init_oc; x_tumbler(:,1)];
trajectory.time = t;

trajectory.observation = [x(:,1:end); x_tumbler(:,1:end)];
%trajectory.next_observation = [x(:,2:end); x_tumbler(:,2:end)];
% add backward derivative to this
% x_next = trajectory.observation(:,end) + (trajectory.observation(:,end) - trajectory.observation(:,end-1)); %assume that next step will have same time
% trajectory.next_observation = [trajectory.next_observation,  x_next];
trajectory.action = u;

% generate end of episode flags (impact)
done = database.generate_terminations(trajectory, parameters);
trajectory.done = done;

% generate rewards and return to go
reward = database.generate_reward(trajectory, parameters, options);
trajectory.reward = reward;



end

