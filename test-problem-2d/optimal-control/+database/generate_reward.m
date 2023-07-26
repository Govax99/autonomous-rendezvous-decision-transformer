function reward = generate_reward(trajectory, parameters, options)
%DESCRIPTION given a certain trajectory determine reward at each instant of
%time, the reward sign is contained in the sign of the options parameters
%
% INPUT:
%    trajectory          structure containing relevant trajectory data from o.c.
%    parameters          structure containing parameters for the dynamics
%    options             contain coefficients for reward assignment
%
% OUTPUT:
%	 reward              reward at each time of the trajectory
%

% 1) choose what's inside reward (K_action (or K_state), R_collision,
% R_success)

t = trajectory.time;
dt = diff(t);
dt = [dt, dt(end)];
x = trajectory.observation;
%xn = trajectory.next_observation;
u = trajectory.action;
n = size(x,2);

reward = zeros(1,n);

K_action = options.K_action;
R_collision = options.R_collision; %needs trajectory.terminals
R_success = options.R_success; 
R_timeout = options.R_timeout; %needs trajectory.timeouts

flag_collision = false;
% Assign negative/positive reward for fuel consumption and collision (using
% done, so for last step we must make particular considerations)
for i = 1:n
    reward(i) = -norm(K_action*dt(i)*u(:,i));
    if (~flag_collision && trajectory.done(i) && i~=n)
        flag_collision = true;
        reward(i) = reward(i) + R_collision;
    end
end

xend = x(:,end);
% if last one is success assign reward for successful completion, otherwise
% check if impact at last timestep
if database.check_success(xend,parameters)
    reward(end) = reward(end) + R_success;
else
    if (trajectory.done(end))
        reward(end) = reward(end) + R_collision;
    end
end

end

