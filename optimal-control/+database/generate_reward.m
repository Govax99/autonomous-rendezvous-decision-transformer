function reward = generate_reward(trajectory, parameters, options)
%GENERATE_REWARD Summary of this function goes here
%   Detailed explanation goes here

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
for i = 1:n
    reward(i) = -norm(K_action*dt(i)*u(:,i));
    if (~flag_collision && trajectory.done(i))
        flag_collision = true;
        reward(i) = reward(i) + R_collision;
    end
end

xend = x(:,end);
if database.check_success(xend,parameters)
    reward(end) = reward(end) + R_success;
end

end

