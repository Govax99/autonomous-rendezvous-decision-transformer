clc;
close all;
clear;

import visualization.*
import database.*

%%
load results\database_batch_1.mat
c =  0;
figure;
for i = 1:length(db)
    traj = db(i);
    if traj.success == 1
        %plot_trajectory(traj)
        plot(traj.observation(1,:),traj.observation(2,:))
        hold on;
    end
%     pause(0);
%     close all;

end
axis equal

%%
% ----- PARAMETERS FOR DYNAMIC SIMULATION ----- %
LC = 1.5; % length side chaser box
LT = 3; % length side target box
parameters.J_C = 2/3;
parameters.J_T = 11/3;
parameters.m_C = 1;
parameters.OM = 7.272e-5;
parameters.pberth = [5 0]';
parameters.kP_tr = 0.1*eye(2);
parameters.kD_tr = 1*eye(2);
parameters.kP_rot = 1;
parameters.u_lim = ones(1,3);
parameters.r2 = 4^2; % keep out zone

% ----- META-PARAMETERS: OPTIONS FOR SOLVERS AND REWARD DEFINITION ----- %
options.K_action = eye(3);
options.R_success = 25;
options.R_collision = -10;
options.R_timeout = -5;

options.xpos_lim = [-30 30];
options.ypos_lim = [-30 30];
options.w_lim = [-0.05 0.05];
options.th_lim = [0 2*pi];
options.evaluation_points = 101;


[x_init,x_final] = database.generate_boundaries(options,parameters);

tsolve = NaN;
t_F = 35;
max_tries = 3;
tol = 0.5;
flag = 0;
for i = 1:max_tries
    guess_traj = database.generate_guess_solution(x_init,x_final,parameters,t_F);
    if (norm(x_init([1:4, 6]) - guess_traj.x_init_oc([1:4, 6])) < tol) % 1:4,6 don't count orientation
        flag = 1;
        break;
    end
    t_F = t_F + 5;
end
t = guess_traj.t;
x = guess_traj.x;
% compute tumbling object trajectory
opts = odeset('RelTol',1e-6,'AbsTol',1e-6);
x_tumbler_final = x_final(5:6,1);
[~, x_tumbler] = ode45(@(t,s) dynamics.dynamics_tumbler(t,s,parameters), t(end:-1:1), x_tumbler_final, opts);
x_tumbler = x_tumbler';
x_tumbler = x_tumbler(:,end:-1:1);

%trajectory.x_init = [guess_traj.x_init_oc; x_tumbler(:,1)];
trajectory.time = t;

trajectory.observation = [x(:,1:end); x_tumbler(:,1:end)];

plot_trajectory(trajectory)