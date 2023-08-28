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
parameters.J_C = 2/3*1500;
parameters.J_T = 11/3*1500;
parameters.m_C = 1500;
parameters.OM = 7.272e-5;
parameters.pberth = [5 0]';
parameters.kP_tr = 0.1*1000*eye(2);
parameters.kD_tr = 1*1000*eye(2);
parameters.kP_rot = 1*1000;
parameters.u_lim = 22*ones(1,3);
parameters.r2 = 4^2; % keep out zone

% ----- META-PARAMETERS: OPTIONS FOR SOLVERS AND REWARD DEFINITION ----- %
options.K_action = eye(3);
options.R_success = 25;
options.R_collision = -10;
options.R_timeout = -5;
options.t_F = 100;

options.xpos_lim = [-30 30];
options.ypos_lim = [-30 30];
options.w_lim = [-0.05 0.05];
options.th_lim = [0 2*pi];
options.evaluation_points = 101;


[x_init,x_final] = database.generate_boundaries(options,parameters);

tsolve = NaN;
t_F = 200;
max_tries = 3;
tol = 0.5;
flag = 0;
for i = 1:max_tries
    guess_traj = database.generate_guess_solution(x_init,x_final,parameters,t_F);
    if (norm(x_init([1:4, 6]) - guess_traj.x_init_oc([1:4, 6])) < tol) % 1:4,6 don't count orientation
        flag = 1;
        break;
    end
    t_F = t_F + 100;
end
t = guess_traj.t;
x = guess_traj.x;
u = guess_traj.u;
% compute tumbling object trajectory
opts = odeset('RelTol',1e-6,'AbsTol',1e-6);
x_tumbler_final = x_final(5:6,1);
[~, x_tumbler] = ode45(@(t,s) dynamics.dynamics_tumbler(t,s,parameters), t(end:-1:1), x_tumbler_final, opts);
x_tumbler = x_tumbler';
x_tumbler = x_tumbler(:,end:-1:1);

%trajectory.x_init = [guess_traj.x_init_oc; x_tumbler(:,1)];
trajectory.time = t;

x = [x(:,1:end); x_tumbler(:,1:end)];
trajectory.observation = x;

%plot_trajectory(trajectory)
%% visualize
load results\database_batch_1.mat
i = 4;
t = db(i).time;
x = db(i).observation;
u = db(i).action;

figure; plot(t, x(1,:)); hold on;
plot(t, x(2,:))
legend(["position x","position y"])
title("Position of chaser c.m. - LVLH frame")
xlabel("t [s]")
ylabel("[m]")
grid on

figure; plot(t, x(3,:)); hold on;
plot(t, x(4,:))
legend(["velocity x","velocity y"])
title("Velocity of chaser c.m. - LVLH frame")
xlabel("t [s]")
ylabel("[m/s]")
grid on

figure; plot(t, x(6,:)); hold on;
legend("ang.velocity")
title("Angular velocity of chaser measured from inertial frame - Chaser local frame")
xlabel("t [s]")
ylabel("[rad/s]")
grid on


figure; plot(t, u(1,:)); hold on;
plot(t, u(2,:))
legend(["force x","force y"])
title("Forces on chaser - LVLH frame")
xlabel("t [s]")
ylabel("[N]")
grid on

figure; plot(t, u(3,:)); hold on;
legend("torque")
title("Torques on chaser - Chaser local frame")
xlabel("t [s]")
ylabel("[Nm]")
grid on
%%
load results\database_batch_1.mat
for i = 1:length(db)
    t = db(i).time;
    x = db(i).observation;
    
    %trajectory.x_init = [guess_traj.x_init_oc; x_tumbler(:,1)];
    trajectory.time = t;
    
    trajectory.observation = x;
    
    plot_trajectory(trajectory)
end
%%
load results\database_batch_1.mat
figure;
cmp = colormap("parula");
n = length(cmp);
for i = 1:length(db)
    traj = db(i);
    r = traj.reward;
    rtg = cumsum(r(end:-1:1));
    if traj.success == 1
        plot(traj.time, rtg(end:-1:1),'Color',cmp(mod(i,n),:))
    end
    hold on
end
xlabel("Time [s]")
ylabel("Reward [-]")
title("Reward of some of the successful episodes in the database")
grid on

%%

create_database("-ntot",2,"-nsave",1,"-savedir","results")