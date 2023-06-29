function [] = create_database(varargin)
% before usage, install from these sources (and add to matlab path all needed):
% https://github.com/casadi/casadi/releases/download/3.6.3/casadi-3.6.3-linux64-matlab2018b.zip
% https://github.com/yoptimization/yop/archive/refs/tags/v1.0-rc3.zip
% usage example:
% !matlab -nodesktop -nosplash -r "create_database -ntot 2 -nsave 1 -printlevel 2"

% 1) deal with options
optionsNames = string(varargin(1:2:end));
optionsValues = string(varargin(2:2:end));
optionsValues = str2double(optionsValues);

defaultOptionsValues = set_options(optionsNames,optionsValues);

Ntot = defaultOptionsValues(1);
Nsave = defaultOptionsValues(2);
Nbatches = floor(Ntot/Nsave);
printLevel = defaultOptionsValues(3);

% 2) set options and parameters

% ---- GENERATOR AND DATABASE ----- %
rng(42);

% ----- PARAMETERS FOR DYNAMIC SIMULATION ----- %
LC = 1; % length side chaser box
LT = 3; % length side target box
parameters.J_C = eye(3);
parameters.J_T = eye(3);
parameters.m_C = 1;
parameters.OM = 0.005;
parameters.pberth = [5 0 0]';
parameters.kP_tr = 0.1*eye(3);
parameters.kD_tr = 1*eye(3);
parameters.kP_rot = 1*eye(3);
parameters.u_lim = ones(1,6);
parameters.r2 = LT^2; % keep out zone

% ----- META-PARAMETERS: OPTIONS FOR SOLVERS AND REWARD DEFINITION ----- %
options.K_action = eye(6);
options.R_success = 5;
options.R_collision = -10;
options.R_timeout = -5;

options.xpos_lim = [-15 15];
options.ypos_lim = [-15 15];
options.zpos_lim = [-15 15];
options.wx_lim = [-0.3 0.3];
options.wy_lim = [-0.3 0.3];
options.wz_lim = [-0.3 0.3];
options.qaxis_lim = [-1 1; ...
                     -1 1; ...
                     -1 1];
options.qtheta_lim = [0 90];
options.evaluation_points = 101;



tproc = zeros(1,Nsave);
results = zeros(1,Nsave);
infos = strings(1,Nsave);
db(1:Nsave) = struct("objective",[],"success",[],"time",[],"observation",[],"action",[],"done",[],"reward",[]);
mkdir('results')
for k = 1:Nbatches
    parfor (i = 1:Nsave,8)
        [tj, tsolve] = database.generate_trajectory(options, parameters);
        infos(i) = visualization.info_trajectory(tj,tsolve,i);
        results(i) = tj.success;
        db(i) = tj;
        tproc(i) = tsolve;
    end
    visualization.report_database_generation(k,infos,tproc,results,printLevel)

    save("results/database_batch_" + string(k) + ".mat","db")
    tproc = zeros(1,Nsave);
    results = zeros(1,Nsave);
    infos = strings(1,Nsave);
    db(1:Nsave) = struct("objective",[],"success",[],"time",[],"observation",[],"action",[],"done",[],"reward",[]);
end

exit
end


