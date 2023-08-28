function [] = create_database(varargin)
% USAGE: command line tool for database generation, follows an example:
% !matlab -nodesktop -nosplash -r "create_database -ntot 2 -nsave 1 -printlevel 2 -savedir results -rngseed 42"
%
% INPUT:
%    -ntot               total trajectories to generate
%    -nsave              number of trajectories in one sub-batch (intermediate save size)
%    -printlevel         level of information shown while solving (0,1,2)
%    -savedir            directory in which to save the results
%    -rngseed            random number generator seed (for reproducibility)
%
% BEFORE USAGE: 
% clone the git repository: https://github.com/Govax99/autonomous-rendezvous-decision-transformer.git
% download and unzip from these sources (in optimal-control folder):
% https://github.com/casadi/casadi/releases/download/3.6.3/casadi-3.6.3-linux64-matlab2018b.zip
% https://github.com/yoptimization/yop/archive/refs/tags/v1.0-rc3.zip
% 
addpath(genpath('..\..\optimal-control\casadi'));
addpath(genpath('..\..\optimal-control\yop-1.0-rc3'));

% set default options if needed
optionsNames = string(varargin(1:2:end));
optionsValues = varargin(2:2:end);
for k = 1:length(optionsValues)
    tmp = str2double(optionsValues(k));
    if (~isnan(tmp))
        optionsValues{k} = tmp;
    end
end

defaultOptionsValues = set_options(optionsNames,optionsValues);

Ntot = defaultOptionsValues{1};
Nsave = defaultOptionsValues{2};
Nbatches = floor(Ntot/Nsave);
printLevel = defaultOptionsValues{3};
saveDir = defaultOptionsValues{4};
rngSeed = defaultOptionsValues{5};

% ---- GENERATOR AND DATABASE ----- %
fprintf("Seed: %d\n",rngSeed)
rng shuffle;

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
parameters.u_lim = 25*ones(1,3);
parameters.r2 = 4^2; % keep out zone

% ----- META-PARAMETERS: OPTIONS FOR SOLVERS AND REWARD DEFINITION ----- %
options.K_action = 0.02*eye(3);
options.R_success = 25;
options.R_collision = -25;
options.R_timeout = -25;
options.t_F = 200;

options.xpos_lim = [-30 30];
options.ypos_lim = [-30 30];
options.w_lim = [-0.05 0.05];
options.th_lim = [0 2*pi];
options.evaluation_points = 101;


% initialize database and result arrays
tproc = zeros(1,Nsave);
results = zeros(1,Nsave);
infos = strings(1,Nsave);
db(1:Nsave) = struct("objective",[],"success",[],"time",[],"observation",[],"action",[],"done",[],"reward",[]);
mkdir(saveDir)

% external for loop, generate batch and save results
for k = 1:Nbatches
    % generate trajectories in parallel using optimal control
    parfor (i = 1:Nsave,12)
        [tj, tsolve] = database.generate_trajectory(options, parameters);
        infos(i) = visualization.info_trajectory(tj,tsolve,i);
        results(i) = tj.success;
        db(i) = tj;
        tproc(i) = tsolve;
    end
    visualization.report_database_generation(k,infos,tproc,results,printLevel)

    save(saveDir + "/database_batch_" + string(k) + ".mat","db")
    tproc = zeros(1,Nsave);
    results = zeros(1,Nsave);
    infos = strings(1,Nsave);
    db(1:Nsave) = struct("objective",[],"success",[],"time",[],"observation",[],"action",[],"done",[],"reward",[]);
end

exit
end


