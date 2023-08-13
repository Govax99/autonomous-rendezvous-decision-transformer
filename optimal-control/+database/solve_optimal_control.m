function sol = solve_optimal_control(guess_traj, parameters, options)
%DESCRIPTION given a guess trajectory (PD control) solve an optimal control
%problem to minimize propellant usage
%
% INPUT:
%	 guess_traj          contains time, states and control action of the initial guess trajectory
%    options             contain solver options parameters
%    parameters          structure containing parameters for the dynamics
%
% OUTPUT:
%    sol                 structure containing solution to the optimal control problem
%
[~,~,~,~,~,~,~,~,u_lim,r2] = dynamics.set_parameters(parameters);
if (~isfield(options,'optimal_control_tol'))
    options.optimal_control_tol = 1e-2;
end
if (~isfield(options,'evaluation_points'))
    options.evaluation_points = 100;
end
if (~isfield(options,'print_level'))
    options.print_level = 0;
end
% Definition System
ocSystem = YopSystem(...
    'states', 13, ...
    'controls', 6, ...
    'model', @(t,s,u) dynamics.dynamics(t,s,parameters,u) ...
    );

time = ocSystem.t;
system = ocSystem.y;

% Obtaining Initial Guess -> solve simpler optimal control

w0 = YopInitialGuess(...
    'signals', [ocSystem.t; ocSystem.x; ocSystem.u], ... % Time-varying variables
    'signalValues', [guess_traj.t; guess_traj.x; guess_traj.u] ... % Guess values
    );

% Optimal Control
p_LC_L_initial = guess_traj.x(1:3,1);
v_LC_L_initial = guess_traj.x(4:6,1);
q_LC_initial = guess_traj.x(7:10,1);
w_IC_C_initial = guess_traj.x(11:13,1);
p_LC_L_final = guess_traj.x(1:3,end);
v_LC_L_final = guess_traj.x(4:6,end);
q_LC_final = guess_traj.x(7:10,end);
w_IC_C_final = guess_traj.x(11:13,end);

ocp = YopOcp();
ocp.min({ timeIntegral( 1/2*(system.u.' * system.u) ) });
ocp.st(...
    'systems', ocSystem, ...
    { 0 '<=' t_f( time ) '<=' guess_traj.t_F  }, ...
    ... % Initial conditions
    {  p_LC_L_initial  '==' t_0( system.p_LC_L ) }, ...
    {  v_LC_L_initial  '==' t_0( system.v_LC_L ) }, ...
    {  q_LC_initial  '==' t_0( system.q_LC  ) }, ...
    {  w_IC_C_initial  '==' t_0( system.w_IC_C  ) }, ...
    ... % Terminal conditions
    {  p_LC_L_final  '==' t_f( system.p_LC_L  ) }, ...
    {  v_LC_L_final  '==' t_f( system.v_LC_L ) }, ...
    {  q_LC_final  '==' t_f( system.q_LC ) }, ...
    {  w_IC_C_final  '==' t_f( system.w_IC_C ) }, ...
    ... % Box conditions
    {  -u_lim  '<=' system.u '<=' u_lim }, ...
    ... % Keep out zone
    {  0 '<=' system.p_LC_L.'*system.p_LC_L - r2 } ...
    );

% Solving the OCP
sol = ocp.solve('initialGuess', w0,...
                'ipopt', struct('tol', options.optimal_control_tol, 'print_level', options.print_level, 'max_cpu_time', 500), ...
                'controlIntervals', options.evaluation_points - 1);
end

