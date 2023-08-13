function txt = info_trajectory(trajectory,tsolve,label)
%DESCRIPTION determine log message given trajectory computation results
%
% INPUT:
%    trajectory          structure containing relevant trajectory data from o.c.
%    tsolve              computational time spent in the solution (in seconds)
%    label               trajectory number in database generation process
%
% OUTPUT:
%	 txt                 text message describing trajectory generation
%
% SUCCESS: [1: trajectory created with success, -1: cannot create guess solution, -2: cannot solve optimal control]
if (trajectory.success == 1)
    txt = sprintf("%5d - %9.3f - %14.3f - %15.3f - %7d - ",label,trajectory.objective,tsolve,trajectory.time(end),trajectory.success);
    txt = txt + "SUCCESS: Obtained solution to optimal control";
elseif (trajectory.success == -1)
    txt = sprintf("%5d - %9.3f - %14.3f - %15.3f - %7d - ",label,NaN,tsolve,NaN,trajectory.success);
    txt = txt + "FAILURE: Initial guess is too far from reference point";
elseif (trajectory.success == -2)
    txt = sprintf("%5d - %9.3f - %14.3f - %15.3f - %7d - ",label,NaN,tsolve,NaN,trajectory.success);
    txt = txt + "FAILURE: Optimal control problem does not converge";
end

