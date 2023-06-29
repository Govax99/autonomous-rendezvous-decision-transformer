function txt = info_trajectory(trajectory,tsolve,label)
%PRINT_TRAJECTORY Summary of this function goes here
%   Detailed explanation goes here
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

