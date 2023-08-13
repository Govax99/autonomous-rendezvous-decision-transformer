function v = extract_from_trajectory(traj, i)
%DESCRIPTION add a trajectory to the database
%
% INPUT:
%    traj                structure containing relevant trajectory data from o.c.
%    i                   index of structure parameter
%
% OUTPUT:
%	 v                   extracted structure parameter
%
    if (i == 1)
        v = traj.observation;
    elseif (i == 2)
        v = traj.action;
    elseif (i == 3)
        v = traj.reward;
    elseif (i == 4)
        v = traj.done;
    end
end
