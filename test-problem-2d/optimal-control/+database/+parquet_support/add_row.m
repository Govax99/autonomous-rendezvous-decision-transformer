function db = add_row(db, traj)
%DESCRIPTION add a trajectory to the database
%
% INPUT:
%    db                  database (vector of trajectories)
%    traj                structure containing relevant trajectory data from o.c.
%
% OUTPUT:
%	 db                  updated database 
%
    row = cell(1,size(db,2));
    for j = 1:2
        v = database.parquet_support.extract_from_trajectory(traj, j);
        timesteps = size(v,2);
        vtable = cell(timesteps,1);
        for k = 1:timesteps
            vtable{k} = v(:,k);
        end
        row{j} = vtable;
    end
    row{3} = traj.reward';
    row{4} = traj.done';
    db = [db; row];
end
