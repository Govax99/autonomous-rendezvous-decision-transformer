function done = generate_terminations(trajectory, parameters)
%DESCRIPTION given a certain trajectory determine termination boolean if
%enters the keep-out zone or terminates successfully
%
% INPUT:
%    trajectory          structure containing relevant trajectory data from o.c.
%    parameters          structure containing parameters for the dynamics
%
% OUTPUT:
%	 done                boolean, true if terminated else false
%
done = true(1, length(trajectory.time));
for i = 1:length(trajectory.time)
    x = trajectory.observation(1:2,i);
    if (norm(x) <= sqrt(parameters.r2))
        break;
    elseif database.check_success(trajectory.observation(:,i),parameters)
        break;
    else
        done(i) = false;
    end
end

end

