function done = generate_terminations(trajectory, parameters)
%GENERATE_TERMINATIONS Summary of this function goes here
%   Detailed explanation goes here
done = true(1, length(trajectory.time));
for i = 1:length(trajectory.time)
    x = trajectory.observation(1:3,i);
    if (norm(x) <= sqrt(parameters.r2))
        break;
    elseif database.check_success(trajectory.observation(:,i),parameters)
        break;
    else
        done(i) = false;
    end
end

end

