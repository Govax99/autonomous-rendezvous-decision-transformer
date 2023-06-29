function v = extract_from_trajectory(traj, i)
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
