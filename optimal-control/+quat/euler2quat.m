function q = euler2quat(e, theta)
    e = e/norm(e);
    q = [e*sind(theta/2); cosd(theta/2)];
end

