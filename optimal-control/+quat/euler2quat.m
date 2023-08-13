function q = euler2quat(e, theta)
%DESCRIPTION compute quaternion from euler axis-angle
%
% INPUT:
%    e                   euler axis vector
%    theta               euler angle (in deg)
%
% OUTPUT:
%    q                   quaternion q = [q_vec; q_scalar]
%
    e = e/norm(e);
    q = [e*sind(theta/2); cosd(theta/2)];
end

