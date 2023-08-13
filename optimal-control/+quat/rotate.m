function vp = rotate(v, q)
%DESCRIPTION rotate vector v with quaternion q (rotation about equivalent
%euler axis of euler angle magnitude)
%
% INPUT:
%    v                   vector
%    q                   quaternion q = [q_vec; q_scalar]
%
% OUTPUT:
%	 vp                  rotated vector
%
    t = 2*cross(q(1:3), v);
    vp = v + q(4)*t + cross(q(1:3), t);
end
