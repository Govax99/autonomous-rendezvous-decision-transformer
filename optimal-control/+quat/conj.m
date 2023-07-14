function qc = conj(q)
%DESCRIPTION compute conjugate of quaternion
%
% INPUT:
%    q                   quaternion q = [q_vec; q_scalar]
%
% OUTPUT:
%	 qc                  conjugate quaternion
%
    qc = [-q(1:3); q(4)];
end
