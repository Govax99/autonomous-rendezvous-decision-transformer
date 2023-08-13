function sat = create_satellite(L, x, R, p_dock)
%DESCRIPTION generate a satellite object containing data for visualization
%
% INPUT:
%    L                   length of the satellite main body (cube)
%    x                   list of satellite position vectors [3 x N]
%    R                   list of satellite orientation matrixes [3 x 3 x N]
%    p_dock              docking point
%
% OUTPUT:
%	 sat                 satellite visualization structure
%
    if nargin < 4
        p_dock = [L/2 0 0];
    end
    % points [8 x 3]
    sat.L = L;
    sat.points = L * [0.5 -0.5 0.5; ...
                  0.5 0.5 0.5; ...
                  0.5 0.5 -0.5; ...
                  0.5 -0.5 -0.5; ...
                  -0.5 -0.5 0.5; ...
                  -0.5 0.5 0.5; ...
                  -0.5 0.5 -0.5; ...
                  -0.5 -0.5 -0.5];
    sat.x = x; % [3 x N]
    sat.R = R; % [3 x 3 x N]
    sat.p_dock = p_dock;
end

