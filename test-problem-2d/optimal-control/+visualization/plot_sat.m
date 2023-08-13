function plot_sat(sat, k)
%DESCRIPTION plot a satellite (cube)
%
% INPUT:
%	 sat                 satellite visualization structure
%    k                   index indicating current time
%
    p = zeros(4,2);
    for i = 1:size(sat.points,1)
        p(i,:) = (sat.x(:,k) + sat.R(:,:,k) * sat.points(i,:).').';
    end
    
    l1 = [1 2 3 4]; % l1, l2 first and second point of a side
    l2 = [2 3 4 1];
    for i = 1:length(l1)
        plot([p(l1(i),1), p(l2(i),1)], [p(l1(i),2), p(l2(i),2)], 'k')
        hold on;
    end
end

