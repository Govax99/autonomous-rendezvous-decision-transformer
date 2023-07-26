function plot_sat(sat, k)
%DESCRIPTION plot a satellite (cube)
%
% INPUT:
%	 sat                 satellite visualization structure
%    k                   index indicating current time
%
    p = zeros(8,3);
    for i = 1:size(sat.points,1)
        p(i,:) = (sat.x(:,k) + sat.R(:,:,k) * sat.points(i,:).').';
    end
    
    l1 = [1 1 1 4 4 5 5 2 2 6 8 3]; % l1, l2 first and second point of a side
    l2 = [2 4 5 3 8 8 6 6 3 7 7 7];
    for i = 1:length(l1)
        plot3([p(l1(i),1), p(l2(i),1)], [p(l1(i),2), p(l2(i),2)], [p(l1(i),3), p(l2(i),3)], 'k')
        hold on;
    end
end

