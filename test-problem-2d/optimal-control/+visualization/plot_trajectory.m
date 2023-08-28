function plot_trajectory(traj)
%DESCRIPTION plot a rendezvous operation
%
% INPUT:
%    traj                structure containing relevant trajectory data from o.c.
%
LC = 1.5;
LT = 3;
OM = 7.272e-5;
tspan = traj.time;
x = traj.observation;
xC = x(1:2,:);
xT = zeros(size(xC));
wC = x(6,:);
wT = x(8,:);
theta_c = x(5,:);
theta_t = x(7,:);
for i = 1:size(theta_c,2)
    R_C(:,:,i) = support.rotm(theta_c(:,i));
    R_T(:,:,i) = support.rotm(theta_t(:,i));
end

satC = visualization.create_satellite(LC, xC, R_C);
satT = visualization.create_satellite(LT, xT, R_T);
vC = x(3:4,:);

figure;
title("Satellite Rendezvous - LVLH frame")
xlabel('x [m]')
ylabel('y [m]')
axis('equal')
lim = 30;
xlim([-lim, lim])
ylim([-lim, lim])
grid on;
hold on;
for k = 1:length(tspan) %since we have simulated in inverse
    visualization.plot_sat(satC, k)
    visualization.plot_sat(satT, k)
    pberth = satT.R(:,:,k)*[5; 0];
    plot([0 pberth(1)], [0, pberth(2)],'r--')

    % ATT: comment or import also OM to print also quiver arrows (vel, ang.
    % vel)
    quiver(xC(1,k),xC(2,k),vC(1,k),vC(2,k),2,'LineWidth',2)
%     wC_rot = wC(:,k) - OM;
%     quiver(xC(1,k),xC(2,k),wC_rot(1),wC_rot(2),5,'LineWidth',2)
%     wT_rot = wT(:,k) - OM;
%     quiver(xT(1,k),xT(2,k),wT_rot(1),wT_rot(2),5,'LineWidth',2)
    %pause(t_F/N)
    pause(1e-6)
    if (k ~= length(tspan))
        clf
    end
    axis('equal')
    xlim([-lim, lim])
    ylim([-lim, lim])
    title("Satellite Rendezvous - LVLH frame")
    xlabel('x [m]')
    ylabel('y [m]')
    grid on;
    hold on;
end
end

