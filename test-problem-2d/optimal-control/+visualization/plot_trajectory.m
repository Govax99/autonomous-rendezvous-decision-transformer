function plot_trajectory(traj)
%DESCRIPTION plot a rendezvous operation
%
% INPUT:
%    traj                structure containing relevant trajectory data from o.c.
%
x = traj.observation;
xC = x(1:3,:);
xT = zeros(size(xC));
wC = x(11:13,:);
wT = x(18:20,:);
qC = x(7:10,:);
qT = x(14:17,:);
for i = 1:size(qC,2)
    R_C(:,:,i) = quad2rotm(qC(:,i));
    R_T(:,:,i) = quad2rotm(qT(:,i));
end

satC = create_satellite(LC, xC, R_C);
satT = create_satellite(LT, xT, R_T);
vC = x(4:6,:);

figure;
title("Satellite Rendezvous - LVLH frame")
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
axis('equal')
lim = 15;
xlim([-lim, lim])
ylim([-lim, lim])
zlim([-lim, lim])
grid on;
hold on;
for k = 1:length(tspan) %since we have simulated in inverse
    plot_sat(satC, k)
    plot_sat(satT, k)

    quiver3(xC(1,k),xC(2,k),xC(3,k),vC(1,k),vC(2,k),vC(3,k),2,'LineWidth',2)
    wC_rot = satC.R(:,:,k)*wC(:,k) - OM_IL_L;
    quiver3(xC(1,k),xC(2,k),xC(3,k),wC_rot(1),wC_rot(2),wC_rot(3),5,'LineWidth',2)
    wT_rot = satT.R(:,:,k)*wT(:,k) - OM_IL_L;
    quiver3(xT(1,k),xT(2,k),xT(3,k),wT_rot(1),wT_rot(2),wT_rot(3),5,'LineWidth',2)
    %pause(t_F/N)
    pause(1e-6)
    if (k ~= length(tspan))
        clf
    end
    axis('equal')
    xlim([-lim, lim])
    ylim([-lim, lim])
    zlim([-lim, lim])
    title("Satellite Rendezvous - LVLH frame")
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    view(0, 90)
    grid on;
    hold on;
end
end

