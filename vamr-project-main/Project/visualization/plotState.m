function plotState(state, T, ax)
%PLOTSTATE Summary of this function goes here
%   Detailed explanation goes here
if nargin < 3
    ax = figure(1447);
else
    axes(ax);
end

points3d = state.X;
plot3(points3d(1,:), points3d(2,:), points3d(3,:), 'o');
hold on
plotCameraPose(T, ax);

axis equal
axis([-50, 50, -50, 50, -100, 100])
rotate3d on;
grid

