function plotReprojectionError(points2d, reprojected)
% Plots 2 sets of 2d points with lines between correspondences
%   points2d - [2xN] original point set (drawn in blue)
%   points2d - [2xN] corresponding set of reprojected points (drawn in red)
    figure_number = 1000000 * rand(1);
    figure_number = round(figure_number);
    figure(figure_number);
    hold on
    title('reprojection error');
    scatter(points2d(1, :), points2d(2, :), 'blue', 'x');
    scatter(reprojected(1, :), reprojected(2, :), 'red', '+');
    plot([points2d(1, :); reprojected(1, :)], [points2d(2, :); reprojected(2, :)], 'k');
    hold off
end

