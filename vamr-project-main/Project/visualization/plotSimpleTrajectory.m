function plotCameraTrajectory(centers, indices, ax)
%   Plots the trajectory of cameras at specific indices
%   Arguments:
%       camTransforms  - cell array of [3x4] camera transformation matrices
%       ax - the axis (figure or subplot) to plot onto
%            When no axis is provided, this plots to figure 1
%       plot_camera - boolean

    if nargin >= 3
        axes(ax);
    else
        figure();
        ax = gca;
    end
    
    to_plot = centers(:, indices);

    plot(to_plot(1, :), to_plot(3, :), '-xb');

    %axis([min(centers(1, :)), max(centers(1, :)) + buffer, min(centers(3, :)), max(centers(3, :)) + buffer])
    hold(ax, 'off');
end

