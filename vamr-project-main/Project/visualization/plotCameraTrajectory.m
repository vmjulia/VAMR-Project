function plotCameraTrajectory(camTransforms, indices, plot_camera, plot_axis_whole_trajectory, ax)
%   Plots the trajectory of cameras at specific indices
%   Arguments:
%       camTransforms  - cell array of [3x4] camera transformation matrices
%       ax - the axis (figure or subplot) to plot onto
%            When no axis is provided, this plots to figure 1
%       plot_camera - boolean

    if nargin >= 5
        axes(ax);
    else
        figure();
        ax = gca;
    end
    
    buffer = 50;

    centers = zeros(3, length(indices));
    plot_idx = 1;
    hold(ax, 'on')
    for state_idx = indices
        centers(:, plot_idx) = getCameraCenter(camTransforms{state_idx});
        plot_idx = plot_idx + 1;
    end

    if plot_camera
        % plot the last camera
        step_length = mean(sqrt(sum((centers(:, 2:end) - centers (:, 1:end-1)) .^ 2, 1)));
        plotCameraPose2d(camTransforms{indices(end)}, step_length, ax);
    end

    plot(centers(1, :), centers(3, :), '-xb');
    if plot_axis_whole_trajectory
        xlim([centers(1, 1) centers(1, end)])
        ylim([centers(3, 1) centers(3, end)])
    end
    %axis([min(centers(1, :)), max(centers(1, :)) + buffer, min(centers(3, :)), max(centers(3, :)) + buffer])
    hold(ax, 'off');
end

