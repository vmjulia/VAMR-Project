
figure();
ax = gca;
%plotFull(ax);
plotSimple(ax);


function plotSimple(ax)
    centers = zeros(3, 1000);
    n_transforms = 1000;

    for i = 1:n_transforms
        transform = [1, 0, 0, sin(i/100);
                     0, 1, 0, 0;
                     0, 0, 1, i];
        centers(:, i) = transform(1:3, 4);
        hold off
        plotSimpleTrajectory(centers, 1:i, ax);
    end
end


function plotFull(ax)
    transforms = cell(1000);
    t_wc = cell(1000);
    n_transforms = 1000;

    for i = 1:n_transforms
        transforms{i} = [1, 0, 0, sin(i/100);
                         0, 1, 0, 0;
                         0, 0, 1, i];
        t_wc{i} = invertTransform(transforms{i});
        hold off
        plotCameraTrajectory(transforms, 1:i, false, false, ax);
    end
end


