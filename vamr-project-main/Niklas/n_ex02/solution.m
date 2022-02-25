num_images = 5;
imagedir = "data/images_undistorted";
K_path = "data/K.txt";
corner_path = "data/detected_corners.txt";
pW_path = "data/p_W_corners.txt";

K = load(K_path);
corners = load(corner_path);
pW = load(pW_path);
pW = [pW, ones(size(pW, 1), 1)];

P = pW;

for image_index = 1:num_images
    p = corners_to_mat(corners(image_index, :));

    M = estimatePoseDLT(p, P', K);
    M = [M; 0, 0, 0, 1];
    reprojected = reprojectPoints(P', M, K);

    image = imread(sprintf("%s/img_%04d.jpg", imagedir, image_index));

    plot_projection(image, p, reprojected);
end


function plot_projection(image, detected, reprojected)
    figure();
    imshow(image); hold on
    plot(detected(1, :), detected(2, :), 'bo');
    plot(reprojected(1, :), reprojected(2, :), 'r+');
    hold off
end

function p_mat = corners_to_mat(corners)
    n = length(corners) / 2;
    p_mat = reshape(corners, [2, n]);
    p_mat = [p_mat; ones(n, 1)'];
end

function p_reprojected = reprojectPoints(P, M, K)
    p_cell = arrayfun(@(i) project_point(P(:, i), M, K), 1:size(P, 2), 'uni', 0);
    p_reprojected = cell2mat(p_cell);
end


    


