%% main program
impath = "Exercises/ex01/data/images/img_0015.jpg";
image = readimage(impath);
poses = get_poses();

pose_matrix = pose_matrix_i(poses, 15);
[points, n] = cube_vertices();
connections = cube_connections();

K = get_intrinsics();
D = get_distortion();
image_locations = arrayfun(@(i) project_point(points(:, i), pose_matrix, K), 1:size(points, 2), 'uni', 0);
image_locations = cell2mat(image_locations);
distorted = arrayfun(@(i) apply_distortion(image_locations(:, i), D, K), 1:size(image_locations, 2), 'uni', 0);
image_locations = cell2mat(distorted);

draw_cube(image, image_locations, connections);

%% draw functions

function draw_cube(image, vertices, edges)
% draws a cube onto the image
% vertices... 2xN matrix of 2D image coordinates
% edges...    2xM matrix of connected indices in *vertices*
    imshow(image)
    hold on
    scatter(vertices(1, :), vertices(2, :), 'red', 'filled')
    draw_connected(vertices, edges)
    hold off
end

function draw_connected(vertices, edges)
    for e = edges
        points = [vertices(:, e(1)), vertices(:, e(2))];
        line(points(1, :), points(2, :), 'Color', 'Red', 'LineWidth', 2);
    end
end

function connections = cube_connections()
    connections = [1, 1, 1, 8, 8, 8, 2, 2, 7, 7, 3, 5;
                   3, 5, 2, 4, 6, 7, 4, 6, 3, 5, 4, 6];
end

function [points, n_points] = cube_vertices()
    n_points = 8;
    d = 0.08;
    %         1   2   3   4   5   6   7   8
    points = [0,  0,  0,  0,  d,  d,  d,  d;
              0,  0,  d,  d,  0,  0,  d,  d;
              0, -d,  0, -d,  0, -d,  0, -d;
              1,  1,  1,  1,  1,  1,  1,  1];
end

function [points, n_points] = checkerboard_positions(n, m)
    distance = 0.04;
    n_points = n * m;
    points = zeros(4, n_points);
    for j = 0:(m-1)
        for i = 0:(n-1)
            points(:, 1 + i * m + j) = [distance * j; distance * i; 0; 1];
        end
    end
end

%% data retrieval and conversion

function pose_mat = pose_matrix_i(all_poses, i)
	current = current_pose(all_poses, i);
	pose_mat = pose2mat(current);
end

function pose_mat = pose2mat(pose_vector)
	rotational_part = pose_vector(1:3);
	translational_part = pose_vector(4:6);

    R = rodrigues(rotational_part);
    t = translational_part;

	pose_mat = [R, t';
                0, 0, 0, 1];
end

function pose = current_pose(all_poses, i)
    pose = table2array(all_poses(i, :));
end

function image = readimage(path)
	image = imread(path);
	image = rgb2gray(image);
end


function poses = get_poses()
	poses = readtable("Exercises/ex01/data/poses.txt");
end

function K = get_intrinsics()
    K = readmatrix("Exercises/ex01/data/K.txt");
end

function D = get_distortion()
    fileID = fopen("ex1/data/D.txt",'r');
    formatSpec = '%f%f%f[^\n\r]';
    data = textscan(fileID, formatSpec,'Delimiter', {' '});
    D = [data{1:end-1}];
end


