close all;
clear all;

% 1. Load camera poses
% Each row i of matrix 'poses' contains the transformations that transforms
% points expressed in the world frame to points expressed in the camera frame.
pose_vectors = load('./data/poses.txt'); % each row represents one image

% 2. Define 3D Pw (world) corner positions
% [Nx3] matrix containing the corners of the checkerboard as 3D points
% (X,Y,Z), expressed in the world coordinate system
square_size = 0.04;
num_corners_x = 9; num_corners_y = 6;
num_corners = num_corners_x * num_corners_y;

% meshgrid returns 2-D grid coordinates based on vectors 0:8 (x), 0:5 (y)
% X is a matrix each row is a copy of x, Y is a matrix where each column is
% a copy of y. Grid represented by coordinates X and Y has length (y) rows
% and length (x) columns. Hence X and Y are both [6x9] matrices
[X, Y] = meshgrid(0:num_corners_x-1, 0:num_corners_y-1); 

% X(:) and Y(:) are both [54x1] vectors stacked together into a matrix
% [54x2]
p_W_corners = square_size * [X(:) Y(:)]; 

% add the Pz dimension (which is all zeros): [54x3]
% then transpose it [3x54]: (Px, Py, Pz)^T
p_W_corners = [p_W_corners zeros(num_corners,1)]';

% 3. Load camera intrinsics
K = load('./data/K.txt'); % calibration matrix      [3x3]
D = load('./data/D.txt'); % distortion coefficients [2x1]

% 4. Load one image with a given index
img_index = 1;

img_distorted = rgb2gray(imread(['./data/images/',sprintf('img_%04d.jpg',img_index)]));
img_undistorted = rgb2gray(imread('./data/images_undistorted/img_0001.jpg'));

% 5. Project the corners on the image
% 5.1 Compute the 4x4 homogeneous transformation matrix that maps points from the world
% to the camera coordinate frame [R|T]
T_C_W = poseVectorToTransformationMatrix(pose_vectors(img_index,:));

% 5.2 Transform 3d points from world to current camera pose (corners in the
% camera frame)
p_C_corners = T_C_W * [p_W_corners; ones(1,num_corners)]; % [4x4] * [[3x54] + [1x54]] -> [4x4] * [4x54] = [4x54]

% only take rows 1:3 because for the projection we need to multiply
% with K [3x3] camera matrix
p_C_corners = p_C_corners(1:3,:);

% 5.3 Project the points from the camera coordinate frame onto the image plane
% [3x54] is the desired projection onto the image plane
% where the third row is only 1s (for the Z coordinate)
projected_pts_undistorted = projectPoints(p_C_corners, K);
projected_pts_distorted = projectPoints(p_C_corners, K, D);

% 6. Plot the points on the undistorted/distorted image
% 6.1 Undistorted image (blue corner points)
figure()
imshow(img_undistorted); hold on;
% plot X and Y image coordinates
plot(projected_pts_undistorted(1,:), projected_pts_undistorted(2,:), 'b.')
hold off;

% 6.2 Distorted image (red corner points)
figure()
imshow(img_distorted); hold on;
% plot X and Y image coordinates
plot(projected_pts_distorted(1,:), projected_pts_distorted(2,:), 'r.');
hold off;

% Undistort image with bilinear interpolation
tic;
img_undistorted_bill = undistortImage(img_distorted,K,D,1);
disp(['Undistortion with bilinear interpolation completed in ' num2str(toc)]);

% Vectorized undistortion without bilinear interpolation
tic;
img_undistorted_vectorized = undistortImageVectorized(img_distorted,K,D);
disp(['Vectorized undistortion completed in ' num2str(toc)]);

figure();
subplot(1, 2, 1);
imshow(img_undistorted_bill);
title('With bilinear interpolation');
subplot(1, 2, 2);
imshow(img_undistorted_vectorized);
title('Without bilinear interpolation');

% Draw a cube on the undistorted image
offset_x = 0.04 * 3; offset_y = 0.04;
s = 2 * 0.04;
[X, Y, Z] = meshgrid(0:1, 0:1, -1:0);
p_W_cube = [offset_x + X(:)*s, offset_y + Y(:)*s, Z(:)*s]';

p_C_cube = T_C_W * [p_W_cube; ones(1,8)];
p_C_cube = p_C_cube(1:3,:);

cube_pts = projectPoints(p_C_cube, K, zeros(4,1));

figure();
imshow(img_undistorted_bill); hold on;

lw = 3;

% base layer of the cube
line([cube_pts(1,1), cube_pts(1,2)],[cube_pts(2,1), cube_pts(2,2)], 'color', 'red', 'linewidth', lw);
line([cube_pts(1,1), cube_pts(1,3)],[cube_pts(2,1), cube_pts(2,3)], 'color', 'red', 'linewidth', lw);
line([cube_pts(1,2), cube_pts(1,4)],[cube_pts(2,2), cube_pts(2,4)], 'color', 'red', 'linewidth', lw);
line([cube_pts(1,3), cube_pts(1,4)],[cube_pts(2,3), cube_pts(2,4)], 'color', 'red', 'linewidth', lw);

% top layer
line([cube_pts(1,1+4), cube_pts(1,2+4)],[cube_pts(2,1+4), cube_pts(2,2+4)], 'color', 'red', 'linewidth', lw);
line([cube_pts(1,1+4), cube_pts(1,3+4)],[cube_pts(2,1+4), cube_pts(2,3+4)], 'color', 'red', 'linewidth', lw);
line([cube_pts(1,2+4), cube_pts(1,4+4)],[cube_pts(2,2+4), cube_pts(2,4+4)], 'color', 'red', 'linewidth', lw);
line([cube_pts(1,3+4), cube_pts(1,4+4)],[cube_pts(2,3+4), cube_pts(2,4+4)], 'color', 'red', 'linewidth', lw);

% vertical lines
line([cube_pts(1,1), cube_pts(1,1+4)],[cube_pts(2,1), cube_pts(2,1+4)], 'color', 'red', 'linewidth', lw);
line([cube_pts(1,2), cube_pts(1,2+4)],[cube_pts(2,2), cube_pts(2,2+4)], 'color', 'red', 'linewidth', lw);
line([cube_pts(1,3), cube_pts(1,3+4)],[cube_pts(2,3), cube_pts(2,3+4)], 'color', 'red', 'linewidth', lw);
line([cube_pts(1,4), cube_pts(1,4+4)],[cube_pts(2,4), cube_pts(2,4+4)], 'color', 'red', 'linewidth', lw);

hold off;
set(gca,'position',[0 0 1 1],'units','normalized')

