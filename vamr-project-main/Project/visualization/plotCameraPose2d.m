function plotCameraPose2d(T, focal_length, ax)
%PLOTSTATE Plots coordinate axes representing the camera
%   Arguments:
%       T  - [3x4] the camera transformation matrix
%       ax - the axis (figure or subplot) to plot onto
%            When no axis is provided, this plots to figure 1
    if nargin >= 2
        axes(ax);
    else
        figure(1);
    end
    
    % R,T should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]
    R = T(1:3, 1:3)';
    cam_z_world = R * [0; 0; 1];
    center_cam = getCameraCenter(T);
    center_xz = [center_cam(1); center_cam(3)];
    lookdir_xz = [cam_z_world(1); cam_z_world(3)];
    %plot(center_xz, 'bx'); to plot the camera as x
    plotCamera(center_xz, lookdir_xz, focal_length);
end

function plotCamera(center, lookdir, focal_length)
    fov = 80;
    rot90 = [0, -1; 
             1,  0];
    sidelength = abs(tan(fov/2)) * focal_length;
    lookdir = lookdir / norm(lookdir) ;
    midpoint = center + lookdir * focal_length;

    xl = midpoint + rot90 * lookdir * sidelength;
    xr = midpoint - rot90 * lookdir * sidelength;
    triangle = [center, xl, xr, center, midpoint];

    plot(triangle(1, :), triangle(2, :));
end

