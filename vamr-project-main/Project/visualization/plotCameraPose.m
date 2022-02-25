function plotCameraPose(T, ax)
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
R = T(1:3, 1:3);
center_cam = getCameraCenter(T);
plotCoordinateFrame(R',center_cam, 0.8);
text(center_cam(1)-0.1, center_cam(2)-0.1, center_cam(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');

