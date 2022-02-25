function P = camTransform2PMat(cam_transform)
% Computes the P matrix from a corresponding camera transform
%   Arguments:
%       cam_transform - [3x4] in the form of [R|t] to compute camera
%                       position and orientation
%   Outputs:
%       P - [3x4] to convert a point from world to camera frame
R = cam_transform(1:3, 1:3);
t = cam_transform(1:3, 4);

P = [R', -R'*t];
end

