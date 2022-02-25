function center = getCameraCenter(cam_transform)
%GETCAMERACENTER Returns the position of the camera center
%   Arguments:
%       cam_transform - [3x4] the [R|t] matrix transforming the camera to
%                             its position and orientation in world space
%   Returns:
%       center - [3x1] the position of the camera center in world space

    R = cam_transform(1:3, 1:3);
    t = cam_transform(1:3, 4);
    
    center = -R'*t;

end

