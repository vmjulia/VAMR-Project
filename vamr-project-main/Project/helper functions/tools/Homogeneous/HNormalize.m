function pts = HNormalize(pts_hom, keep_homogeneous)
% Normalizes each point by dividing by its w component.
% If keep_homogeneous is False, the last column will be removed
% Argument:
%   pts_hom -  KxN array of (K-1)D homogeneous coordinates
%   keep_homogeneous - Bool (optional)
% Returns:
%   pts - normalized (non_)homogeneous points
if nargin < 2
    keep_homogeneous = true;
end

K = size(pts_hom, 1);

if keep_homogeneous
    return_dim = K;
else
    return_dim = K-1;
end

pts = pts_hom(1:return_dim, :) ./ repmat(pts_hom(K, :), return_dim, 1);
end

