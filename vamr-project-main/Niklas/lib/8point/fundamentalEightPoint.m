function F = fundamentalEightPoint(p1,p2)
% fundamentalEightPoint  The 8-point algorithm for the estimation of the fundamental matrix F
%
% The eight-point algorithm for the fundamental matrix with a posteriori
% enforcement of the singularity constraint (det(F)=0).
% Does not include data normalization.
%
% Reference: "Multiple View Geometry" (Hartley & Zisserman 2000), Sect. 10.1 page 262.
%
% Input: point correspondences
%  - p1(3,N): homogeneous coordinates of 2-D points in image 1
%  - p2(3,N): homogeneous coordinates of 2-D points in image 2
%
% Output:
%  - F(3,3) : fundamental matrix

[dim, N] = size(p1);
[dim2, N2] = size(p2);

% Sanity checks
assert(dim==dim2 && N==N2,'Size mismatch of input points');
assert(dim==3,'Input arguments are not 2D points');
assert(N>=8,'Insufficient number of points to compute fundamental matrix (need >=8)');

C = zeros(N, 9);
for i = 1:N
    point1 = p1(:, i);
    point2 = p2(:, i);

    C(i, :) = kron(point1, point2)';
end

[~, ~, V] = svd(C,0);
F_vec = V(:, 9);
F = reshape(F_vec, [3,3]);

%% ensure F has rank 2

[u, s, v] = svd(F);

s(3, 3) = 0;
F = u * s * v';

end
