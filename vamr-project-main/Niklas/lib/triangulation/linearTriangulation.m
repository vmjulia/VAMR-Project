function P = linearTriangulation(p1,p2,M1,M2)
% LINEARTRIANGULATION  Linear Triangulation
%
% Input:
%  - p1(3,N): homogeneous coordinates of points in image 1
%  - p2(3,N): homogeneous coordinates of points in image 2
%  - M1(3,4): projection matrix corresponding to first image
%  - M2(3,4): projection matrix corresponding to second image
%
% Output:
%  - P(4,N): homogeneous coordinates of 3-D points

size_p = size(p1);
N = size_p(2);
P = zeros([4,N]);

for i = 1:N
    point_1 = p1(:, i);
    point_2 = p2(:, i);
    
    C = [cross_matrix_3(point_1) * M1;
         cross_matrix_3(point_2) * M2];
    [~, ~, v] = svd(C,0);

    P(:, i) = HNormalize(v(:, end));

end