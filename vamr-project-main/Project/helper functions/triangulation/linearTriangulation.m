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
% Sanity checks
[dim,NumPoints] = size(p1);
[dim2,npoints2] = size(p2);

assert(dim==dim2,'Size mismatch of input points');
assert(NumPoints==npoints2,'Size mismatch of input points');
assert(dim==3,'Arguments x1, x2 should be 3xN matrices (homogeneous coords)');

[rows,cols] = size(M1);
assert(rows==3 && cols==4,'Projection matrices should be of size 3x4');
[rows,cols] = size(M2);
assert(rows==3 && cols==4,'Projection matrices should be of size 3x4');
%------------------------------------------------------------------------
num_points = size(p1, 2);
P = zeros(4, num_points);

for point = 1:num_points;
    % Built matrix of linear homogeneous system of equations
    A1 = cross2Matrix(p1(:,point))*M1; % cross2Matrix builds the skew symmetric matrix from p1: [3x3] * [3x4] = [3x4]
    A2 = cross2Matrix(p2(:,point))*M2;
    A = [A1; A2]; % stack on top of each other: [6x4]
    
    % Solve the linear homogeneous system of equations
    [~,~,v] = svd(A);
    P(:,point) = v(:,end);
end

P = P./(P(4,:));% Dehomogeneize (P is expressed in homogeneous coordinates)
% divide all entries by the last entry of P: P = P./repmat(P(4,:),4,1); 

return


