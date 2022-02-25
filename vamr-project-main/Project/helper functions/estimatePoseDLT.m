function M_tilde = estimatePoseDLT(p, P, K)
% Estimates the pose of a camera using a set of 2D-3D correspondences and a
% given camera matrix [R|t] = [3x4] 
%
% p: [nx2] vector containing the undistorted coordinates of the 2D points
% P: [nx3] vector containing the 3D point positions
% K: [3x3] camera matrix
%
% M_tilde: [3x4] projection matrix under the form M_tilde=[R_tilde|alpha*t] where R is a rotation
%    matrix. M_tilde encodes the transformation that maps points from the world
%    frame to the camera frame

% Convert 2D points to normalized coordinates
p_normalized = (K \ [p ones(length(p),1)]')'; % ([3x3] \ [3x12])' =  [12x3] matrix left division 
% p_normalized = K\pts_2d is the solution to the equation K*p_normalized = pts_2d. 
% Matrices K and pts_2d must have the same number of rows.
% is computationally faster than this: (inv(K)*[pts2d ones(length(pts2d),1)]')'
% p_normalized yields the calibrated coordinates ~pi: xi, yi, and 1

% Build the measurement matrix Q [24 x 12] 
num_corners = length(p_normalized); % 12
Q = zeros(2*num_corners, 12); 
% Q should have rank 11, and each 2D-3D point correspondence provides 2 independent equations
% thus at least 6 point correspondence are needed

% Q is of form: (can be built using kron function)
% [X1w Y1w Z1w 1 0    0   0  0 -x1X1w -x1Y1w -x1Z1w -x1]
% [0    0   0  0 X1w Y1w Z1w 1 -y1X1w -y1Y1w -y1Z1w -y1]
% ...
% [Xnw Ynw Znw 1 0    0   0  0 -xnXnw -xnYnw -xnZnw -xn] n = 12
% [0    0   0  0 Xnw Ynw Znw 1 -ynXnw -ynYnw -ynZnw -yn]

for i=1:num_corners
    u = p_normalized(i,1);
    v = p_normalized(i,2);
    
    Q(2*i-1,1:3) = P(i,:);
    Q(2*i-1,4) = 1;
    Q(2*i-1,9:12) = -u * [P(i,:) 1];
    
    Q(2*i,5:7) = P(i,:);
    Q(2*i,8) = 1;
    Q(2*i,9:12) = -v * [P(i,:) 1];
end

% Solve for Q.M_tilde = 0 subject to the constraint ||M_tilde||=1; a
% solution that minimizes ||Q * M_tilde||
[~,~,V] = svd(Q);
M_tilde = V(:,end); % last column of V: eigenvector corresponding to the smallest eigenvalue of Q^T * Q

M_tilde = reshape(M_tilde, 4, 3)'; % transform in format [3x4]

%% Extract [R|t] with the correct scale from M_tilde ~ [R_tilde|t_tilde] ~ [alpha * R | alpha * t] ~ [R|t]

% if m_tilde34 < 0, multiply with -1 to ensure that the rotation matrix R
% (extracted from M_tilde) is a proper rotation matrix with det(R) = 1
if det(M_tilde(:,1:3)) < 0
    M_tilde = -M_tilde;
end

R = M_tilde(:,1:3); % last column wourld be translation

% Find the closest orthogonal matrix to R to obtain R_tilde
% https://en.wikipedia.org/wiki/Orthogonal_Procrustes_problem
[U,~,V] = svd(R); % R = UΣV'
R_tilde = U*V'; % R_tilde = UIV' = UV'
% R_tilde is the nearest true rotation matrix R_tilde from given matrix R

% Normalization scheme using the Frobenius norm:
% recover the unknown scale using the fact that R_tilde is a true rotation
% matrix (orthogonal matrix)
% R_tilde = alpha * R -> alpha = norm(R_tilde) / norm(R)
alpha = norm(R_tilde, 'fro')/norm(R, 'fro'); 

% Build M_tilde with the corrected rotation and scale
% M_tilde = [R_tilde | alpha * t]
M_tilde = [R_tilde alpha * M_tilde(:,4)];


end

