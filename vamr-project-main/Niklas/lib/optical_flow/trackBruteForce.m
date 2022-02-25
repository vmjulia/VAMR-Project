function [dx, ssds] = trackBruteForce(I_R, I, x_T, r_T, r_D)
% I_R: reference image, I: image to track point in, x_T: point to track,
% expressed as [x y]=[col row], r_T: radius of patch to track, r_D: radius
% of patch to search dx within; dx: translation that best explains where
% x_T is in image I, ssds: SSDs for all values of dx within the patch
% defined by center x_T and radius r_D.
disp(r_T);
[v_max, u_max] = size(I_R);
ssds = zeros([2*r_D + 1, 2*r_D + 1]);
min_ssd = inf;
dx = [0;0];
IR_patch = zeros([2*r_T + 1, 2*r_T + 1]);

v_start = max(x_T(2)-r_T, 1);
v_end = min(x_T(2)+r_T, v_max);
u_start = max(x_T(1)-r_T, 1);
u_end = min(x_T(1)+r_T, u_max);
IR_patch(v_start-x_T(2)+r_T+1:v_end-x_T(2)+r_T+1, u_start-x_T(1)+r_T+1:u_end-x_T(1)+r_T+1) = I_R(v_start:v_end, u_start:u_end);

IR_patch = IR_patch(:);


for du = -r_D:r_D
    for dv = -r_D:r_D  
        dx_candidate = [du;dv];
        W = [eye(2), dx_candidate];
        I_warped = getWarpedPatch(I, W, x_T, r_T);
        current_ssd = dot(I_warped(:), IR_patch); % ssd is a dot product between the patches
        ssds(dv+r_D+1, du+r_D+1) = current_ssd;
        if current_ssd < min_ssd
            min_ssd = current_ssd;
            dx = dx_candidate;
        end
    end
end

end