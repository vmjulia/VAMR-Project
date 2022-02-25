function disp_img = getDisparity(...
    left_img, right_img, patch_radius, min_disp, max_disp)
% left_img and right_img are both H x W and you should return a H x W
% matrix containing the disparity d for each pixel of left_img. Set
% disp_img to 0 for pixels where the SSD and/or d is not defined, and for d
% estimates rejected in Part 2. patch_radius specifies the SSD patch and
% each valid d should satisfy min_disp <= d <= max_disp.
[im_height, im_width] = size(left_img);
im_size = [im_height, im_width];
patch_size = (2 * patch_radius + 1)^2;

% size for extended images which can deal with edge cases
l_buf_size = max_disp + patch_radius;
r_buf_size = patch_radius;
top_buf_size = patch_radius;
bottom_buf_size = patch_radius;
disp_img = zeros(im_size);

disp_range = max_disp - min_disp + 1;

u_range = l_buf_size+1 : im_width - r_buf_size;
v_range = top_buf_size+1 : im_height - bottom_buf_size;

for v = v_range
    disp(v);
for u = u_range

    l_patch = left_img(v-patch_radius : v+patch_radius, u-patch_radius : u+patch_radius);
    l_patch_flat = reshape(l_patch, [1, patch_size]);

    ssds = zeros([1, disp_range]);
    r_patch_mat = zeros([disp_range, patch_size]);
    for d = min_disp:max_disp
        u_prime = u-d;
        v_prime = v;
        r_patch = right_img(v_prime-patch_radius : v_prime+patch_radius, u_prime-patch_radius : u_prime+patch_radius);
        r_patch_flat = reshape(r_patch, [1, patch_size]);
        r_patch_mat(d - min_disp + 1, :) = r_patch_flat;
    end
    ssds = pdist2(single(l_patch_flat), single(r_patch_mat));
    possible_matches = right_img(v-patch_radius : v+patch_radius, u-max_disp:u-min_disp);
    intermediate_plot([u, v], l_patch, possible_matches, ssds);

    [minval, argmin] = min(ssds);

    best_d = min_disp + argmin;

    % filter unclear
    threshold = 1.5 * minval;
    similar = ssds < threshold;
    num_similar = sum(similar,2);
    
    if best_d == min_disp || best_d == max_disp || num_similar > 2
        best_d = 0;
    end

    disp_img(v,u) = best_d;
end
end
end

function intermediate_plot(uv, l_patch, possible_matches, ssds)

    f = figure(3);
    f.Position = [500 500 900 300];
    h(1) = subplot(1,3,1);
    imagesc(l_patch);
    axis equal;
    axis off;
    xlabel(num2str(uv(1),'%i'));
    ylabel(num2str(uv(2),'%i'));
    
    h(2) = subplot(1,3,2);
    imagesc(possible_matches);
    pom_size = size(possible_matches);
    axis equal;
    axis off;
    h(3) = subplot(1,3,3);
    plot(-length(ssds):-1, fliplr(ssds));

    pause(.005);
end