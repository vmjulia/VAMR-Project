function DoG = compute_DoG(image, S, sigma_0)
    filtered_images = cell(1, S+2);

    DoG = ones([size(image), S+2]);
    filtered_images{1} = imgaussfilt(image, scaled_sigma(-1, S, sigma_0));
    for s = 1:(S+1)
        sigma_i = scaled_sigma(s-1, S, sigma_0);
        filtered_images{s+1} = imgaussfilt(image, sigma_i);

        DoG(:, :, s) = filtered_images{s} - filtered_images{s+1};
    end
end