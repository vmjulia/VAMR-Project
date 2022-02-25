function image = loadImage(ds, i)
kitti_path = 'datasets/kitti';
malaga_path = 'datasets/malaga-urban-dataset-extract-07';
parking_path = 'datasets/parking';
drone_path_1 = 'datasets/drone_1/flight/video_2';
drone_path_2 = 'datasets/drone_2/flight/video_3';
    
if ds == 0
        image = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i)]);
elseif ds == 1
        images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
        left_images = images(3:2:end);
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
elseif ds == 3
        images = dir([drone_path_1 ...
        '/images']);
        images = images(10:5:end);
        image = im2uint8(rgb2gray(imread([drone_path_1 ...
        '/images/', images(i).name])));
        %image = im2uint8(rgb2gray(imread([drone_path_1 ...
        %sprintf('/images/%03d.png',i)])));
elseif ds == 4
        images = dir([drone_path_2 ...
        '/images']);
        images = images(11:3:end);
        image = im2uint8(rgb2gray(imread([drone_path_2 ...
        '/images/', images(i).name])));

else
        assert(false);
end