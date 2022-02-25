% Create images folder to store frames: change workingDir for every folder
workingDir = "..\video_3_cut_small";
mkdir(workingDir, "images")
video = 'video_3_cut_5fps.mp4';

% create a VideoReader to use for reading frames from the file
% filename)
droneVideo = VideoReader(video);

% create the Image Sequence
ii = 1;

while hasFrame(droneVideo)
   img = readFrame(droneVideo);
   filename = [sprintf('%03d',ii) '.png'];
   fullname = fullfile(workingDir,'images',filename);
   imwrite(img, fullname) 
   ii = ii+1;
end