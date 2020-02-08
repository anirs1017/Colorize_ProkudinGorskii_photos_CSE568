%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% CSE 568: Robotics Algorithms Fall 18 %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% LAB 3: Prokudin Gorskii Colorizing Harris Corner %%%%%%%%%%%%%%%
%%%%%%%%%% ANIRUDDHA SINHA, asinha6 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%% Clear workspace %%%%%%%%%%%
close all;
clear all;
clc;

%%%%%%% Iterate over the set of input images
for index = 1:6
    imageName = strcat('image',int2str(index),'.jpg');
    img = imread(imageName);        % Read every image
% img = imread('image1.jpg');
    [height, width] = size(img);    % Read the dimensions of the image

    h = floor(height/3);    

    % Create the channel images
    blue = img(1:h,:);          % Blue Channel       
    green = img(h+1:2*h,:);     % Green Channel
    red = img(2*h+1:3*h,:);     % Red Channel

    disp(strcat('For image - ',int2str(index)))
    disp('For image - 6');
    
    %%%%%%%%%%%%%%%%% Perform image alignment using Harris Corner detection 
    %%%%%%%%%%%%%%%%% and RANSAC algorithm %%%%%%%%%%%%%%%%%%%%55
    [alignedCorners, offsetG, offsetR] = imalign3(red, green, blue);
    
    %%%%%%%%% Display the aligned image by corner detection %%%%%%%%%
    figure, imshow(alignedCorners); hold on;
    xlabel('Aligned Image by Harris Corner');
    saveas(gcf, strcat('image', int2str(index), '-corner_plot.jpg')); % save the image to file
    imwrite(alignedCorners, strcat('image', int2str(index),'-corner.jpg'));
% saveas(gcf, 'image6-corner.jpg');
end







