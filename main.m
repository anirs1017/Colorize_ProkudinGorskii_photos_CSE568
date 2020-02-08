%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% CSE 568: Robotics Algorithms Fall 18 %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% LAB 3: Prokudin Gorskii Colorizing main %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% ANIRUDDHA SINHA, asinha6 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%% Clear workspace %%%%%%%%%%%
close all;
clear all;
clc;

%Declare the input path to read the images
% srcPath = dir('F:\M.S\1st sem\Robotics\Assignment 3\*.jpg');

%%% Iterating the code over the six image files %%%%%%%%%%%
for index = 1:6
    inputimg = imread(strcat('images/image',int2str(index),'.jpg'));   % Reading each image file
    inputimg = im2double(inputimg);         % Converting the image to double for faster and accurate calculations
    [height, width] = size(inputimg);       % Reading the dimensions of the image

    h = height/3;                           % Dividing the stacked channels into different channels
    blue = inputimg(1:h, 1:width);          % Initializing the blue channel
    green = inputimg(h+1:2*h, 1:width);     % Initializing the green channel
    red = inputimg(2*h+1:3*h, 1:width);     % Initializing the red channel

    unaligned  = cat(3, red, green, blue);      % Creating the unaligned colour image
    figure
    imshow(unaligned);
    title(strcat('Unaligned image',int2str(index)));        %Display and write the image
    imwrite(unaligned, strcat('image',int2str(index),'-color.jpg'))
    
    % Calculating the shifts of green and red channels w.r.t the blue
    % channels using SSD and display them
    fprintf(strcat('SSD results for image', int2str(index)));
    [shift_g_ssd, shift_r_ssd] = imalign1(red, green, blue);            
    aligned_ssd = cat(3, shift_r_ssd, green, shift_g_ssd);
    
    % Display the SSD aligned image
    figure
    imshow(aligned_ssd);
    title('Aligned SSD image');
    imwrite(aligned_ssd, strcat('images/image', int2str(index),'-ssd.jpg'))
    
    
    %%%%%%%%%%% Calculating the shifts of green and red channels w.r.t the 
    %%%%%%%% blue channels using NCC and display them
    fprintf(strcat('NCC results for image', int2str(index)));
    [shift_g_ncc, shift_r_ncc] = imalign2(red, green, blue);
    aligned_ncc = cat(3, shift_r_ncc, shift_g_ncc, blue);
%     
    % Display the NCC aligned image
    figure,
    imshow(aligned_ncc);
    title('Aligned NCC image');
    imwrite(aligned_ncc, strcat('images/image', int2str(index),'-ncc.jpg'))
end
