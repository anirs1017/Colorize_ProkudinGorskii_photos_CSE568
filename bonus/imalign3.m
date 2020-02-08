%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% CSE 568: Robotics Algorithms Fall 18 %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% LAB 3: Prokudin Gorskii Colorizing - Harris Corner, RANSAC %%%%%
%%%%%%%%%% ANIRUDDHA SINHA, asinha6 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [alignedCorners, offset_g, offset_r] = imalign3(red, green, blue)

% Calculate the Harris corners for all channels and take the locations of
% the corners for each channel
[b_corners, b_pos] = findHarrisCorners(blue, 1);
[g_corners, g_pos] = findHarrisCorners(green, 2);
[r_corners, r_pos] = findHarrisCorners(red, 3);


fprintf('\nHarris Corner detection '); %disp(length(b_pos)); %disp(length(b_ypos));
input = cat(3, red, green, blue);
[alignedCorners, disps] = alignCorners(input);
fprintf('\nGreen offset = '); %disp(length(g_pos)); %disp(length(g_ypos));
offset_g = disps(1,:); disp(offset_g);
fprintf('\nRed offset = '); %disp(length(r_pos)); %disp(length(r_ypos));
offset_r = disps(2,:); disp(offset_r);

%%%%%%%% Step 6: Use RANSAC Algorithm %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Applying the RANSAC Algorithm %%%%%%%%%%%%%%%%%%%%%
for k = 1:100               % Algorithm runs for 100 iterations
     
    % Step 7: Select at random, features from blue, green and red channels
     b = randsample(1:length(b_pos), 1);
     rand_b = b_pos(b,:);
     
     g = randsample(1:length(g_pos), 1);
     rand_g = g_pos(g,:);
     
     r = randsample(1:length(r_pos), 1);
     rand_r = r_pos(r,:);
     
     % Step 8: Find the offset or pixel shift for a random feature in blue v/s
     % green and blue v/s red, assuming they match
     offset_bg = rand_b - rand_g;
     offset_br = rand_b - rand_r;
     
     % Step 9 : Shift all other features in the image w.r.t the new shifts
     % calculated
     new_g = imtranslate(green, fliplr(offset_bg));
     new_r = imtranslate(red, fliplr(offset_br));
     
     inliers = 0;
     
     % Step 10 : Update the inlier count by searching for corresponding feature
     % matching in both channels within a window
     for i = 1:length(g_pos)
         if (new_g - blue == 0)
             inliers = inliers + 1;
         else
             continue;
         end
         
         % Step 11: Calculate the best alignment for the two channels 
         [aC_g, best_alignment_g] = alignCorners(new_g);
         [aC_r, best_alignment_r] = alignCornerns (new_r);
     end
end
end

%%%%%%% Harris Corner detector function %%%%%%%%%%%%%%%%%
%%%%%% Returns the corner matrix, and the positions of the corners
%%%%%% %%%%%%%%%%%
function [corners, pos] = findHarrisCorners(ch, no)

[h, w] = size(ch);  % Read the dimensions of the channel

k = 0.08;           % Sensitivity parameter, lies between 0.04 and 0.15
threshold = 2e+9;   % threshold for finding the best corners

sigma = 1;          % setting the S.D. for the Gaussian filter
[xmask, ymask] = meshgrid(-3:3, -3:3);  % deriving the masks for calculating gradient of the image in x and y direction

%%% Step 1: Make the Gaussian Filter of dim 7x7 %%%%%%%%%%%%%%%%%
Gxy = exp(-(xmask.^2 + ymask.^2)/(2*sigma^2));  % Define the xy-direction filter for the image
Gx = xmask.*exp(-(xmask.^2 + ymask.^2)/(2*sigma^2));    % Define the x-direction filter for the image
Gy = ymask.*exp(-(xmask.^2 + ymask.^2)/(2*sigma^2));    % Define the y-direction filter for the image

%%%%% Step 2: Filter the image in x and y directions and calculate
%%%%% gradients
Ix = conv2(Gx, ch);     % Calculate the image gradient in the x-direction
Iy = conv2(Gy, ch);     % Calculate the image gradient in the y-direction

%%%% Step 3: Calculate the squares of gradients and combined gradient in XY
%%%% direction
Ix2 = Ix.^2;            % Square the x-gradient values
Iy2 = Iy.^2;            % Square the y-gradient values
Ixy = Ix.*Iy;           % Calculate gradients in both directions

Ix2c = conv2(Gxy, Ix2);     % Filter the squared-x-gradients
Iy2c = conv2(Gxy, Iy2);     % Filter the squared-y-gradients
Ixyc = conv2(Gxy, Ixy);     % Filter the both direction gradient

corners = zeros(h, w);      

%%%%%%%%%%% Step 4: Find corners throughout the image
for i = 1:h
    for j = 1:w
        % Draw the auto-correlation matrix for every pixel
        M = [Ix2c(i,j), Ixyc(i,j); Ixyc(i,j) Iy2c(i,j)];
        
        % Calculate the cornerness value for each pixel
        C = det(M) - k*(trace(M)^2);
        
        % Check if the cornerness is more than the threshold. If true, it
        % is a corner
        if (C > threshold)
            corners(i,j) = C;
        end
    end
end

%%%%%% Step5: Using non-max suppression to detect every feature only once
positions = corners > imdilate(corners, [1 1 1; 1 0 1; 1 1 1]);
[xpos, ypos] = find(positions);     % Calculate the positions of the corners
pos = [xpos ypos];

%%%%%% Step 6: Display the corners on the image %%%%%%%%%%%%%%%%%%%%%%%
if (no == 1)
    channel = 'Blue';
elseif (no == 2)
    channel = 'Green';
else
    channel = 'Red';
end
    
figure; imshow(ch, []); hold on;
plot(ypos, xpos, 'g*'); title(strcat('Detected Corners - ','  ', channel));
% saveas(gcf, strcat(channel, '-corners.png'));
end

%%%%%%%%% Align Harris Corner function %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Returns the aligned corner image with the amount of alignment%%%%
function [new, move] = alignCorners(input)
% Find dimensions of the input to the function
rows = size(input, 1);
cols = size(input, 2);

% Compute the edges of all the channels before comparison and convert them
% to double for fast calculations
R = double(edge(input(1:rows, 1:cols, 1),'canny'));
G = double(edge(input(1:rows, 1:cols, 2), 'canny'));
B = double(edge(input(1+15:rows-15, 1+15:cols-15, 3), 'canny'));

% Concatenate the red and green templates to comparison with blue reference
% channels
all_Corners = cat(3,R, G);

scoreRed = 0;
scoreGreen = 0;

move = zeros(2,2);

% Iterate over the [-15,15] pixel window for calculating the movement of
% corners
for i = -15:1:15
    for j = -15:1:15
        
        % Derive the temporary templates for comparison with the reference
        % channel
        tempR = all_Corners(i+16:end-15+i, j+16:end-15+j, 1);
        tempG = all_Corners(i+16:end-15+i, j+16:end-15+j, 2);
        
        % Calculate the NCC values for Red and Green channels without
        % normxcorr2
        calc_R = sum(tempR(:).*B(:))/(norm(tempR(:))*norm(B(:)));
        calc_G = sum(tempG(:).*B(:))/(norm(tempG(:))*norm(B(:)));
        
        % Check if the new Red channel value is greater than the
        % previous. If true, save the new one with its location
        if (calc_R > scoreRed)
            move(2,:) = [i-1 j+1];
            scoreRed = calc_R;
        end
        
        % Check if the new Green channel value is greater than the
        % previous. If true, save the new one with its location
        if (calc_G > scoreGreen)
            move(1,:) = [i-1 j+1];
            scoreGreen = calc_G;
        end
    end
end

% Modify the red and green channels based on the corner offsets
input(16:end-move(2,1), 15+1:end-move(2,2), 1) = input(16+move(2,1):end, 16+move(2,2):end, 1);
input(16:end-move(1,1), 15+1:end-move(1,2), 2) = input(16+move(1,1):end, 16+move(1,2):end, 2);

% Return the aligned image
new = uint8(input);

end