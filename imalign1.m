%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% CSE 568: Robotics Algorithms Fall 18 %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% LAB 3: Prokudin Gorskii Colorizing - SSD %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% ANIRUDDHA SINHA, asinha6 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [shift_g_ssd, shift_r_ssd] = imalign1(red, green, blue)

% Reading the dimensions of all the channels
[h, w] = size(red);

% Cropping the template and the reference channels by 20% on each edge
tocrop = floor([0.2*w, 0.2*h, 0.6*w, 0.6*h]);
b_crop = imcrop(blue, tocrop);
g_crop = imcrop(green,tocrop);
r_crop = imcrop(red,tocrop);

% Find the shifts of red and blue channels
shifts = zeros(2,2);
shifts(1,:) = ssd(b_crop, g_crop);
shifts(2,:) = ssd(r_crop, g_crop);

%%% Display the shifts through SSD for each channel to the command window
fprintf('\n Green channel SSD shift = ');
disp(shifts(1,:));
fprintf('\n Red channel SSD shift = ');
disp(shifts(2,:));

% Translate the red and blue channels 
shift_g_ssd = imtranslate(blue, fliplr(floor(shifts(1,:))));
shift_r_ssd = imtranslate(red, fliplr(floor(shifts(2,:))));

end


% Calculate the SSD of the two channels
function shift = ssd(ch1, ch2)

highest_ssd = inf;
dispWindow = 15;
% Iterate over a window of [-15,15] pixels
for y_disp = -dispWindow:dispWindow
    for x_disp = -dispWindow:dispWindow
        moved = circshift(ch1, [x_disp y_disp]);       %Circurly shift the template
        ssd_ch = sum(sum((moved - ch2).^2));        
        
        % Check if the SSD is minimum and the corresponding location taken
        % as the shift
        if ssd_ch < highest_ssd
            highest_ssd = ssd_ch;
            shift = [x_disp y_disp];
        end
    end
end

end