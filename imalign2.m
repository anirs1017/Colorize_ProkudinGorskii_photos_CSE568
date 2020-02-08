%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% CSE 568: Robotics Algorithms Fall 18 %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% LAB 3: Prokudin Gorskii Colorizing - NCC %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% ANIRUDDHA SINHA, asinha6 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [shift_g_ncc, shift_r_ncc] = imalign2(r, g, b)

shifts = zeros(2,2);

[row1, col1] = size(r);
[row2, col2] = size(g);
[row3, col3] = size(b);

% Perform edge detection on each channel
r_edge = edge(r(1:row3, 1:col3), 'canny');
g_edge = edge(g(1:row2, 1:col2), 'canny');
b_edge = edge(b(1:row1,1:col1), 'canny');

% Calculate the shifts of red and green channels w.r.t the blue channels
shifts(1,:) = ncc(g_edge, b_edge);
shifts(2,:) = ncc(r_edge, b_edge);

%%% Display the shifts through NCC for each channel to the command window
fprintf('\n Green channel NCC shift = ');
disp(shifts(1,:));
fprintf('\n Red channel NCC shift = ');
disp(shifts(2,:));

% Translate the red and green channels with the corresponding shifts
shift_g_ncc = imtranslate(g, [shifts(1,2), shifts(1,1)]);
shift_r_ncc = imtranslate(r, [shifts(2,2), shifts(2,1)]);

end

% Function to perform NCC over the template and the reference channels
function shift = ncc(ch2, ch1)
[row, col] = size(ch2);         % Read the dimensions of the template

%%%%%% Imp Note: Part of the following code has been copied and refered from the
%%%%%% official documentation of normxcorr2 on www.mathworks.com %%%%%%%%%%
cc = normxcorr2(ch2, ch1);      % Check NCC coefficients of template and reference
[max_cc, ind_cc] = max(abs(cc(:))); % Find the max NCC value 
[y_ext, x_ext] = ind2sub(size(cc), ind_cc(1)); % Find the location of the max NCC value
Best_match_row = y_ext - (row-1);   % Calculate the row location of the max NCC value
Best_match_col = x_ext - (col-1);   % Calculate the column location of the max NCC value
shift = [Best_match_row, Best_match_col];   % Shifts in both directions

end