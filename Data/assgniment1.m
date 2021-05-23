clc
clear all
%Sample script for Assignment 1
us_img = imread('dataset2_img_heart.png'); % Load the image
mask   = imread('dataset2_mask_heart.png'); % Load the mask of regions of interest
mask_bg   = imread('dataset2_bg_mask_heart.png'); % Load the background mask of regions of interest

% Convert Image to grayscale

us_img = rgb2gray(us_img);
mask   = rgb2gray(mask);
mask_bg   = im2gray(mask_bg);

% Display

figure;
subplot(1,3,1); imshow(us_img,[])
subplot(1,3,2); imshow(mask,[])
subplot(1,3,3); imshow(mask_bg,[])

% Edge detection
I = us_img;
BW1 = edge(I,'sobel');
figure;
imshow(BW1);

% Apply Enhancement
% Convert matrix to double format for calculation

[row,colum] = size(us_img);
img_cal = double(us_img);
mask_cal = double(mask);
mask_bg_cal = double(mask_bg);

% Sobel Operator
% Sobel x & Sobel y

G_x = zeros(row - 3, colum - 3);
G_y = zeros(row - 3, colum - 3);
mask_x = zeros(row - 3, colum - 3);
mask_y = zeros(row - 3, colum - 3);
mask_bg_x = zeros(row - 3, colum - 3);
mask_bg_y = zeros(row - 3, colum - 3);
sobel_x = [-1 0 1; -2 0 2; -1 0 1];
sobel_y = [-1 -2 -1; 0 0 0; 1 2 1];

% Sobel operation in the original image & Calculate indices

for x_j = 1:(colum - 3)
    for x_i = 1:(row - 3)
        I = img_cal(x_i:(x_i+2), x_j:(x_j+2));
        G_y(x_i,x_j) = sum(sum(sobel_y.* I));
    end
end
for x_i = 1:(row - 3)
    for x_j = 1:(colum - 3)
        I = img_cal(x_i:(x_i+2), x_j:(x_j+2));
        G_x(x_i,x_j) = sum(sum(sobel_x.* I));
    end
end
sobel_g_x = G_x;
sobel_g_y = G_y;
G = sqrt(sobel_g_x.^2 + sobel_g_y.^2);

% shows the image after apply sobel enhancement
figure;
imshow(G,[]);

% image after sobel x and sobel y
figure;
subplot(1,2,1); imshow(G_x,[]);
subplot(1,2,2); imshow(G_y,[]);

% SNR
% Multiply both foreground and background masks
fg = img_cal .* mask_cal;
bg = img_cal .* mask_bg_cal;
N = colum * row;
signal = 1/N * sum(sum(fg));  % Calculate signal with mean value of pixels
noise = std2(bg);   % Calculate noise with standard deviation
snr = signal / noise;

% SNR after Sober Operator
% Obtain new dimension of matrix.
gN = (row-3) * (colum-3);

% Multiply both foreground and background masks with image after Sobel
% operator
x = G.* mask_cal(1:row-3,1:colum-3);
x_bg = G.* mask_bg_cal(1:row-3,1:colum-3);
g_signal = 1/gN * sum(sum(x));  % Calculate signal with mean value of pixels
g_noise = std2(x_bg);   % Calculate noise with standard deviation
snr_g = g_signal / g_noise;

% SNR in dB
snr_db = 10*log10(1/N * ((sum(sum(fg-signal))).^2)/ (noise)^2);
snr_db_g = 10*log10(1/gN * ((sum(sum(x-g_signal))).^2)/ (g_noise)^2);

% RMS Contrast
% Calculate indicesa
% Original image
N = colum*row;
P = us_img;
u = 1/N * sum(sum(P));
P_final = (P-u).^2;
rms = sqrt((1/(N-1)) * sum(sum(P_final)));

% RMS after enhancement
u_rms = 1/gN * sum(sum(G));
G_final = (G - u_rms).^2;
rms_s = sqrt((1/(gN-1)) * sum(sum(G_final)));

