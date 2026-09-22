
clear; clc; close all;

% Cấu hình Random
X_MIN = -45.0; X_MAX = 95.0; %[cite: 4]
Y_MIN = 635.0; Y_MAX = 775.0; %[cite: 4]
Z_MIN = -63.5; Z_MAX = -33.5; %[cite: 4]
TOTAL_RANDOM_POINTS = 150; %[cite: 4]

% Sinh tọa độ ngẫu nhiên phân phối đều (Uniform)
x = X_MIN + (X_MAX - X_MIN) * rand(TOTAL_RANDOM_POINTS, 1); %[cite: 4]
y = Y_MIN + (Y_MAX - Y_MIN) * rand(TOTAL_RANDOM_POINTS, 1); %[cite: 4]
z = Z_MIN + (Z_MAX - Z_MIN) * rand(TOTAL_RANDOM_POINTS, 1); %[cite: 4]

path = [x, y, z];

% Vẽ đồ thị
figure('Name', 'Random Trajectory');
plot3(path(:,1), path(:,2), path(:,3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 4);
title('Random Trajectory'); xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on; view(3);