
clear; clc; close all;

% Cấu hình Helical
XC = 25.0; YC = 705.0; ZC = -63.5; %[cite: 2]
R_BAR_BASE = 70; R_TOP = 35; HELIX_HEIGHT = 30; %[cite: 2]
LOOPS = 5; N_POINTS = 150; %[cite: 2]

path = zeros(N_POINTS, 3);
for i = 0:(N_POINTS-1)
    t = i / (N_POINTS - 1); %[cite: 2]
    theta = 2 * pi * LOOPS * t; %[cite: 2]
    r = R_BAR_BASE + (R_TOP - R_BAR_BASE) * t; %[cite: 2]

    x = XC + r * cos(theta); %[cite: 2]
    y = YC + r * sin(theta); %[cite: 2]
    z = ZC + HELIX_HEIGHT * t; %[cite: 2]

    path(i+1, :) = [x, y, z];
end

% Vẽ đồ thị
figure('Name', 'Helical Trajectory');
plot3(path(:,1), path(:,2), path(:,3), '-ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4);
title('Helical Trajectory'); xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on; view(3);