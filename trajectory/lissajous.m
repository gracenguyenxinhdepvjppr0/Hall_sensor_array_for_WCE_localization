
clear; clc; close all;

% Cấu hình Lissajous
NUM_POINTS = 150; %[cite: 3]
X_MIN = -45.0; X_MAX = 95.0; %[cite: 3]
Y_MIN = 635.0; Y_MAX = 775.0; %[cite: 3]
Z_MIN = -63.5; Z_MAX = -33.5; %[cite: 3]
XC = 35.0; YC = 695.0; ZC = -63.5; %[cite: 3]

Ax = (X_MAX - X_MIN) / 2; %[cite: 3]
Ay = (Y_MAX - Y_MIN) / 2; %[cite: 3]
Az = (Z_MAX - Z_MIN) / 2; %[cite: 3]

Cx = (X_MAX + X_MIN) / 2; %[cite: 3]
Cy = (Y_MAX + Y_MIN) / 2; %[cite: 3]
Cz = (Z_MAX + Z_MIN) / 2; %[cite: 3]

a = 3; b = 4; c = 2; %[cite: 3]
delta = pi / 2; %[cite: 3]

path = zeros(NUM_POINTS, 3);
for i = 0:(NUM_POINTS-1)
    t = 2 * pi * i / NUM_POINTS; %[cite: 3]
    x = Cx + Ax * sin(a * t + delta); %[cite: 3]
    y = Cy + Ay * sin(b * t); %[cite: 3]
    z = Cz + Az * sin(c * t); %[cite: 3]
    path(i+1, :) = [x, y, z];
end
path(1, :) = [XC, YC, ZC]; % Đặt điểm đầu tiên tại tâm theo cấu trúc code gốc[cite: 3]

% Vẽ đồ thị
figure('Name', 'Lissajous Trajectory');
plot3(path(:,1), path(:,2), path(:,3), '-go', 'MarkerFaceColor', 'g', 'MarkerSize', 4);
title('Lissajous Trajectory'); xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on; view(3);