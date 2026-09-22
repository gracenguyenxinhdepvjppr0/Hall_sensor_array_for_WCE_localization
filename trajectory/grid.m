% grid_trajectory.m
% Sinh quỹ đạo dạng lưới 3D dựa trên file robot_ABB_grid.py
clear; clc; close all;

% Cấu hình Grid
XC = -45.0; YC = 635.0; ZC = -63.5; %[cite: 1]
GRID_X = 8; GRID_Y = 8; GRID_Z = 6; %[cite: 1]
STEP_XY = 20.0; STEP_Z = 5.0;       %[cite: 1]
GRID_ROW_TARGET = 8; GRID_COL_TARGET = 8; %[cite: 1]

path = [];
for iz = 0:(GRID_Z-1)
    z = ZC + iz * STEP_Z; %[cite: 1]
    for ix = 0:(GRID_X-1)
        x = XC + ix * STEP_XY; %[cite: 1]
        for iy = 0:(GRID_Y-1)
            y = YC + iy * STEP_XY; %[cite: 1]
            
            % Chỉ giữ các điểm nằm trên hàng 8 và cột 8
            if (ix == GRID_COL_TARGET - 1) || (iy == GRID_ROW_TARGET - 1) %[cite: 1]
                path = [path; x, y, z];
            end
        end
    end
end

% Vẽ đồ thị
figure('Name', 'Grid Trajectory');
plot3(path(:,1), path(:,2), path(:,3), '-bo', 'MarkerFaceColor', 'b', 'MarkerSize', 4);
title('Grid Trajectory'); xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
grid on; view(3);