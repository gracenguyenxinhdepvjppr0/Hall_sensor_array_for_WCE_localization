% ==============================================================================
% 1. GRID TRAJECTORY - KHÔNG DÙNG HÀM TỰ ĐỊNH NGHĨA
% Inputs LabVIEW: Rx_in, Ry_in, Rz_in
% Outputs LabVIEW: Path_Out, Quat_Out, Euler_Out
% ==============================================================================
Rx_in = 180.0; Ry_in = 0.0; Rz_in = 0.0; % Tương đương REFERENCE_QUAT_ABB (0,0,1,0)
TOOL_OFFSET_X = -3.0; TOOL_OFFSET_Y = 0.0; TOOL_OFFSET_Z = 98.5;

% Cấu hình Grid
XC = -45.0; YC = 635.0; ZC = -63.5;
GRID_X = 8; GRID_Y = 8; GRID_Z = 6;
STEP_XY = 20.0; STEP_Z = 5.0;
GRID_ROW_TARGET = 8; GRID_COL_TARGET = 8;

% 1. Tính Quaternion Base Inline
hz = Rz_in*pi/360; hy = Ry_in*pi/360; hx = Rx_in*pi/360;
cz = cos(hz); sz = sin(hz); cy = cos(hy); sy = sin(hy); cx = cos(hx); sx = sin(hx);
w_b = cx*cy*cz+sx*sy*sz; x_b = sx*cy*cz-cx*sy*sz; y_b = cx*sy*cz+sx*cy*sz; z_b = cx*cy*sz-sx*sy*cz;
n_b = sqrt(w_b^2 + x_b^2 + y_b^2 + z_b^2); if n_b==0; n_b=1; w_b=1; x_b=0; y_b=0; z_b=0; end
qw = w_b/n_b; qx = x_b/n_b; qy = y_b/n_b; qz = z_b/n_b;
Quat_List = [qw, qx, qy, qz];

% 2. Tạo quỹ đạo lưới (Chỉ lấy điểm trên hàng 8 hoặc cột 8)
Path_Tip = zeros(GRID_X * GRID_Y * GRID_Z, 3);
pt_idx = 1;
for iz = 0:(GRID_Z-1)
    z_val = ZC + iz * STEP_Z;
    for ix = 0:(GRID_X-1)
        x_val = XC + ix * STEP_XY;
        for iy = 0:(GRID_Y-1)
            y_val = YC + iy * STEP_XY;
            if (ix == GRID_COL_TARGET - 1) || (iy == GRID_ROW_TARGET - 1)
                Path_Tip(pt_idx, :) = [x_val, y_val, z_val];
                pt_idx = pt_idx + 1;
            end
        end
    end
end
Path_Tip = Path_Tip(1:pt_idx-1, :);

% 3. Bù trừ TCP Tool Inline (áp dụng full offset X, Y, Z)
N = size(Path_Tip, 1); Path_Out = zeros(N, 3); Quat_Out = zeros(N, 4);
R11 = 1 - 2*(qy^2 + qz^2); R12 = 2*(qx*qy - qz*qw); R13 = 2*(qx*qz + qy*qw);
R21 = 2*(qx*qy + qz*qw);   R22 = 1 - 2*(qx^2 + qz^2); R23 = 2*(qy*qz - qx*qw);
R31 = 2*(qx*qz - qy*qw);   R32 = 2*(qy*qz + qx*qw);   R33 = 1 - 2*(qx^2 + qy^2);

dx = R11*TOOL_OFFSET_X + R12*TOOL_OFFSET_Y + R13*TOOL_OFFSET_Z;
dy = R21*TOOL_OFFSET_X + R22*TOOL_OFFSET_Y + R23*TOOL_OFFSET_Z;
dz = R31*TOOL_OFFSET_X + R32*TOOL_OFFSET_Y + R33*TOOL_OFFSET_Z;

for i = 1:N
    Path_Out(i, :) = [Path_Tip(i,1) - dx, Path_Tip(i,2) - dy, Path_Tip(i,3) - dz] / 1000;
    Quat_Out(i, :) = Quat_List;
end

Euler_Out = repmat([Rx_in, Ry_in, Rz_in], N, 1);