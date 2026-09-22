% ==============================================================================
% 2. HELICAL TRAJECTORY - KHÔNG DÙNG HÀM TỰ ĐỊNH NGHĨA
% Inputs LabVIEW: Rx_in, Ry_in, Rz_in
% Outputs LabVIEW: Path_Out, Quat_Out, Euler_Out
% ==============================================================================
Rx_in = 180.0; Ry_in = 0.0; Rz_in = 0.0; 
TOOL_OFFSET_X = -3.0; TOOL_OFFSET_Y = 0.0; TOOL_OFFSET_Z = 98.5;

% Cấu hình Helical
XC = 25.0; YC = 705.0; ZC = -63.5;
R_BAR_BASE = 70; R_TOP = 35; HELIX_HEIGHT = 30;
LOOPS = 5; N_POINTS = 150;

% 1. Tính Quaternion Base Inline
hz = Rz_in*pi/360; hy = Ry_in*pi/360; hx = Rx_in*pi/360;
cz = cos(hz); sz = sin(hz); cy = cos(hy); sy = sin(hy); cx = cos(hx); sx = sin(hx);
w_b = cx*cy*cz+sx*sy*sz; x_b = sx*cy*cz-cx*sy*sz; y_b = cx*sy*cz+sx*cy*sz; z_b = cx*cy*sz-sx*sy*cz;
n_b = sqrt(w_b^2 + x_b^2 + y_b^2 + z_b^2); if n_b==0; n_b=1; w_b=1; x_b=0; y_b=0; z_b=0; end
qw = w_b/n_b; qx = x_b/n_b; qy = y_b/n_b; qz = z_b/n_b;
Quat_List = [qw, qx, qy, qz];

% 2. Tạo quỹ đạo xoắn ốc
Path_Tip = zeros(N_POINTS, 3);
for i = 1:N_POINTS
    t = (i - 1) / (N_POINTS - 1);
    theta = 2 * pi * LOOPS * t;
    r = R_BAR_BASE + (R_TOP - R_BAR_BASE) * t;
    Path_Tip(i, :) = [XC + r * cos(theta), YC + r * sin(theta), ZC + HELIX_HEIGHT * t];
end

% 3. Bù trừ TCP Tool Inline
Path_Out = zeros(N_POINTS, 3); Quat_Out = zeros(N_POINTS, 4);
R11 = 1 - 2*(qy^2 + qz^2); R12 = 2*(qx*qy - qz*qw); R13 = 2*(qx*qz + qy*qw);
R21 = 2*(qx*qy + qz*qw);   R22 = 1 - 2*(qx^2 + qz^2); R23 = 2*(qy*qz - qx*qw);
R31 = 2*(qx*qz - qy*qw);   R32 = 2*(qy*qz + qx*qw);   R33 = 1 - 2*(qx^2 + qy^2);

dx = R11*TOOL_OFFSET_X + R12*TOOL_OFFSET_Y + R13*TOOL_OFFSET_Z;
dy = R21*TOOL_OFFSET_X + R22*TOOL_OFFSET_Y + R23*TOOL_OFFSET_Z;
dz = R31*TOOL_OFFSET_X + R32*TOOL_OFFSET_Y + R33*TOOL_OFFSET_Z;

for i = 1:N_POINTS
    Path_Out(i, :) = [Path_Tip(i,1) - dx, Path_Tip(i,2) - dy, Path_Tip(i,3) - dz] / 1000;
    Quat_Out(i, :) = Quat_List;
end

Euler_Out = repmat([Rx_in, Ry_in, Rz_in], N_POINTS, 1);