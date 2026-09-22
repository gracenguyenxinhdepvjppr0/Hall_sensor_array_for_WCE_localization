% ==============================================================================
% 3. LISSAJOUS TRAJECTORY - KHÔNG DÙNG HÀM TỰ ĐỊNH NGHĨA
% Inputs LabVIEW: Rx_in, Ry_in, Rz_in
% Outputs LabVIEW: Path_Out, Quat_Out, Euler_Out
% ==============================================================================
Rx_in = 180.0; Ry_in = 0.0; Rz_in = 0.0; 
TOOL_OFFSET_X = -3.0; TOOL_OFFSET_Y = 0.0; TOOL_OFFSET_Z = 98.5;

% Cấu hình Lissajous
NUM_POINTS = 150;
X_MIN = -45.0; X_MAX = 95.0; Y_MIN = 635.0; Y_MAX = 775.0; Z_MIN = -63.5; Z_MAX = -33.5;
XC = 35.0; YC = 695.0; ZC = -63.5;
Ax = (X_MAX - X_MIN) / 2; Ay = (Y_MAX - Y_MIN) / 2; Az = (Z_MAX - Z_MIN) / 2;
Cx = (X_MAX + X_MIN) / 2; Cy = (Y_MAX + Y_MIN) / 2; Cz = (Z_MAX + Z_MIN) / 2;
a = 3; b = 4; c = 2; delta = pi / 2;

% 1. Tính Quaternion Base Inline
hz = Rz_in*pi/360; hy = Ry_in*pi/360; hx = Rx_in*pi/360;
cz = cos(hz); sz = sin(hz); cy = cos(hy); sy = sin(hy); cx = cos(hx); sx = sin(hx);
w_b = cx*cy*cz+sx*sy*sz; x_b = sx*cy*cz-cx*sy*sz; y_b = cx*sy*cz+sx*cy*sz; z_b = cx*cy*sz-sx*sy*cz;
n_b = sqrt(w_b^2 + x_b^2 + y_b^2 + z_b^2); if n_b==0; n_b=1; w_b=1; x_b=0; y_b=0; z_b=0; end
qw = w_b/n_b; qx = x_b/n_b; qy = y_b/n_b; qz = z_b/n_b;
Quat_List = [qw, qx, qy, qz];

% 2. Tạo quỹ đạo Lissajous
Path_Tip = zeros(NUM_POINTS, 3);
for i = 1:NUM_POINTS
    t = 2 * pi * (i - 1) / NUM_POINTS;
    Path_Tip(i, :) = [Cx + Ax * sin(a * t + delta), Cy + Ay * sin(b * t), Cz + Az * sin(c * t)];
end
Path_Tip(1, :) = [XC, YC, ZC];

% 3. Bù trừ TCP Tool Inline
Path_Out = zeros(NUM_POINTS, 3); Quat_Out = zeros(NUM_POINTS, 4);
R11 = 1 - 2*(qy^2 + qz^2); R12 = 2*(qx*qy - qz*qw); R13 = 2*(qx*qz + qy*qw);
R21 = 2*(qx*qy + qz*qw);   R22 = 1 - 2*(qx^2 + qz^2); R23 = 2*(qy*qz - qx*qw);
R31 = 2*(qx*qz - qy*qw);   R32 = 2*(qy*qz + qx*qw);   R33 = 1 - 2*(qx^2 + qy^2);

dx = R11*TOOL_OFFSET_X + R12*TOOL_OFFSET_Y + R13*TOOL_OFFSET_Z;
dy = R21*TOOL_OFFSET_X + R22*TOOL_OFFSET_Y + R23*TOOL_OFFSET_Z;
dz = R31*TOOL_OFFSET_X + R32*TOOL_OFFSET_Y + R33*TOOL_OFFSET_Z;

for i = 1:NUM_POINTS
    Path_Out(i, :) = [Path_Tip(i,1) - dx, Path_Tip(i,2) - dy, Path_Tip(i,3) - dz] / 1000;
    Quat_Out(i, :) = Quat_List;
end

Euler_Out = repmat([Rx_in, Ry_in, Rz_in], NUM_POINTS, 1);