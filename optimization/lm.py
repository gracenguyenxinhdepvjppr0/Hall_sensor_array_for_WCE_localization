#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
magnet_lm.py - định vị nam châm 5 bậc tự do trên mảng 4x4 TMAG5170 bằng Levenberg-Marquardt.
Thuật toán theo inverse_linear.py: mô hình lưỡng cực + hiệu chuẩn 2 giai đoạn.

Dòng UART từ firmware (chỉ mảng từ trường thô, B tính bằng uT, cảm biến i = bus*8 + địa chỉ, i = 0..15):
  B,<frame>,<err_hex>,Bx[16],By[16],Bz[16]                                  (51 trường)
(Nếu firmware bật calib + nhiệt độ thì thêm ,T[16],<cal_hex>,<stale_hex> ở cuối: 69 trường; script bỏ qua phần thêm.)

TRẠNG THÁI (5 DOF, giống inverse_linear.py):   state = [x, y, z, pitch, yaw]     (x, y, z: mét;  pitch, yaw: rad)
  m_hat = [cos(pitch) cos(yaw), cos(pitch) sin(yaw), sin(pitch)]      vector đơn vị S -> N của nam châm
  pitch = góc ngẩng so với mặt phẳng xy (-90..+90 độ, +90 = thẳng lên theo +z)
  yaw   = góc phương vị trong mặt phẳng xy, từ +x sang +y (-180..+180 độ); không xác định khi |pitch| > ~87 độ ('n/a')
  tilt  = 90 - pitch (góc lệch so với +z, chỉ để so sánh).  Roll KHÔNG đo được (trường lưỡng cực đối xứng quanh trục).
  Độ lớn moment KHÔNG phải ẩn: nó nằm trong hệ số gain g (xem dưới).

MÔ HÌNH THUẬN (giống inverse_linear.py, mở rộng từ 1 trục Z sang 3 trục X, Y, Z của từng cảm biến):
  r         = sensor_pos - magnet_pos                                         [m]
  B_dip     = 1e-7 * ( 3 (m_hat . r) r / |r|^5 - m_hat / |r|^3 )              [T cho moment 1 A.m^2]
  h         = z_magnet - z_sensor                                             [m]
  alpha(h)  = c0 + c1 * h                                                     (c0, c1 chung cho mọi cảm biến, h tính bằng m)
  B_pred    = a + g * B_dip * alpha(h)                                        [mT]   (a, g riêng từng cảm biến và từng trục)
  residual  = B_pred - B_đo  (48 giá trị: 16 cảm biến x 3 trục)  ->  cực tiểu hóa 1/2 ||residual||^2 bằng LM.

HIỆU CHUẨN (giống 2 giai đoạn của inverse_linear.py):
  Stage 1 (--calib file.csv): vị trí cảm biến x, y, z (mét), offset a, gain g.
      Cột bắt buộc: x, y, z, offset, gain  (offset/gain dùng chung cho cả 3 trục của cảm biến đó);
      hoặc từng trục: offset_x, offset_y, offset_z, gain_x, gain_y, gain_z.   Đơn vị offset: mT.
  Stage 2 (--alpha file.csv): hai dòng  coefficient,value  với c0 và c1 (c1 theo 1/m).
  Không có file hiệu chuẩn thì dùng mặc định: vị trí theo bảng SENSOR_XY_MM, a = 0, g = moment * 1000 (--moment, A.m^2),
  c0 = 1, c1 = 0 (alpha = 1).  Khi đó độ lớn moment (--moment) phải đúng, nếu sai thì z và các giá trị |B| bị lệch.

Hình học mảng cảm biến mặc định (bảng SENSOR_XY_MM, đơn vị mm):
  - 16 cảm biến nằm trên mặt phẳng z = 15 mm (đổi bằng --z-plane), lưới 4x4 bước 30 mm.
  - SENSOR1..4  : x = 30,  y = 120, 90, 60, 30
    SENSOR5..8  : x = 60,  y = 120, 90, 60, 30
    SENSOR9..12 : x = 90,  y = 120, 90, 60, 30
    SENSOR13..16: x = 120, y = 120, 90, 60, 30
  - Chỉ số i của dòng UART (0..15) ứng với SENSOR(i+1).  NẾU thứ tự trong firmware khác, sửa bảng SENSOR_XY_MM.
  - trục X, Y, Z của mọi cảm biến song song với trục x, y, z của hệ tọa độ này.
  - mặc định nam châm nằm TRÊN mặt phẳng cảm biến (z > 15 mm); nếu nằm DƯỚI thì dùng --side below.

Cách dùng:
  python magnet_lm.py --port COM5 --baud 115200 [--z-plane 15] [--side above] [--moment 1.0] [--csv out.csv]
  python magnet_lm.py --port COM5 --calib Calibration_PHYSICAL.csv --alpha Calibration_Alpha.csv
  python magnet_lm.py --demo 200      # dữ liệu giả lập, không cần phần cứng
  python magnet_lm.py --demo 200 --offset 0.35   # giả lập offset cảm biến chưa hiệu chuẩn (±0.35 mT mỗi trục)
  python magnet_lm.py --selftest      # kiểm tra Jacobian, LM, mô hình hiệu chuẩn và bộ phân tích dòng
Cần: numpy;  pyserial (chỉ khi đọc cổng thật):  pip install numpy pyserial
"""
import argparse
import csv
import math
import sys
import time

import numpy as np

N_SENS = 16
Z_PLANE_DEFAULT_MM = 15.0   # mm, cao độ mặt phẳng cảm biến
MU0_4PI = 1e-7              # mu0 / 4pi
T_TO_MT = 1e3
MOMENT_DEFAULT = 1.0        # A.m^2  (~ quả cầu NdFeB bán kính 6 mm, Br ~ 1.3 T)

# Tọa độ (x, y) mm của SENSOR1..SENSOR16, đúng thứ tự chỉ số 0..15 của dòng UART.
SENSOR_XY_MM = np.array([
    [30, 120], [30, 90], [30, 60], [30, 30],        # SENSOR1..4
    [60, 120], [60, 90], [60, 60], [60, 30],        # SENSOR5..8
    [90, 120], [90, 90], [90, 60], [90, 30],        # SENSOR9..12
    [120, 120], [120, 90], [120, 60], [120, 30],    # SENSOR13..16
], dtype=float)

# Điểm xuất phát khi chưa có nghiệm trước: khoảng cách tới mặt phẳng cảm biến (m) x hướng (pitch, yaw) độ.
# Tránh pitch = ±90 vì ở đó yaw suy biến (Jacobian theo yaw bằng 0).
START_HEIGHTS_M = (0.010, 0.020, 0.040)
START_ANGLES_DEG = ((80.0, 0.0), (-80.0, 0.0), (0.0, 0.0), (0.0, 180.0), (0.0, 90.0), (0.0, -90.0))
PENALTY_K = 1e4             # mT trên mét: phạt khi nam châm gần mặt phẳng cảm biến hơn z_min (tương đương 10 mT/mm)


# --------------------------------------------------------------------------------------
# 1) Đọc và phân tích dòng UART
# --------------------------------------------------------------------------------------
def parse_line(line):
    """Trả về dict(frame, err, B(16,3) mT, T(16,) °C hoặc None, cal, stale) hoặc None nếu dòng hỏng."""
    parts = line.strip().split(',')
    if parts[0] != 'B' or len(parts) not in (51, 69):
        return None
    try:
        frame = int(parts[1])
        err = int(parts[2], 16)
        v = np.array([int(x) for x in parts[3:51]], dtype=float) / 1000.0        # uT -> mT
        out = dict(frame=frame, err=err, T=None, cal=None, stale=None)
        out['B'] = v.reshape(3, N_SENS).T                                           # (16, 3): cột = Bx, By, Bz
        if len(parts) == 69:
            out['T'] = np.array([int(x) for x in parts[51:67]], dtype=float) / 10.0
            out['cal'] = int(parts[67], 16)
            out['stale'] = int(parts[68], 16)
        return out
    except ValueError:
        return None


# --------------------------------------------------------------------------------------
# 2) Hiệu chuẩn và hình học
# --------------------------------------------------------------------------------------
def sensor_positions(z_plane_mm=Z_PLANE_DEFAULT_MM):
    """(16, 3) MÉT: tọa độ từng cảm biến theo bảng SENSOR_XY_MM, cùng nằm trên mặt phẳng z = z_plane_mm."""
    pos = np.zeros((N_SENS, 3))
    pos[:, :2] = SENSOR_XY_MM * 1e-3
    pos[:, 2] = z_plane_mm * 1e-3
    return pos


def _per_axis(v, n, name):
    v = np.asarray(v, dtype=float)
    if v.ndim == 1:
        v = np.repeat(v[:, None], 3, axis=1)
    if v.shape != (n, 3):
        raise ValueError('%s phải có kích thước (%d,) hoặc (%d, 3), nhận được %s' % (name, n, n, v.shape))
    return v


class Calibration:
    """
    Tham số của mô hình  B_pred = a + g * B_dip * alpha(h),  alpha(h) = c0 + c1 * h.
      pos : (N,3) m     vị trí cảm biến
      a   : (N,) hoặc (N,3) mT    offset            (mặc định 0)
      g   : (N,) hoặc (N,3)       gain              (mặc định moment[A.m^2] * 1000, đổi T -> mT)
      c0, c1                      hệ số alpha(h), h tính bằng m   (mặc định 1, 0)
    """
    def __init__(self, pos, a=None, g=None, c0=1.0, c1=0.0, moment=MOMENT_DEFAULT):
        self.pos = np.asarray(pos, dtype=float)
        n = self.pos.shape[0]
        self.a = np.zeros((n, 3)) if a is None else _per_axis(a, n, 'a')
        self.g = np.full((n, 3), moment * T_TO_MT) if g is None else _per_axis(g, n, 'g')
        self.c0, self.c1 = float(c0), float(c1)


def _read_csv_rows(path):
    with open(path, newline='', encoding='latin-1') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError('File rỗng: %s' % path)
    cols = {k.strip().lower(): k for k in rows[0].keys() if k is not None}
    return rows, cols


def load_stage1_calibration(path):
    """Stage 1: trả về (pos (N,3) m, a (N,3) mT, g (N,3)). Định dạng cột xem docstring đầu file."""
    rows, cols = _read_csv_rows(path)

    def col(name):
        return np.array([float(r[cols[name]]) for r in rows])

    missing = [c for c in ('x', 'y', 'z') if c not in cols]
    if missing:
        raise KeyError('Stage-1 CSV thiếu cột %s. Các cột hiện có: %s' % (missing, list(cols)))
    pos = np.stack([col('x'), col('y'), col('z')], axis=1)

    def per_axis(base):
        if base in cols:
            return col(base)
        ax = ['%s_%s' % (base, c) for c in 'xyz']
        if all(c in cols for c in ax):
            return np.stack([col(c) for c in ax], axis=1)
        raise KeyError("Stage-1 CSV cần cột '%s' hoặc %s. Các cột hiện có: %s" % (base, ax, list(cols)))

    n = pos.shape[0]
    return pos, _per_axis(per_axis('offset'), n, 'offset'), _per_axis(per_axis('gain'), n, 'gain')


def load_stage2_alpha(path):
    """Stage 2: file 'coefficient,value' với hai dòng c0, c1. Trả về (c0, c1)."""
    rows, cols = _read_csv_rows(path)
    if 'coefficient' not in cols or 'value' not in cols:
        raise KeyError('Stage-2 CSV cần cột coefficient,value. Các cột hiện có: %s' % list(cols))
    vals = {str(r[cols['coefficient']]).strip(): float(r[cols['value']]) for r in rows}
    for k in ('c0', 'c1'):
        if k not in vals:
            raise ValueError('Không tìm thấy coefficient = %s' % k)
    return vals['c0'], vals['c1']


# --------------------------------------------------------------------------------------
# 3) Mô hình lưỡng cực, mô hình thuận và Jacobian
# --------------------------------------------------------------------------------------
YAW_VALID_RATIO = 0.05      # hypot(mx,my)/|m| nhỏ hơn mức này (pitch > ~87 độ) -> yaw không xác định


def moment_vec(pitch, yaw):
    """Vector đơn vị S -> N từ pitch, yaw (rad)."""
    cp = math.cos(pitch)
    return np.array([cp * math.cos(yaw), cp * math.sin(yaw), math.sin(pitch)])


def moment_angles(m):
    """(pitch, yaw, tilt, yaw_hợp_lệ) của vector m. Góc tính bằng độ, pitch -90..90, yaw -180..180."""
    mx, my, mz = (float(v) for v in m)
    rho = math.hypot(mx, my)
    n = math.hypot(rho, mz)
    pitch = math.degrees(math.atan2(mz, rho))
    yaw = math.degrees(math.atan2(my, mx))
    return pitch, yaw, 90.0 - pitch, bool(n > 0.0 and rho / n >= YAW_VALID_RATIO)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def dipole_field(p, pos):
    """F = (3 (m.r) r / |r|^5 - m / |r|^3) tại pos (N,3), p = [x,y,z,mx,my,mz]; r = pos - p[:3].  B[T] = 1e-7 * F * |m|[A.m^2]."""
    r = pos - p[:3]
    d2 = np.maximum(np.einsum('ij,ij->i', r, r), 1e-12)
    m = p[3:]
    mr = r @ m
    return (3.0 * mr[:, None] * r / d2[:, None] - m) / (d2 * np.sqrt(d2))[:, None]


def dipole_jacobian(p, pos):
    """dF/dp, dạng (N*3, 6), thứ tự hàng khớp với F.ravel()."""
    r = pos - p[:3]
    d2 = np.maximum(np.einsum('ij,ij->i', r, r), 1e-12)
    d = np.sqrt(d2)
    d3, d5, d7 = d2 * d, d2 * d2 * d, d2 * d2 * d2 * d
    m = p[3:]
    mr = r @ m
    eye = np.eye(3)
    rr = np.einsum('ni,nj->nij', r, r)
    dB_dm = (3.0 * rr / d2[:, None, None] - eye) / d3[:, None, None]
    dB_dr = (3.0 * np.einsum('j,ni->nij', m, r) / d5[:, None, None]
             + 3.0 * mr[:, None, None] * eye / d5[:, None, None]
             - 15.0 * mr[:, None, None] * rr / d7[:, None, None]
             + 3.0 * np.einsum('i,nj->nij', m, r) / d5[:, None, None])
    return np.concatenate([-dB_dr, dB_dm], axis=2).reshape(-1, 6)       # r = pos - p_mag nên dấu âm


def forward_model(state, pos, a, g, c0, c1):
    """B_pred (N,3) mT = a + g * B_dip * alpha(h)  -  đúng công thức V_pred của inverse_linear.py (cho cả 3 trục)."""
    p6 = np.concatenate([state[:3], moment_vec(state[3], state[4])])
    B = MU0_4PI * dipole_field(p6, pos)                                    # T cho moment 1 A.m^2
    alpha = c0 + c1 * (state[2] - pos[:, 2])                               # h = z_magnet - z_sensor
    return a + g * B * alpha[:, None]


def forward_jacobian(state, pos, g, c0, c1):
    """d B_pred / d [x, y, z, pitch, yaw], dạng (N*3, 5), thứ tự hàng khớp với B_pred.ravel()."""
    pitch, yaw = state[3], state[4]
    sp, cp, sy, cy = math.sin(pitch), math.cos(pitch), math.sin(yaw), math.cos(yaw)
    dm_dpitch = np.array([-sp * cy, -sp * sy, cp])
    dm_dyaw = np.array([-cp * sy, cp * cy, 0.0])
    p6 = np.concatenate([state[:3], np.array([cp * cy, cp * sy, sp])])
    F = dipole_field(p6, pos)                                              # (N,3)
    J6 = dipole_jacobian(p6, pos).reshape(-1, 3, 6)                        # (N,3,6)
    alpha = c0 + c1 * (state[2] - pos[:, 2])                               # (N,)
    scale = MU0_4PI * g * alpha[:, None]                                   # (N,3)
    J = np.empty((pos.shape[0], 3, 5))
    J[:, :, 0] = scale * J6[:, :, 0]
    J[:, :, 1] = scale * J6[:, :, 1]
    J[:, :, 2] = scale * J6[:, :, 2] + MU0_4PI * g * c1 * F                # alpha phụ thuộc z: d alpha / dz = c1
    J[:, :, 3] = scale * (J6[:, :, 3:6] @ dm_dpitch)
    J[:, :, 4] = scale * (J6[:, :, 3:6] @ dm_dyaw)
    return J.reshape(-1, 5)


def angle_sigma_deg(var_rad2):
    return math.degrees(math.sqrt(max(float(var_rad2), 0.0)))


# --------------------------------------------------------------------------------------
# 4) Levenberg-Marquardt tổng quát
# --------------------------------------------------------------------------------------
def numeric_jacobian(fun, p):
    """Sai phân trung tâm."""
    cols = []
    for j in range(p.size):
        h = 1e-5 * max(abs(p[j]), 1.0)
        pp, pm = p.copy(), p.copy()
        pp[j] += h
        pm[j] -= h
        cols.append((fun(pp) - fun(pm)) / (2.0 * h))
    return np.stack(cols, axis=1)


def levenberg_marquardt(fun, p0, jac=None, lam0=1e-3, max_iter=100,
                        gtol=1e-10, xtol=1e-10, ftol=1e-12):
    """
    Cực tiểu hóa  C(p) = 1/2 * ||fun(p)||^2  bằng Levenberg-Marquardt (co giãn Marquardt, cập nhật lambda kiểu Nielsen).
    Tương đương scipy.optimize.least_squares(method='lm') mà inverse_linear.py dùng, nhưng chỉ cần numpy và có Jacobian giải tích.

      (J^T J + lam * diag(J^T J)) dp = -J^T r
      rho = (C(p) - C(p + dp)) / (dự đoán giảm),  dự đoán giảm = 1/2 dp^T (lam * D dp - g)
      rho > 0  : chấp nhận bước,  lam *= max(1/3, 1 - (2 rho - 1)^3)
      rho <= 0 : từ chối bước,    lam *= nu,  nu *= 2   (tiến gần về hướng gradient)

    fun(p) -> vector residual;  jac(p) -> ma trận Jacobian (nếu None thì dùng sai phân số).
    Trả về dict(p, cost, r, J, iterations, nfev, status).
    """
    jac = jac or (lambda q: numeric_jacobian(fun, q))
    p = np.array(p0, dtype=float)
    r = fun(p)
    nfev = 1
    cost = 0.5 * float(r @ r)
    J = jac(p)
    A, g = J.T @ J, J.T @ r
    lam, nu = lam0, 2.0
    status, it, done = 'max_iter', 0, False

    while it < max_iter and not done:
        if np.max(np.abs(g)) <= gtol:
            status = 'gradient'
            break
        Dv = np.maximum(np.diag(A), 1e-12)
        while True:                                          # tìm lam cho bước được chấp nhận
            try:
                dp = np.linalg.solve(A + lam * np.diag(Dv), -g)
            except np.linalg.LinAlgError:
                dp = None
            if dp is not None and np.all(np.isfinite(dp)):
                if np.linalg.norm(dp) <= xtol * (np.linalg.norm(p) + xtol):
                    status, done = 'step', True
                    break
                r_new = fun(p + dp)
                nfev += 1
                cost_new = 0.5 * float(r_new @ r_new)
                pred = 0.5 * float(dp @ (lam * Dv * dp - g))
                rho = (cost - cost_new) / pred if pred > 0 else -1.0
                if rho > 0 and np.isfinite(cost_new):
                    dcost = cost - cost_new
                    p, r, cost = p + dp, r_new, cost_new
                    J = jac(p)
                    A, g = J.T @ J, J.T @ r
                    lam *= max(1.0 / 3.0, 1.0 - (2.0 * rho - 1.0) ** 3)
                    nu = 2.0
                    if dcost <= ftol * max(cost, 1e-300):
                        status, done = 'cost', True
                    break
            lam *= nu
            nu *= 2.0
            if lam > 1e16:
                status, done = 'lambda', True
                break
        it += 1
    return dict(p=p, cost=cost, r=r, J=J, iterations=it, nfev=nfev, status=status)


# --------------------------------------------------------------------------------------
# 5) Bộ định vị nam châm (5 DOF)
# --------------------------------------------------------------------------------------
class MagnetTracker:
    def __init__(self, calib, z_min=1.0, min_field=1.0, rms_ok=0.15, side=+1):
        """
        calib     : Calibration (vị trí cảm biến, a, g, c0, c1).
        z_min     : khoảng cách tối thiểu (mm) từ nam châm tới mặt phẳng cảm biến (phạt nếu nhỏ hơn).
        min_field : mT. |B - a| lớn nhất nhỏ hơn mức này -> coi như không có nam châm
                    (chưa hiệu chuẩn: offset cảm biến tới ±0.35 mT theo datasheet, nên để > 0.35).
        rms_ok    : mT. rms vượt mức này thì thử lại nhiều điểm xuất phát.
        side      : +1 nam châm ở phía z > z_plane (trên mặt cảm biến); -1 ở phía z < z_plane (dưới).
        """
        self.calib = calib
        self.z_min = z_min * 1e-3
        self.min_field, self.rms_ok = min_field, rms_ok
        self.side = 1.0 if side >= 0 else -1.0
        self.z_plane = float(np.mean(calib.pos[:, 2]))
        self.p_prev = None
        self.K = PENALTY_K

    def height(self, z):
        """Khoảng cách có dấu (m) từ mặt phẳng cảm biến, dương về phía nam châm."""
        return self.side * (z - self.z_plane)

    def _functions(self, B, valid):
        c = self.calib
        pos_v, a_v, g_v, B_v = c.pos[valid], c.a[valid], c.g[valid], B[valid]
        c0, c1 = c.c0, c.c1
        zmin, K, sd, zp = self.z_min, self.K, self.side, self.z_plane

        def fun(s):
            pen = K * (zmin - sd * (s[2] - zp)) if sd * (s[2] - zp) < zmin else 0.0
            return np.append((forward_model(s, pos_v, a_v, g_v, c0, c1) - B_v).ravel(), pen)

        def jac(s):
            J = forward_jacobian(s, pos_v, g_v, c0, c1)
            row = np.zeros((1, 5))
            if sd * (s[2] - zp) < zmin:
                row[0, 2] = -K * sd
            return np.vstack([J, row])

        return fun, jac

    def _starts(self, B, valid):
        c = self.calib
        mag = np.linalg.norm(B[valid] - c.a[valid], axis=1)
        w = mag ** 2 + 1e-12
        cx, cy = (w[:, None] * c.pos[valid][:, :2]).sum(0) / w.sum()
        out = []
        for h0 in START_HEIGHTS_M:
            for pitch_d, yaw_d in START_ANGLES_DEG:
                out.append(np.array([cx, cy, self.z_plane + self.side * h0,
                                     math.radians(pitch_d), math.radians(yaw_d)]))
        return out

    def fit(self, B, valid=None):
        """B: (16,3) mT. valid: mặt nạ cảm biến dùng được. Trả về dict kết quả hoặc None nếu không có nam châm."""
        valid = np.ones(N_SENS, bool) if valid is None else np.asarray(valid, bool)
        if valid.sum() < 4:
            return None
        if np.linalg.norm(B[valid] - self.calib.a[valid], axis=1).max() < self.min_field:
            self.p_prev = None
            return None
        fun, jac = self._functions(B, valid)
        best = None
        if self.p_prev is not None:                  # theo dấu: bắt đầu từ nghiệm trước
            best = levenberg_marquardt(fun, self.p_prev, jac)
        if best is None or self._rms(best) > self.rms_ok:
            for s0 in self._starts(B, valid):        # khung đầu hoặc mất dấu: thử nhiều điểm xuất phát
                res = levenberg_marquardt(fun, s0, jac)
                if best is None or res['cost'] < best['cost']:
                    best = res
        out = self._summary(best, valid)
        self.p_prev = out['state'].copy()
        return out

    @staticmethod
    def _rms(res):
        return float(np.sqrt(np.mean(res['r'][:-1] ** 2)))      # bỏ hàng phạt cuối

    def _summary(self, res, valid):
        s = res['p']
        n_res = 3 * int(valid.sum())
        dof = max(n_res - 5, 1)
        s2 = 2.0 * res['cost'] / dof
        try:
            cov = np.linalg.pinv(res['J'].T @ res['J']) * s2
        except np.linalg.LinAlgError:
            cov = np.full((5, 5), np.nan)
        var = np.diag(cov)
        pitch, yaw, tilt, yaw_ok = moment_angles(moment_vec(s[3], s[4]))      # chuẩn hóa: pitch -90..90, yaw -180..180
        state = np.array([s[0], s[1], s[2], math.radians(pitch), math.radians(yaw)])
        return dict(state=state, pos_mm=s[:3] * 1e3, h_mm=self.height(s[2]) * 1e3,
                    pitch=pitch, yaw=yaw if yaw_ok else float('nan'), tilt=tilt, yaw_ok=yaw_ok,
                    sigma_pos_mm=np.sqrt(np.maximum(var[:3], 0.0)) * 1e3,
                    sigma_pitch=angle_sigma_deg(var[3]),
                    sigma_yaw=angle_sigma_deg(var[4]) if yaw_ok else float('nan'),
                    rms=self._rms(res), iterations=res['iterations'], nfev=res['nfev'], status=res['status'])


# --------------------------------------------------------------------------------------
# 6) Nguồn dữ liệu: cổng nối tiếp thật hoặc giả lập
# --------------------------------------------------------------------------------------
def encode_line(frame, B_mT, err=0, T_C=None, cal=0xFFFF, stale=0):
    """Tạo dòng UART đúng định dạng firmware (làm tròn về uT nguyên như firmware). T_C=None: 51 trường."""
    ut = np.rint(np.asarray(B_mT) * 1000.0).astype(int)
    f = ['B', str(frame), '%04X' % err]
    f += [str(v) for v in ut[:, 0]] + [str(v) for v in ut[:, 1]] + [str(v) for v in ut[:, 2]]
    if T_C is not None:
        f += [str(int(round(t * 10))) for t in T_C] + ['%04X' % cal, '%04X' % stale]
    return ','.join(f)


def demo_lines(n, calib, noise_mT, seed=1, offset_mT=0.0, side=1.0):
    """Nam châm di chuyển theo đường tròn trên mảng, độ cao và hướng thay đổi; trả về (dòng, chân trị state 5 phần tử)."""
    rng = np.random.default_rng(seed)
    offsets = rng.uniform(-offset_mT, offset_mT, (N_SENS, 3))        # offset cố định của từng cảm biến, KHÔNG có trong mô hình
    cx, cy = calib.pos[:, 0].mean(), calib.pos[:, 1].mean()          # tâm mảng
    zp = calib.pos[:, 2].mean()
    for k in range(n):
        t = 2.0 * np.pi * k / max(n, 1)
        pitch, yaw = np.radians(50.0 + 30.0 * np.sin(t)), 3.0 * t - np.pi            # pitch 20..80 độ, yaw quay 3 vòng
        h = 0.022 + 0.006 * np.sin(2 * t)                                           # cách mặt phẳng cảm biến 16..28 mm
        s = np.array([cx + 0.025 * np.cos(t), cy + 0.025 * np.sin(t), zp + side * h, pitch, yaw])
        B = (forward_model(s, calib.pos, calib.a, calib.g, calib.c0, calib.c1)
             + offsets + rng.normal(0.0, noise_mT, (N_SENS, 3)))
        yield encode_line(k, B), s


def serial_lines(port, baud):
    try:
        import serial                                # pyserial
    except ImportError:
        sys.exit('Thiếu pyserial:  pip install pyserial')
    ser = serial.Serial(port, baud, timeout=1)
    ser.reset_input_buffer()
    while True:
        raw = ser.readline()
        if raw:
            yield raw.decode('ascii', errors='ignore'), None


# --------------------------------------------------------------------------------------
# 7) Vòng chính
# --------------------------------------------------------------------------------------
def run(source, tracker, csv_path=None, quiet=False):
    csv_f = open(csv_path, 'w') if csv_path else None
    if csv_f:
        csv_f.write('frame,x_mm,y_mm,z_mm,pitch_deg,yaw_deg,tilt_deg,rms_mT,iters\n')
    errs = []
    try:
        for line, truth in source:
            d = parse_line(line)
            if d is None:
                continue
            valid = np.array([(d['err'] >> i) & 1 == 0 for i in range(N_SENS)])
            t0 = time.perf_counter()
            res = tracker.fit(d['B'], valid)
            dt = (time.perf_counter() - t0) * 1e3
            if res is None:
                if not quiet:
                    print('frame %6d: không có nam châm' % d['frame'])
                continue
            x, y, z = res['pos_mm']
            if truth is not None:
                tp, ty, _, ty_ok = moment_angles(moment_vec(truth[3], truth[4]))
                errs.append((np.linalg.norm(res['pos_mm'] - truth[:3] * 1e3), res['rms'], abs(res['pitch'] - tp),
                             abs(wrap180(res['yaw'] - ty)) if (ty_ok and res['yaw_ok']) else float('nan')))
            if not quiet:
                yaw_txt = '%7.1f' % res['yaw'] if res['yaw_ok'] else '    n/a'
                sig_txt = ('±%.1f ±%.1f' % (res['sigma_pitch'], res['sigma_yaw']) if res['yaw_ok']
                           else '±%.1f ±n/a' % res['sigma_pitch'])
                print('frame %6d: x=%7.2f y=%7.2f z=%6.2f mm | pitch=%6.1f yaw=%s deg (%s) | '
                      'rms=%5.1f uT | ±%.2f mm | %2d it %5.1f ms %s'
                      % (d['frame'], x, y, z, res['pitch'], yaw_txt, sig_txt,
                         res['rms'] * 1e3, res['sigma_pos_mm'].max(), res['iterations'], dt, res['status']))
            if csv_f:
                csv_f.write('%d,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.5f,%d\n'
                            % (d['frame'], x, y, z, res['pitch'], res['yaw'], res['tilt'], res['rms'], res['iterations']))
    except KeyboardInterrupt:
        pass
    finally:
        if csv_f:
            csv_f.close()
    return errs


# --------------------------------------------------------------------------------------
# 8) Tự kiểm tra
# --------------------------------------------------------------------------------------
def _random_state(rng, zp, side=1.0, h_range=(0.015, 0.040)):
    """Tư thế ngẫu nhiên: x, y trong vùng mảng, cách mặt phẳng cảm biến 15..40 mm, hướng đều trên mặt cầu."""
    pitch, yaw, _, _ = moment_angles(rng.normal(size=3))
    return np.array([*rng.uniform(0.04, 0.11, 2), zp + side * rng.uniform(*h_range),
                     math.radians(pitch), math.radians(yaw)])


def selftest():
    ok = True
    rng = np.random.default_rng(0)
    pos = sensor_positions()
    zp = pos[0, 2]
    calib0 = Calibration(pos)                                     # mặc định: a = 0, g = 1 A.m^2, alpha = 1

    # 0) bảng tọa độ khớp với đề bài (đổi sang mét)
    mm = pos * 1e3
    good = (pos.shape == (16, 3) and np.allclose(mm[:, 2], Z_PLANE_DEFAULT_MM)
            and np.allclose(mm[0, :2], (30, 120)) and np.allclose(mm[3, :2], (30, 30))
            and np.allclose(mm[4, :2], (60, 120)) and np.allclose(mm[9, :2], (90, 90))
            and np.allclose(mm[12, :2], (120, 120)) and np.allclose(mm[15, :2], (120, 30))
            and len({tuple(np.round(q, 6)) for q in mm[:, :2]}) == 16)
    print('Bảng tọa độ cảm biến (16 điểm khác nhau, mặt phẳng z = %.0f mm): %s' % (zp * 1e3, 'đạt' if good else 'LỖI'))
    ok &= good

    # a) bộ phân tích dòng
    B0 = rng.normal(0, 5, (N_SENS, 3))
    line69 = encode_line(7, B0, err=0x0020, T_C=np.full(N_SENS, 25.3), cal=0xFFDF, stale=0x0001)
    d = parse_line(line69)
    good = (d is not None and d['frame'] == 7 and d['err'] == 0x20 and d['cal'] == 0xFFDF and d['stale'] == 1
            and np.allclose(d['B'], np.rint(B0 * 1000) / 1000) and np.allclose(d['T'], 25.3))
    line51 = encode_line(7, B0, err=0x0020)
    d51 = parse_line(line51)
    good &= len(line51.split(',')) == 51 and d51 is not None and d51['T'] is None and np.allclose(d51['B'], d['B'])
    good &= parse_line('B,1,0000,1,2,3') is None and parse_line('garbage') is None and parse_line('B,x') is None
    print('parse_line (69 và 51 trường, dòng hỏng): %s' % ('đạt' if good else 'LỖI'))
    ok &= good

    # b) Jacobian giải tích của mô hình thuận (có a, g, c0, c1 khác mặc định) so với sai phân số
    cal_t = Calibration(pos, a=rng.uniform(-0.3, 0.3, (16, 3)), g=1e3 * (1 + rng.uniform(-0.05, 0.05, (16, 3))),
                        c0=0.8, c1=3.0)
    s = np.array([0.07, 0.08, zp + 0.025, math.radians(35.0), math.radians(-70.0)])
    Ja = forward_jacobian(s, pos, cal_t.g, cal_t.c0, cal_t.c1)
    Jn = numeric_jacobian(lambda q: forward_model(q, pos, cal_t.a, cal_t.g, cal_t.c0, cal_t.c1).ravel(), s)
    err = np.max(np.max(np.abs(Ja - Jn), axis=0) / np.max(np.abs(Jn), axis=0))
    print('Jacobian giải tích vs số (5 tham số, có a, g, alpha(h)): sai lệch tương đối lớn nhất %.2e  %s'
          % (err, 'đạt' if err < 1e-5 else 'LỖI'))
    ok &= err < 1e-5

    # c) LM trên hàm Rosenbrock (kiểm tra riêng bộ LM)
    def rosen(q):
        return np.array([10.0 * (q[1] - q[0] ** 2), 1.0 - q[0]])
    res = levenberg_marquardt(rosen, [-1.2, 1.0])
    good = np.allclose(res['p'], [1.0, 1.0], atol=1e-6)
    print('LM trên Rosenbrock: p = %s sau %d vòng  %s' % (np.round(res['p'], 6), res['iterations'], 'đạt' if good else 'LỖI'))
    ok &= good

    # d) định vị: 200 tư thế ngẫu nhiên, mô hình mặc định, nhiễu 0.03 mT, bắt đầu từ trạng thái mất dấu
    n, bad, pe, ap, ay = 200, 0, [], [], []
    for _ in range(n):
        st = _random_state(rng, zp)
        B = forward_model(st, pos, calib0.a, calib0.g, calib0.c0, calib0.c1) + rng.normal(0, 0.03, (N_SENS, 3))
        B = np.rint(B * 1000) / 1000
        r = MagnetTracker(calib0, min_field=0.1).fit(B)
        e = np.linalg.norm(r['pos_mm'] - st[:3] * 1e3) if r else np.inf
        pe.append(e)
        bad += e > 1.0
        tp, ty, _, ty_ok = moment_angles(moment_vec(st[3], st[4]))
        ap.append(abs(r['pitch'] - tp))
        if ty_ok and r['yaw_ok']:
            ay.append(abs(wrap180(r['yaw'] - ty)))
    pe, ap, ay = np.array(pe), np.array(ap), np.array(ay)
    print('Định vị 200 tư thế ngẫu nhiên: sai số vị trí trung vị %.3f mm, p95 %.3f mm, %d/%d lần > 1 mm  %s'
          % (np.median(pe), np.percentile(pe, 95), bad, n, 'đạt' if bad <= 2 else 'LỖI'))
    ok &= bad <= 2
    good = np.percentile(ap, 95) < 3.0 and np.percentile(ay, 95) < 3.0
    print('  góc: pitch trung vị %.2f độ (p95 %.2f), yaw trung vị %.2f độ (p95 %.2f)  %s'
          % (np.median(ap), np.percentile(ap, 95), np.median(ay), np.percentile(ay, 95), 'đạt' if good else 'LỖI'))
    ok &= good

    # d1) mô hình có hiệu chuẩn: a, g riêng từng cảm biến/trục, alpha(h) = c0 + c1*h
    bad_c, pe_c = 0, []
    for _ in range(100):
        st = _random_state(rng, zp)
        B = forward_model(st, pos, cal_t.a, cal_t.g, cal_t.c0, cal_t.c1) + rng.normal(0, 0.03, (N_SENS, 3))
        r = MagnetTracker(cal_t, min_field=0.1).fit(B)
        e = np.linalg.norm(r['pos_mm'] - st[:3] * 1e3) if r else np.inf
        pe_c.append(e)
        bad_c += e > 1.0
    print('Mô hình hiệu chuẩn (a, g, c0 = 0.8, c1 = 3.0/m), 100 tư thế: trung vị %.3f mm, %d/100 lần > 1 mm  %s'
          % (np.median(pe_c), bad_c, 'đạt' if bad_c <= 2 else 'LỖI'))
    ok &= bad_c <= 2

    # d2) quy ước góc (pitch = góc ngẩng so với mặt phẳng xy, yaw = phương vị từ +x sang +y)
    cases = [((1, 0, 0), 0, 0, True), ((0, 1, 0), 0, 90, True), ((-1, 0, 0), 0, 180, True), ((0, -1, 0), 0, -90, True),
             ((1, 0, 1), 45, 0, True), ((0, 0, 1), 90, None, False), ((0, 0, -1), -90, None, False),
             ((-1, -1, -np.sqrt(2)), -45, -135, True)]
    good = True
    for m, ep, ey, eok in cases:
        pch, yw, tilt, vok = moment_angles(m)
        good &= abs(pch - ep) < 1e-9 and vok == eok and (ey is None or abs(wrap180(yw - ey)) < 1e-9) and abs(tilt - (90 - ep)) < 1e-9
        if ey is not None:                                    # moment_vec là nghịch đảo của moment_angles
            good &= np.allclose(moment_vec(math.radians(ep), math.radians(ey)), np.asarray(m) / np.linalg.norm(m))
    print('Quy ước pitch/yaw/tilt trên 8 trường hợp đặc biệt: %s' % ('đạt' if good else 'LỖI'))
    ok &= good

    # d3) độ lệch chuẩn ước lượng có khớp sai số thực không (Monte Carlo 300 lần, cùng một tư thế)
    st = np.array([0.07, 0.08, zp + 0.020, math.radians(40.0), math.radians(60.0)])
    clean = forward_model(st, pos, calib0.a, calib0.g, calib0.c0, calib0.c1)
    pit, yw, sp, sy = [], [], [], []
    for _ in range(300):
        r = MagnetTracker(calib0, min_field=0.1).fit(clean + rng.normal(0, 0.05, (N_SENS, 3)))
        pit.append(r['pitch']); yw.append(r['yaw']); sp.append(r['sigma_pitch']); sy.append(r['sigma_yaw'])
    emp_p, emp_y = np.std(pit), np.std(yw)
    pred_p, pred_y = np.mean(sp), np.mean(sy)
    good = abs(emp_p / pred_p - 1) < 0.25 and abs(emp_y / pred_y - 1) < 0.25
    print('σ góc (nhiễu 0.05 mT, pitch 40, yaw 60): đo %.3f / %.3f độ, ước lượng %.3f / %.3f độ (pitch / yaw)  %s'
          % (emp_p, emp_y, pred_p, pred_y, 'đạt' if good else 'LỖI'))
    ok &= good

    # e) cảm biến lỗi: bỏ 3 cảm biến, kết quả vẫn đúng
    st = np.array([0.085, 0.065, zp + 0.020, math.radians(60.0), math.radians(20.0)])
    B = forward_model(st, pos, calib0.a, calib0.g, calib0.c0, calib0.c1)
    valid = np.ones(N_SENS, bool)
    valid[[2, 9, 14]] = False
    B[~valid] = 999.0
    r = MagnetTracker(calib0, min_field=0.1).fit(B, valid)
    e = np.linalg.norm(r['pos_mm'] - st[:3] * 1e3)
    print('Bỏ 3 cảm biến lỗi (giá trị rác 999 mT): sai số vị trí %.4f mm  %s' % (e, 'đạt' if e < 0.01 else 'LỖI'))
    ok &= e < 0.01

    # f) nam châm nằm DƯỚI mặt phẳng cảm biến (--side below)
    bad_below = 0
    for _ in range(50):
        st = _random_state(rng, zp, side=-1.0)
        B = np.rint((forward_model(st, pos, calib0.a, calib0.g, calib0.c0, calib0.c1)
                     + rng.normal(0, 0.03, (N_SENS, 3))) * 1000) / 1000
        r = MagnetTracker(calib0, min_field=0.1, side=-1).fit(B)
        bad_below += (r is None) or np.linalg.norm(r['pos_mm'] - st[:3] * 1e3) > 1.0
    print('Nam châm dưới mặt phẳng cảm biến (side = -1): %d/50 lần > 1 mm  %s' % (bad_below, 'đạt' if bad_below <= 1 else 'LỖI'))
    ok &= bad_below <= 1

    print('KẾT QUẢ:', 'ĐẠT' if ok else 'CÓ LỖI')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description='Định vị nam châm 5 DOF trên mảng 4x4 TMAG5170 bằng Levenberg-Marquardt')
    ap.add_argument('--port', help='cổng nối tiếp, ví dụ COM5 hoặc /dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--z-plane', type=float, default=Z_PLANE_DEFAULT_MM,
                    help='cao độ mặt phẳng cảm biến (mm) cho bảng tọa độ mặc định, mặc định %.0f' % Z_PLANE_DEFAULT_MM)
    ap.add_argument('--side', choices=('above', 'below'), default='above',
                    help='nam châm nằm trên (z > z-plane) hay dưới (z < z-plane) mặt phẳng cảm biến')
    ap.add_argument('--zmin', type=float, default=1.0,
                    help='khoảng cách tối thiểu từ nam châm tới mặt phẳng cảm biến (mm)')
    ap.add_argument('--moment', type=float, default=MOMENT_DEFAULT,
                    help='độ lớn moment nam châm (A.m^2), chỉ dùng khi không có --calib: g = moment * 1000')
    ap.add_argument('--calib', help='CSV hiệu chuẩn Stage 1: x, y, z (m), offset, gain (xem docstring)')
    ap.add_argument('--alpha', help='CSV hiệu chuẩn Stage 2: coefficient,value với c0 và c1 (c1 theo 1/m)')
    ap.add_argument('--min-field', type=float, default=1.0,
                    help='|B - a| lớn nhất dưới mức này (mT) = không có nam châm. Giữ > 0.35 vì offset cảm biến chưa hiệu chuẩn tới ±0.35 mT')
    ap.add_argument('--csv', help='ghi kết quả ra file CSV')
    ap.add_argument('--demo', type=int, metavar='N', help='chạy N khung dữ liệu giả lập')
    ap.add_argument('--noise', type=float, default=0.03, help='nhiễu giả lập (mT)')
    ap.add_argument('--offset', type=float, default=0.0,
                    help='giả lập offset ngẫu nhiên mỗi cảm biến/trục KHÔNG có trong mô hình, ± giá trị này (mT)')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()

    if a.selftest:
        sys.exit(selftest())

    if a.calib:
        pos, off, gain = load_stage1_calibration(a.calib)
        if pos.shape[0] != N_SENS:
            sys.exit('--calib phải có đúng %d cảm biến, file có %d' % (N_SENS, pos.shape[0]))
        calib = Calibration(pos, a=off, g=gain)
    else:
        calib = Calibration(sensor_positions(a.z_plane), moment=a.moment)
    if a.alpha:
        calib.c0, calib.c1 = load_stage2_alpha(a.alpha)

    side = 1.0 if a.side == 'above' else -1.0
    tracker = MagnetTracker(calib, z_min=a.zmin, min_field=a.min_field, side=side)
    if a.demo:
        errs = run(demo_lines(a.demo, calib, a.noise, offset_mT=a.offset, side=side), tracker, a.csv, quiet=False)
        e = np.array(errs)
        print('\nGiả lập %d khung: sai số vị trí trung bình %.3f mm, lớn nhất %.3f mm' % (len(e), e[:, 0].mean(), e[:, 0].max()))
        print('                 sai số pitch trung bình %.2f độ (lớn nhất %.2f), yaw trung bình %.2f độ (lớn nhất %.2f)'
              % (e[:, 2].mean(), e[:, 2].max(), np.nanmean(e[:, 3]), np.nanmax(e[:, 3])))
        return
    if not a.port:
        ap.error('cần --port (hoặc dùng --demo / --selftest)')
    run(serial_lines(a.port, a.baud), tracker, a.csv)


if __name__ == '__main__':
    main()