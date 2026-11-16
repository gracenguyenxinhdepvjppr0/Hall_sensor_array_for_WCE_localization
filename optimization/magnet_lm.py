#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
magnet_lm.py - định vị nam châm trên mảng 4x4 TMAG5170 bằng Levenberg-Marquardt.

Dòng UART từ firmware (chỉ mảng từ trường thô, B tính bằng uT, cảm biến i = hàng*4 + cột = bus*8 + địa chỉ):
  B,<frame>,<err_hex>,Bx[16],By[16],Bz[16]                                  (51 trường)
(Nếu firmware bật calib + nhiệt độ thì thêm ,T[16],<cal_hex>,<stale_hex> ở cuối: 69 trường; script bỏ qua phần thêm.)
Giá trị chưa calib: offset mỗi cảm biến tới ±0.35 mT (datasheet, dải 100 mT) nằm lẫn trong dữ liệu.

Mô hình: nam châm là một lưỡng cực từ ở (x, y, z) với moment m = (mx, my, mz):
    B(r) = ( 3 (m . r̂) r̂ - m ) / |r|^3 ,   r = vị trí cảm biến - vị trí nam châm
Đơn vị: vị trí mm, B mT, nên m có đơn vị mT*mm^3 (đã gộp µ0/4π).
Ví dụ quả cầu bán kính R (mm), từ dư Br (mT):  |m| = Br * R^3 / 3.
6 tham số p = [x, y, z, mx, my, mz] được fit từ 16 x 3 = 48 giá trị đo.

Góc của nam châm (từ vector moment m, chiều S -> N, hệ tọa độ của mảng: x, y trên mặt mảng, z hướng lên):
  pitch = góc ngẩng so với mặt phẳng xy = atan2(mz, hypot(mx, my)),  -90..+90 độ  (+90 = thẳng lên theo +z)
  yaw   = góc phương vị trong mặt phẳng xy, từ +x sang +y = atan2(my, mx),  -180..+180 độ
  tilt  = góc lệch so với +z = 90 - pitch  (chỉ để tiện so sánh)
  Roll (quay quanh trục nam châm) KHÔNG đo được: trường lưỡng cực đối xứng quanh trục.
  Khi nam châm gần thẳng đứng (|pitch| > ~87 độ) thì yaw không xác định (in 'n/a').

Giả định hình học (SỬA CHO ĐÚNG MẠCH CỦA BẠN):
  - 16 cảm biến nằm trên mặt phẳng z = 0, lưới 4x4 bước --pitch mm, gốc tọa độ ở tâm mảng.
  - cảm biến i ở hàng r = i // 4, cột c = i % 4  ->  x = (c - 1.5) * pitch,  y = (r - 1.5) * pitch.
  - trục X, Y, Z của mọi cảm biến song song với trục x, y, z của mảng; nam châm nằm ở z > 0.

Cách dùng:
  python magnet_lm.py --port COM4 --baud 115200 --pitch 10 [--csv out.csv]
  python magnet_lm.py --demo 200      # dữ liệu giả lập, không cần phần cứng
  python magnet_lm.py --demo 200 --offset 0.35   # giả lập offset cảm biến chưa calib (±0.35 mT mỗi trục)
  python magnet_lm.py --selftest      # kiểm tra Jacobian, LM và bộ phân tích dòng
Cần: numpy;  pyserial (chỉ khi đọc cổng thật):  pip install numpy pyserial
"""
import argparse
import math
import sys
import time

import numpy as np

N_SENS = 16
DIRS = np.array([[0, 0, 1], [0, 0, -1], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0]], dtype=float)


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
# 2) Hình học và mô hình lưỡng cực
# --------------------------------------------------------------------------------------
def sensor_positions(pitch_mm, flip_x=False, flip_y=False):
    pos = np.zeros((N_SENS, 3))
    for i in range(N_SENS):
        r, c = divmod(i, 4)
        x, y = (c - 1.5) * pitch_mm, (r - 1.5) * pitch_mm
        pos[i] = (-x if flip_x else x, -y if flip_y else y, 0.0)
    return pos


def dipole_field(p, pos):
    """B (mT) tại các vị trí pos (N,3), từ lưỡng cực p = [x,y,z,mx,my,mz]."""
    r = pos - p[:3]
    d2 = np.maximum(np.einsum('ij,ij->i', r, r), 1e-6)
    m = p[3:]
    mr = r @ m
    return (3.0 * mr[:, None] * r / d2[:, None] - m) / (d2 * np.sqrt(d2))[:, None]


def dipole_jacobian(p, pos):
    """dB/dp, dạng (N*3, 6), thứ tự hàng khớp với B.ravel()."""
    r = pos - p[:3]
    d2 = np.maximum(np.einsum('ij,ij->i', r, r), 1e-6)
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


YAW_VALID_RATIO = 0.05      # hypot(mx,my)/|m| nhỏ hơn mức này (pitch > ~87 độ) -> yaw không xác định


def moment_angles(m):
    """(pitch, yaw, tilt, yaw_hợp_lệ) của vector moment m. Góc tính bằng độ."""
    mx, my, mz = (float(v) for v in m)
    rho = math.hypot(mx, my)
    n = math.hypot(rho, mz)
    pitch = math.degrees(math.atan2(mz, rho))
    yaw = math.degrees(math.atan2(my, mx))
    return pitch, yaw, 90.0 - pitch, bool(n > 0.0 and rho / n >= YAW_VALID_RATIO)


def angle_sigmas(m, cov_m):
    """(sigma_pitch, sigma_yaw) bằng độ, lan truyền từ hiệp phương sai 3x3 của m."""
    mx, my, mz = (float(v) for v in m)
    rho2 = mx * mx + my * my
    n2 = rho2 + mz * mz
    if rho2 <= 0.0 or n2 <= 0.0:
        return float('nan'), float('nan')
    rho = math.sqrt(rho2)
    g_yaw = np.array([-my / rho2, mx / rho2, 0.0])
    g_pit = np.array([-mz * mx / (rho * n2), -mz * my / (rho * n2), rho / n2])

    def sig(g):
        return math.degrees(math.sqrt(max(float(g @ cov_m @ g), 0.0)))
    return sig(g_pit), sig(g_yaw)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


# --------------------------------------------------------------------------------------
# 3) Levenberg-Marquardt tổng quát
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
# 4) Bộ định vị nam châm
# --------------------------------------------------------------------------------------
class MagnetTracker:
    def __init__(self, pos, z_min=1.0, min_field=1.0, rms_ok=0.15):
        self.pos, self.z_min = pos, z_min
        self.min_field = min_field          # mT: |B| lớn nhất nhỏ hơn mức này -> coi như không có nam châm
                                    # (chưa calib: offset cảm biến tới ±0.35 mT theo datasheet, nên để > 0.35)
        self.rms_ok = rms_ok                # mT: rms vượt mức này thì thử lại nhiều điểm xuất phát
        self.p_prev = None
        self.K = 10.0                       # trọng số phạt khi z < z_min

    def _functions(self, B, valid):
        pos_v, B_v = self.pos[valid], B[valid]
        zmin, K = self.z_min, self.K

        def fun(p):
            pen = K * (zmin - p[2]) if p[2] < zmin else 0.0
            return np.append((dipole_field(p, pos_v) - B_v).ravel(), pen)

        def jac(p):
            J = dipole_jacobian(p, pos_v)
            row = np.zeros((1, 6))
            if p[2] < zmin:
                row[0, 2] = -K
            return np.vstack([J, row])

        return fun, jac

    def _starts(self, B):
        mag = np.linalg.norm(B, axis=1)
        w = mag ** 2 + 1e-12
        cx, cy = (w[:, None] * self.pos[:, :2]).sum(0) / w.sum()
        bmax = mag.max()
        out = []
        for z0 in (6.0, 12.0, 24.0):
            m0 = bmax * z0 ** 3 / 2.0                # lưỡng cực nằm trên trục: B = 2m / z^3
            for d in DIRS:
                out.append(np.array([cx, cy, z0, *(m0 * d)]))
        return out

    def fit(self, B, valid=None):
        """B: (16,3) mT. valid: mặt nạ cảm biến dùng được. Trả về dict kết quả hoặc None nếu không có nam châm."""
        valid = np.ones(N_SENS, bool) if valid is None else np.asarray(valid, bool)
        if valid.sum() < 4:
            return None
        if np.linalg.norm(B[valid], axis=1).max() < self.min_field:
            self.p_prev = None
            return None
        fun, jac = self._functions(B, valid)
        best = None
        if self.p_prev is not None:                  # theo dấu: bắt đầu từ nghiệm trước
            best = levenberg_marquardt(fun, self.p_prev, jac)
        if best is None or self._rms(best, valid) > self.rms_ok:
            for p0 in self._starts(B):               # khung đầu hoặc mất dấu: thử nhiều điểm xuất phát
                res = levenberg_marquardt(fun, p0, jac)
                if best is None or res['cost'] < best['cost']:
                    best = res
        self.p_prev = best['p'].copy()
        return self._summary(best, valid)

    @staticmethod
    def _rms(res, valid):
        return float(np.sqrt(np.mean(res['r'][:-1] ** 2)))

    def _summary(self, res, valid):
        p = res['p']
        n_res = 3 * int(valid.sum())
        dof = max(n_res - 6, 1)
        s2 = 2.0 * res['cost'] / dof
        try:
            cov = np.linalg.pinv(res['J'].T @ res['J']) * s2
        except np.linalg.LinAlgError:
            cov = np.full((6, 6), np.nan)
        sig = np.sqrt(np.maximum(np.diag(cov), 0.0))
        pitch, yaw, tilt, yaw_ok = moment_angles(p[3:])
        s_pitch, s_yaw = angle_sigmas(p[3:], cov[3:, 3:])
        return dict(p=p, pos=p[:3], m=p[3:], m_abs=float(np.linalg.norm(p[3:])),
                    pitch=pitch, yaw=yaw if yaw_ok else float('nan'), tilt=tilt, yaw_ok=yaw_ok,
                    sigma_pitch=s_pitch, sigma_yaw=s_yaw if yaw_ok else float('nan'),
                    rms=self._rms(res, valid), sigma_pos=sig[:3], iterations=res['iterations'],
                    nfev=res['nfev'], status=res['status'])


# --------------------------------------------------------------------------------------
# 5) Nguồn dữ liệu: cổng nối tiếp thật hoặc giả lập
# --------------------------------------------------------------------------------------
def encode_line(frame, B_mT, err=0, T_C=None, cal=0xFFFF, stale=0):
    """Tạo dòng UART đúng định dạng firmware (làm tròn về uT nguyên như firmware). T_C=None: 51 trường."""
    ut = np.rint(np.asarray(B_mT) * 1000.0).astype(int)
    f = ['B', str(frame), '%04X' % err]
    f += [str(v) for v in ut[:, 0]] + [str(v) for v in ut[:, 1]] + [str(v) for v in ut[:, 2]]
    if T_C is not None:
        f += [str(int(round(t * 10))) for t in T_C] + ['%04X' % cal, '%04X' % stale]
    return ','.join(f)


def demo_lines(n, pos, noise_mT, seed=1, offset_mT=0.0):
    """Nam châm di chuyển theo đường tròn, z và hướng moment thay đổi; trả về (dòng, chân trị p)."""
    rng = np.random.default_rng(seed)
    offsets = rng.uniform(-offset_mT, offset_mT, (N_SENS, 3))        # offset cố định của từng cảm biến (chưa calib)
    for k in range(n):
        t = 2.0 * np.pi * k / max(n, 1)
        pitch, yaw = np.radians(50.0 + 30.0 * np.sin(t)), 3.0 * t - np.pi          # pitch 20..80 độ, yaw quay 3 vòng
        p = np.array([6.0 * np.cos(t), 6.0 * np.sin(t), 11.0 + 3.0 * np.sin(2 * t),
                      15000 * np.cos(pitch) * np.cos(yaw), 15000 * np.cos(pitch) * np.sin(yaw), 15000 * np.sin(pitch)])
        B = dipole_field(p, pos) + offsets + rng.normal(0.0, noise_mT, (N_SENS, 3))
        yield encode_line(k, B), p


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
# 6) Vòng chính
# --------------------------------------------------------------------------------------
def run(source, tracker, csv_path=None, quiet=False):
    csv = open(csv_path, 'w') if csv_path else None
    if csv:
        csv.write('frame,x_mm,y_mm,z_mm,mx,my,mz,rms_mT,iters,pitch_deg,yaw_deg,tilt_deg\n')
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
            x, y, z = res['pos']
            if truth is not None:
                tp, ty, _, ty_ok = moment_angles(truth[3:])
                errs.append((np.linalg.norm(res['pos'] - truth[:3]), res['rms'], abs(res['pitch'] - tp),
                             abs(wrap180(res['yaw'] - ty)) if (ty_ok and res['yaw_ok']) else float('nan')))
            if not quiet:
                yaw_txt = '%7.1f' % res['yaw'] if res['yaw_ok'] else '    n/a'
                sig_txt = ('±%.1f ±%.1f' % (res['sigma_pitch'], res['sigma_yaw']) if res['yaw_ok']
                           else '±%.1f ±n/a' % res['sigma_pitch'])
                print('frame %6d: x=%7.2f y=%7.2f z=%6.2f mm | pitch=%6.1f yaw=%s deg (%s) | |m|=%6.0f | '
                      'rms=%5.1f uT | ±%.2f mm | %2d it %5.1f ms %s'
                      % (d['frame'], x, y, z, res['pitch'], yaw_txt, sig_txt,
                         res['m_abs'], res['rms'] * 1e3, res['sigma_pos'].max(), res['iterations'], dt, res['status']))
            if csv:
                csv.write('%d,%.4f,%.4f,%.4f,%.1f,%.1f,%.1f,%.5f,%d,%.3f,%.3f,%.3f\n'
                          % (d['frame'], x, y, z, *res['m'], res['rms'], res['iterations'],
                             res['pitch'], res['yaw'], res['tilt']))
    except KeyboardInterrupt:
        pass
    finally:
        if csv:
            csv.close()
    return errs


# --------------------------------------------------------------------------------------
# 7) Tự kiểm tra
# --------------------------------------------------------------------------------------
def selftest():
    ok = True
    rng = np.random.default_rng(0)
    pos = sensor_positions(10.0)

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

    # b) Jacobian giải tích so với sai phân số
    p = np.array([2.0, -3.0, 9.0, 4000.0, -2500.0, 9000.0])
    Ja = dipole_jacobian(p, pos)
    Jn = numeric_jacobian(lambda q: dipole_field(q, pos).ravel(), p)
    err = np.max(np.abs(Ja - Jn) / (np.abs(Jn) + 1e-9))
    print('Jacobian giải tích vs số: sai lệch tương đối lớn nhất %.2e  %s' % (err, 'đạt' if err < 1e-5 else 'LỖI'))
    ok &= err < 1e-5

    # c) LM trên hàm Rosenbrock (kiểm tra riêng bộ LM)
    def rosen(q):
        return np.array([10.0 * (q[1] - q[0] ** 2), 1.0 - q[0]])
    res = levenberg_marquardt(rosen, [-1.2, 1.0])
    good = np.allclose(res['p'], [1.0, 1.0], atol=1e-6)
    print('LM trên Rosenbrock: p = %s sau %d vòng  %s' % (np.round(res['p'], 6), res['iterations'], 'đạt' if good else 'LỖI'))
    ok &= good

    # d) định vị nam châm: 200 tư thế ngẫu nhiên, nhiễu 0.03 mT, bắt đầu từ trạng thái mất dấu
    n, bad, pe, ap, ay = 200, 0, [], [], []
    for _ in range(n):
        pt = np.array([*rng.uniform(-9, 9, 2), rng.uniform(6, 20), *(14000 * rng.normal(size=3) / 1.7)])
        B = dipole_field(pt, pos) + rng.normal(0, 0.03, (N_SENS, 3))
        B = np.rint(B * 1000) / 1000
        tr = MagnetTracker(pos, min_field=0.1)
        r = tr.fit(B)
        e = np.linalg.norm(r['pos'] - pt[:3]) if r else np.inf
        pe.append(e)
        bad += e > 1.0
        tp, ty, _, ty_ok = moment_angles(pt[3:])
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

    # d2) quy ước góc (pitch = góc ngẩng so với mặt phẳng xy, yaw = phương vị từ +x sang +y)
    cases = [((1, 0, 0), 0, 0, True), ((0, 1, 0), 0, 90, True), ((-1, 0, 0), 0, 180, True), ((0, -1, 0), 0, -90, True),
             ((1, 0, 1), 45, 0, True), ((0, 0, 1), 90, None, False), ((0, 0, -1), -90, None, False),
             ((-1, -1, -np.sqrt(2)), -45, -135, True)]
    good = True
    for m, ep, ey, eok in cases:
        pch, yw, tilt, vok = moment_angles(m)
        good &= abs(pch - ep) < 1e-9 and vok == eok and (ey is None or abs(wrap180(yw - ey)) < 1e-9) and abs(tilt - (90 - ep)) < 1e-9
    print('Quy ước pitch/yaw/tilt trên 8 trường hợp đặc biệt: %s' % ('đạt' if good else 'LỖI'))
    ok &= good

    # d3) độ lệch chuẩn góc ước lượng có khớp sai số thực không (Monte Carlo 300 lần, cùng một tư thế)
    pt = np.array([2.0, -3.0, 10.0, *(15000 * np.array([np.cos(np.radians(40)) * np.cos(np.radians(60)),
                                                         np.cos(np.radians(40)) * np.sin(np.radians(60)),
                                                         np.sin(np.radians(40))]))])
    clean = dipole_field(pt, pos)
    pit, yw, sp, sy = [], [], [], []
    for _ in range(300):
        r = MagnetTracker(pos, min_field=0.1).fit(clean + rng.normal(0, 0.05, (N_SENS, 3)))
        pit.append(r['pitch']); yw.append(r['yaw']); sp.append(r['sigma_pitch']); sy.append(r['sigma_yaw'])
    emp_p, emp_y = np.std(pit), np.std(yw)
    pred_p, pred_y = np.mean(sp), np.mean(sy)
    good = abs(emp_p / pred_p - 1) < 0.25 and abs(emp_y / pred_y - 1) < 0.25
    print('σ góc (nhiễu 0.05 mT, pitch 40, yaw 60): đo %.3f / %.3f độ, ước lượng %.3f / %.3f độ (pitch / yaw)  %s'
          % (emp_p, emp_y, pred_p, pred_y, 'đạt' if good else 'LỖI'))
    ok &= good

    # e) cảm biến lỗi: bỏ 3 cảm biến, kết quả vẫn đúng
    pt = np.array([3.0, -2.0, 10.0, 3000.0, 1000.0, 12000.0])
    B = dipole_field(pt, pos)
    valid = np.ones(N_SENS, bool)
    valid[[2, 9, 14]] = False
    B[~valid] = 999.0
    r = MagnetTracker(pos, min_field=0.1).fit(B, valid)
    e = np.linalg.norm(r['pos'] - pt[:3])
    print('Bỏ 3 cảm biến lỗi (giá trị rác 999 mT): sai số vị trí %.4f mm  %s' % (e, 'đạt' if e < 0.01 else 'LỖI'))
    ok &= e < 0.01
    print('KẾT QUẢ:', 'ĐẠT' if ok else 'CÓ LỖI')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description='Định vị nam châm trên mảng 4x4 TMAG5170 bằng Levenberg-Marquardt')
    ap.add_argument('--port', help='cổng nối tiếp, ví dụ COM5 hoặc /dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--pitch', type=float, default=10.0, help='khoảng cách giữa các cảm biến (mm)  [PHẢI chỉnh cho đúng]')
    ap.add_argument('--flip-x', action='store_true')
    ap.add_argument('--flip-y', action='store_true')
    ap.add_argument('--zmin', type=float, default=1.0, help='độ cao tối thiểu của nam châm (mm)')
    ap.add_argument('--min-field', type=float, default=1.0,
                    help='|B| lớn nhất dưới mức này (mT) = không có nam châm. Giữ > 0.35 vì offset cảm biến chưa calib tới ±0.35 mT')
    ap.add_argument('--csv', help='ghi kết quả ra file CSV')
    ap.add_argument('--demo', type=int, metavar='N', help='chạy N khung dữ liệu giả lập')
    ap.add_argument('--noise', type=float, default=0.03, help='nhiễu giả lập (mT)')
    ap.add_argument('--offset', type=float, default=0.0, help='giả lập offset ngẫu nhiên mỗi cảm biến/trục, ± giá trị này (mT)')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()

    if a.selftest:
        sys.exit(selftest())
    pos = sensor_positions(a.pitch, a.flip_x, a.flip_y)
    tracker = MagnetTracker(pos, z_min=a.zmin, min_field=a.min_field)
    if a.demo:
        errs = run(demo_lines(a.demo, pos, a.noise, offset_mT=a.offset), tracker, a.csv, quiet=False)
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
