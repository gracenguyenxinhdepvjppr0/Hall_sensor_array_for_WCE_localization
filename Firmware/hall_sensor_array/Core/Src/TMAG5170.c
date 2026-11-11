#include "TMAG5170.h"

/* CRC-4: x^4 + x + 1, khởi tạo 1111b (datasheet 7.5.2.5) */
static uint8_t crc4(uint32_t frame)
{
    uint32_t p   = frame & 0xFFFFFFF0u;   /* 4 bit CRC coi như = 0 */
    uint8_t  crc = 0xF;

    for (int i = 31; i >= 0; i--) {
        uint8_t inv = (uint8_t)(((p >> i) & 1u) ^ ((crc >> 3) & 1u));
        crc = (uint8_t)(((crc << 1) & 0xEu) ^ (inv ? 0x3u : 0x0u));
    }
    return crc;
}

/* Khung SDI 32 bit: [31]=R/W(1=đọc) [30:24]=địa chỉ [23:8]=dữ liệu [7:4]=CMD3..0 [3:0]=CRC */
static uint32_t make_frame(bool rd, uint8_t addr, uint16_t data, uint8_t cmd)
{
    uint32_t f = ((uint32_t)rd << 31) | ((uint32_t)(addr & 0x7Fu) << 24) |
                 ((uint32_t)data << 8) | ((uint32_t)(cmd & 0xFu) << 4);
    return f | crc4(f);
}

static err_t xfer(TMAG5170_t *d, uint32_t tx, uint32_t *rx)
{
    uint8_t t[4] = { (uint8_t)(tx >> 24), (uint8_t)(tx >> 16), (uint8_t)(tx >> 8), (uint8_t)tx };
    uint8_t r[4] = { 0 };

    d->last_tx = tx;
    d->last_rx = 0;

    d->cs(true);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(d->hspi, t, r, 4, 10);
    d->cs(false);
    if (st != HAL_OK) return TMAG_ERR_SPI;

    uint32_t v = ((uint32_t)r[0] << 24) | ((uint32_t)r[1] << 16) | ((uint32_t)r[2] << 8) | r[3];
    d->last_rx = v;
    if (crc4(v) != (v & 0xFu)) return TMAG_ERR_CRC;

    d->error_stat = (v >> 7) & 1u;        /* STAT3 = ERROR_STAT */
    if (rx) *rx = v;
    return TMAG_OK;
}

static err_t reg_write(TMAG5170_t *d, uint8_t addr, uint16_t val)
{
    return xfer(d, make_frame(false, addr, val, 0), NULL);
}

/* Đọc dữ liệu thanh ghi nằm ngay trong bit [23:8] của chính khung đó */
static err_t reg_read_cmd(TMAG5170_t *d, uint8_t addr, uint8_t cmd, uint16_t *val)
{
    uint32_t rx = 0;
    err_t e = xfer(d, make_frame(true, addr, 0, cmd), &rx);
    if (e == TMAG_OK && val) *val = (uint16_t)(rx >> 8);
    return e;
}

static err_t reg_read(TMAG5170_t *d, uint8_t addr, uint16_t *val)
{
    return reg_read_cmd(d, addr, 0, val);
}

/* ================= Khởi tạo ================= */
static uint16_t dev_cfg_of(tmag_mode_t mode)
{
    /* CONV_AVG=32x, bù nhiệt NdFeB, vào Active mode */
    return (uint16_t)(CONV_AVG(5) | TEMPCO_NDFEB |
                      (mode == TRIGGER ? MODE_ACTIVE_TRIG : MODE_ACTIVE_CONT));
}

void TMAG5170_InitBegin(TMAG5170_t *d, SPI_HandleTypeDef *hspi, cs_fn cs, tmag_mode_t mode)
{
    d->hspi       = hspi;
    d->cs         = cs;
    d->mode       = mode;
    d->range_mT   = TMAG_RANGE_MT;       /* trước đây không được gán -> mT luôn sai/bằng 0 */
    d->error_stat = false;
}

/* Mỗi bước = đúng 1 khung SPI, để có thể chạy xen kẽ qua nhiều cảm biến dùng chung decoder */
err_t TMAG5170_InitStep(TMAG5170_t *d, uint8_t step)
{
    uint16_t v;
    err_t e;

    switch (step) {
    case 0: return reg_read(d, AFE_STATUS, &v);   /* đọc AFE_STATUS / SYS_STATUS để xóa cờ CFG_RESET */
    case 1: return reg_read(d, SYS_STATUS, &v);   /* và lỗi tồn đọng (kiểu RC) */
    case 2: return reg_write(d, SYSTEM_CONFIG, 0x0000);        /* DATA_TYPE=0, TRIGGER_MODE=0 (lệnh SPI), tắt diag */
    case 3: return reg_write(d, SENSOR_CONFIG, TMAG_SENSOR_CFG);
    case 4: return reg_write(d, DEVICE_CONFIG, dev_cfg_of(d->mode));   /* ghi SAU CÙNG */
    case 5:
        HAL_Delay(1);                                                   /* > tstand_by (90 us) */
        if ((e = reg_read(d, DEVICE_CONFIG, &v)) != TMAG_OK) return e;
        return (v == dev_cfg_of(d->mode)) ? TMAG_OK : TMAG_ERR_VERIFY;
    case 6:
        if ((e = reg_read(d, SENSOR_CONFIG, &v)) != TMAG_OK) return e;
        return (v == TMAG_SENSOR_CFG) ? TMAG_OK : TMAG_ERR_VERIFY;
    default:
        return TMAG_OK;
    }
}

/* Khởi tạo 1 cảm biến, chặn. Với nhiều cảm biến dùng chung decoder vòng Gray thì dùng InitStep theo vòng */
err_t TMAG5170_Init(TMAG5170_t *d, SPI_HandleTypeDef *hspi, cs_fn cs, tmag_mode_t mode)
{
    err_t e;

    TMAG5170_InitBegin(d, hspi, cs, mode);
    d->cs(false);
    HAL_Delay(2);   /* > tstart_power_up (tối đa 500 us ở VCC = 2.3 V) */

    for (uint8_t s = 0; s < TMAG5170_INIT_STEPS; s++)
        if ((e = TMAG5170_InitStep(d, s)) != TMAG_OK) return e;

    if (mode == CONTINUOUS) HAL_Delay(CONV_WAIT_MS);   /* chờ bộ kết quả đầu tiên */
    return TMAG_OK;
}

/* ================= Các khung đơn lẻ ================= */
err_t TMAG5170_Trigger(TMAG5170_t *d)
{
    /* CMD0 = 1: bắt đầu một bộ chuyển đổi X,Y,Z khi CS lên cao */
    return reg_read_cmd(d, CONV_STATUS, 0x1, NULL);
}

err_t TMAG5170_ReadAxis(TMAG5170_t *d, uint8_t axis, int16_t *code)
{
    static const uint8_t axis_reg[3] = { X_CH_RESULT, Y_CH_RESULT, Z_CH_RESULT };
    uint16_t v;
    err_t e;

    if (axis > 2u) return TMAG_ERR_VERIFY;
    if ((e = reg_read(d, axis_reg[axis], &v)) != TMAG_OK) return e;
    *code = (int16_t)v;               /* chỉ ghi khi đọc thành công */
    return TMAG_OK;
}

/* Công thức (1) datasheet: B = code / 2^16 * 2*|BR| = code * BR / 32768 */
float TMAG5170_CodeToMT(const TMAG5170_t *d, int16_t code)
{
    return (float)code * d->range_mT / 32768.0f;
}

/* ================= Đọc X, Y, Z ================= */
err_t TMAG5170_read_XYZ(TMAG5170_t *d, xyz_t *out)
{
    static const uint8_t axis_reg[3] = { X_CH_RESULT, Y_CH_RESULT, Z_CH_RESULT };
    uint16_t raw[3];
    err_t e;

    if (d->mode == TRIGGER) {
        /* CMD0 = 1: bắt đầu một bộ chuyển đổi X,Y,Z khi CS lên cao */
        if ((e = reg_read_cmd(d, CONV_STATUS, 0x1, NULL)) != TMAG_OK) return e;
        HAL_Delay(CONV_WAIT_MS);
        for (int i = 0; i < 3; i++)
            if ((e = reg_read(d, axis_reg[i], &raw[i])) != TMAG_OK) return e;
    } else {
        /* Continuous: 3 lần đọc không đồng thời -> kiểm tra SET_COUNT (CONV_STATUS[6:4]) không đổi */
        bool ok = false;
        for (int retry = 0; retry < 3 && !ok; retry++) {
            uint16_t s0, s1;
            if ((e = reg_read(d, CONV_STATUS, &s0)) != TMAG_OK) return e;
            for (int i = 0; i < 3; i++)
                if ((e = reg_read(d, axis_reg[i], &raw[i])) != TMAG_OK) return e;
            if ((e = reg_read(d, CONV_STATUS, &s1)) != TMAG_OK) return e;
            ok = (((s0 >> 4) & 7u) == ((s1 >> 4) & 7u));
        }
        if (!ok) return TMAG_ERR_UNSTABLE;
    }

    /* Công thức (1) datasheet: B = code / 2^16 * 2*|BR| = code * BR / 32768 */
    for (int i = 0; i < 3; i++) {
        out->raw[i] = (int16_t)raw[i];
        out->mT[i]  = TMAG5170_CodeToMT(d, out->raw[i]);
    }
    return TMAG_OK;
}
