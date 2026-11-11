/*
 * TMAG5170.h
 *
 *  Created on: Oct 27, 2026
 *      Author: Admin
 */

#ifndef INC_TMAG5170_H_
#define INC_TMAG5170_H_

/*
 * TMAG5170 (TI) - driver SPI cho STM32 HAL
 * Cấu hình: Active mode, 3 kênh X/Y/Z, bù nhiệt nam châm NdFeB (MAG_TEMPCO = 01b)
 * SPI: Mode 0 (CPOL=0, CPHA=0), MSB first, 8-bit, tối đa 10 MHz, CS điều khiển bằng phần mềm
 */

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*  Địa chỉ thanh ghi */
#define DEVICE_CONFIG  0x00
#define SENSOR_CONFIG  0x01
#define SYSTEM_CONFIG  0x02
#define CONV_STATUS    0x08
#define X_CH_RESULT    0x09
#define Y_CH_RESULT    0x0A
#define Z_CH_RESULT    0x0B
#define AFE_STATUS     0x0D
#define SYS_STATUS     0x0E
#define TEST_CONFIG    0x0F

/* DEVICE_CONFIG (0x00) */
#define CONV_AVG(n)        ((uint16_t)((n) & 7u) << 12)  /* 0=1x ... 5=32x */
#define TEMPCO_NDFEB       (1u << 8)                     /* MAG_TEMPCO = 01b: 0.12 %/°C */
#define MODE_ACTIVE_CONT   (2u << 4)                     /* OPERATING_MODE = 010b */
#define MODE_ACTIVE_TRIG   (3u << 4)                     /* OPERATING_MODE = 011b */

/* ---------- SENSOR_CONFIG (0x01) ---------- */
#define MAG_CH_XYZ         (7u << 6)                     /* MAG_CH_EN = 0111b */
#define RANGE_MAX_XYZ      ((2u << 4) | (2u << 2) | 2u)  /* Z/Y/X_RANGE = 10b: ±100 mT (A1) / ±300 mT (A2) */

/* ---------- Thời gian chuyển đổi: CONV_AVG = 32x, 3 trục ---------- */
/* 825 us (trục đầu) + 2 x 800 us + 10 us khởi tạo ≈ 2.44 ms  -> chờ 3 ms */
#define CONV_WAIT_MS       3

typedef enum {
    TMAG_OK = 0,
    TMAG_ERR_SPI,
    TMAG_ERR_CRC,
    TMAG_ERR_VERSION,
    TMAG_ERR_VERIFY,
    TMAG_ERR_UNSTABLE
} err_t;

typedef enum {
    CONTINUOUS = 0,    /* Active measure mode (OPERATING_MODE = 010b) */
    TRIGGER    = 1     /* Active trigger mode (OPERATING_MODE = 011b) */
} tmag_mode_t;         /* không đặt tên mode_t: trùng với mode_t của newlib (<sys/types.h>) */

/* ±range của dải đo cao nhất (mT): TMAG5170A1 = 100, TMAG5170A2 = 300 */
#define TMAG_RANGE_MT      100.0f

typedef void (*cs_fn)(bool select);   /* select=true -> kéo CS xuống thấp */

typedef struct {
    SPI_HandleTypeDef *hspi;
    cs_fn         cs;
    tmag_mode_t   mode;
    float              range_mT;     /* ±range của dải cao nhất: 100 (A1) hoặc 300 (A2) */
    bool               error_stat;   /* bit ERROR_STAT của lần đọc gần nhất */
    uint32_t           last_tx;      /* debug: frame SDI vừa gửi */
    uint32_t           last_rx;      /* debug: frame SDO vừa nhận */
} TMAG5170_t;

typedef struct {
    int16_t raw[3];                  /* X, Y, Z - mã ADC 16 bit có dấu */
    float   mT[3];                   /* X, Y, Z - từ trường (mT) */
} xyz_t;

/* Cấu hình SENSOR_CONFIG: X,Y,Z + dải cao nhất */
#define TMAG_SENSOR_CFG    (MAG_CH_XYZ | RANGE_MAX_XYZ)  /* 0x01EA */

/* --- Dùng cho 1 cảm biến, chặn (blocking): mỗi thanh ghi 1 khung SPI, giữa các khung CS lên cao --- */
err_t TMAG5170_Init(TMAG5170_t *d, SPI_HandleTypeDef *hspi, cs_fn cs, tmag_mode_t mode);
err_t TMAG5170_read_XYZ(TMAG5170_t *d, xyz_t *out);

/* --- Dùng cho quét theo vòng decoder: mỗi hàm dưới đây gửi ĐÚNG MỘT khung SPI --- */
#define TMAG5170_INIT_STEPS  7                       /* số bước của TMAG5170_InitStep (0..6) */
void  TMAG5170_InitBegin(TMAG5170_t *d, SPI_HandleTypeDef *hspi, cs_fn cs, tmag_mode_t mode);   /* không gửi SPI */
err_t TMAG5170_InitStep(TMAG5170_t *d, uint8_t step);                 /* 1 khung SPI / bước */
err_t TMAG5170_Trigger(TMAG5170_t *d);                                /* bắt đầu 1 bộ chuyển đổi X,Y,Z (chế độ TRIGGER) */
err_t TMAG5170_ReadAxis(TMAG5170_t *d, uint8_t axis, int16_t *code);  /* axis: 0=X 1=Y 2=Z */
float TMAG5170_CodeToMT(const TMAG5170_t *d, int16_t code);


#endif /* INC_TMAG5170_H_ */
