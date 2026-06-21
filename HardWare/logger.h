#ifndef LOGGER_H
#define LOGGER_H

#include "stm32f10x.h"

#define LOG_SECTOR_SIZE   512
#define LOG_ENTRY_SIZE    64       /* 16 fields × 4 bytes */

/* Compiler-portable packed attribute */
#if defined(__GNUC__) || defined(__clang__)
  #define PACKED_STRUCT __attribute__((packed))
#else
  #define PACKED_STRUCT
#endif

typedef struct PACKED_STRUCT {
    uint32_t magic;               /* 0x4C4F4745 ("LOGE") */
    uint32_t seq;                 /* Sequence number */
    uint32_t timestamp_ms;        /* System tick (ms) */
    float    roll;                /* Actual roll angle (°) */
    float    pitch;               /* Actual pitch angle (°) */
    float    yaw;                 /* Actual yaw angle (°) */
    float    gyro_x;              /* Gyro X (°/s) */
    float    gyro_y;              /* Gyro Y (°/s) */
    float    gyro_z;              /* Gyro Z (°/s) */
    float    m1;                  /* Motor 1 (μs) */
    float    m2;                  /* Motor 2 (μs) */
    float    m3;                  /* Motor 3 (μs) */
    float    m4;                  /* Motor 4 (μs) */
    float    target_roll;         /* Target roll (°)   — NEW */
    float    target_pitch;        /* Target pitch (°)  — NEW */
    float    target_yaw_rate;     /* Target yaw rate (°/s) — NEW */
} LogEntry_t;

uint8_t LOG_Init(void);
void LOG_WriteEntry(const LogEntry_t *entry);
void LOG_Task(void *pvParameters);

#endif
