#ifndef ISHAANFILTER_H
#define ISHAANFILTER_H

#pragma once

#include <Arduino.h>
#include <CircularBuffer.hpp>   // https://github.com/rlogiacco/CircularBuffer
#include "mpu6500.h"            // BFS Mpu‑6500 driver (defines bfs::Mpu6500)

// ──────────────────────────────────────────────────────────────────────────────
//  ishaanfilter
//  Simple acceleration → velocity integrator with running‑mean drift correction
// ──────────────────────────────────────────────────────────────────────────────
class IshaanFilter
{
public:
    // Constructor – supply static calibration offsets
    IshaanFilter(float xCalp, float yCalp, float zCalp);

    // Reject small accelerations (dead‑band)
    float filterLowAcc(float d);

    // Integrate buffered acceleration and update xV/yV/zV
    void  updateV();

    void updateCalValues(float nxCal, float nyCal, float nzCal);

    bool IshaanFilter::isStable();

    bool IshaanFilter::isAccZero(float ax,float ay,float az);

    // Main entry point: push a new raw accel sample
    void  updateFilter(float ax, float ay, float az);
    

    ~IshaanFilter();

    // ───── Public members (read on demand) ─────
    CircularBuffer<float, 48> xBuffer;
    CircularBuffer<float, 48> yBuffer;
    CircularBuffer<float, 48> zBuffer;

    float xCal{}, yCal{}, zCal{};   // static accel offsets
    float xV{},   yV{},   zV{};     // current velocity estimates
    float xBufferMean{}, yBufferMean{}, zBufferMean{};

    bfs::Mpu6500 imu;               // underlying IMU object

private:
    // (no private helpers beyond the public API for now)
};

#endif  // ISHAANFILTER_H
