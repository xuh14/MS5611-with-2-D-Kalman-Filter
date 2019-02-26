/*
 * MS5611_Kalman.h
 * A library for barometer MS5611 with 2-d Kalman filter
 *
 * Copyright (c) 2019 <xuh14@miamioh.edu>
 */
#ifndef __MS5611_KAL__H
#define __MS5611_KAL__H

#include <Arduino.h>
#include <Math.h>
#include <Wire.h>

#define BARO_ADDRESS 0x77
#define RAW_PRESS_COMMAND 0x48
#define RAW_TEMP_COMMAND 0x58

const int EXTRA_PRECISION = 5;
const int CIRAVG_SIZE = 32; 


class MS5611_KAL
{
    public:
    MS5611_KAL(bool USER_FIL_OPTION);
    float Kalman_filter(float measure_acc, float mea_velocity);
    void MS5611_setup(bool StartZero, float USER_Q, float USER_R, float USER_DT);
    float getMS5611data(float USER_ACC, float USER_ALTITUDE);
    float get_RAW_Altitude();
    float getPressure();
    float getTemperature();
    float get_RAW_Pressure();
    float get_RAW_Temperature();
    void update();

    private:
    int8_t command;
    uint16_t baro_calibration[6];
    float baro_conversion_timer, current_altitude, average_altitude;
    int32_t baro_raw_pressure, baro_raw_temp;
    float baro_pressure,baro_temp,baro_altitude;
    bool pressure_flag, temp_flag, data_ready_flag, FIL_OPTION, USER_FIL_OPTION;
    //filtering parameters    
    int ciravg_pt = 32;
    float ciravg_arr[CIRAVG_SIZE];
    float xkhat_, vkhat_, xkhat, vkhat;
    float dt, Pk, Pk_, Kk, R, Q;
    float USER_ACC, USER_VEL;
    float average = 0;
    bool StartZero;
    
    void MS5611_doconversion(uint8_t command);
    void MS5611_conversionResult();
    void MS5611_Kalman_init(bool StartZero, float USER_Q, float USER_R, float USER_DT);
};

#endif
