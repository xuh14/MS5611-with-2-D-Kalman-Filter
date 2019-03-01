# MS5611-with-2-D-Kalman-Filter


 * README
 * 
 * EDIT: Feb. 23, 2019
 * 
 * Copyright (c) 2019 <xuh14@miamioh.edu>
 * 
 * A library for barometer MS5611 with 2-d Kalman filter
 * This library will implement I2C communication to get the barometer MS5611 data and process with filtering to get smoother result
 * The library is served as a basis of a sensor fusion system for measuring altitude. Due to the fact that barometer is very sensitive 
 * to temperature, sensors such as accelorometer or GPS can be used to correct barometer data due to special conditions such as fast 
 * warm wind. Here is a basic implementation used with accelerometer.
 *
 *
 * USER_FIL_OPTION = 0 : circular array averaging;
 * USER_FIL_OPTION = 1 : 2-D Kalman filtering
 *
 * Kalman filtering required USER_ENTERED acceleration and velocity for calculations
 * Kalman result is simulated from a zero mean gaussian distribution noise with standard deviation 0.1, Q is set at 0.01. 
 * 
 * steps:
 * 1. MS5611_KAL baro = MS5611_KAL(USER_FIL_OPTION)
 * 2. MS5611_setup(bool USER_STARTZERO = false, float USER_Q = 0, float USER_R = 0, float USER_DT = 0)
 *    Enter values if chosen Kalman Filtering, if reletive altitude is preferred, set USER_STARTZERO = true
 * 3. baro.update();
 *    double baro_altitude = baro.get_RAW_Altitude();   // getting raw MS5611 data
 *    
 *    MS5611_KAL::getMS5611data(float USER_ACC = 0, float USER_VEL = 0)  // getting filtered data, enter value if chosen Kalman filtering
 *
 * 
 * Notion**:
 * MS5611 barometer will need at least 10ms to collect either temp or pressure data.
 * altitude is calculated based on both temp & pressure thus will need to collect both temp & pressure data first
 * In the process, the program switches between temp & pressure to reduce run time 
 * and use timers to contour 10ms deadzone for applications required fast loop
 *
 * MPU9250 accelerometer has been used for testing, though, data from double integral will tend to drift, thus it is replaced
 * with gaussian noise and can lead to smoother values.
 *
 * Latency Test:
 * CIRAVG_SIZE = 32 : 1.108ms
 * Kalman : 1.720ms
 *
 * Wiring:
 * VCC -> 3.3V
 * PS -> 3.3V    % I2C
 * SCL -> SCL
 * SDA -> SDA
 */
