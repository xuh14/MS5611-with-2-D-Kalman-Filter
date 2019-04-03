# MS5611-with-2-D-Kalman-Filter


 * README
 * 
 * EDIT: March 20, 2019
 * 
 * Report is linked in wiki page 
 * https://github.com/xuh14/MS5611-with-2-D-Kalman-Filter/wiki
 *
 * ------------------------------------------------Description & Report-----------------------------------------
 * Barometer-MS5611 is a chip that measures the atmoshphere pressure data that can achieve accuracy of 0.012 mbar.
 * Altitude can be calculated from barometric equation below(from wiki.)
 *
 * The specific application focus for this library is to generate a steady altitude data from I2C connection with 
 * data processing to help implement the altitude hold feature of a multiroter. 
 * 
 * From some pre-acknowledgement, altitude generated from MS5611 raw pressure data has a standard deviation of 0.1m while the
 * tested result has standard deviation of 0.2m. The data, as shown below, has the trend of a normal distribution around the 
 * mean value. Therefore, for steady control of a multiroter, some filters will be needed to process the raw data to prevent
 * the multiroter from jumping up and down. 
 *
 * Here, two filters are implemented. First is a circular array filter that uses the mean of the array as the output, which
 * the array size can be set by user. Kalman filter is also implemented to make use of the accelerometer that most multiroter
 * has onboard. The equations used for Kalman filter is shown below. 
 *
 * Here, acceloremter is intended to be used as an estimation of the current altitude from equation X = Vo*t+ 1/2*a*t^2. The 
 * system equation of the Kalman implementation is a 1-d equation that only contains acceleratio update but not velocity. 
 * Users will need to provide pre-calculated velocity into the function.
 * 
 * In real testing, this approach of fusing the accelerometer has failed. The reason is mainly due to signal noise in discrete
 * signal processing as the drift altitude or velocity will be cumulative and has drift of 1m every 3s. Instead, a P
 * controller is added into the update equation. By using last stage barometer's raw altitude data as the estimation and the 
 * current state as the measument data, a P controller can be used to adjust the smoothness of the data using the error, 
 * measurement - estimation, times the adjustable gain. In the program, 0.1 is used.
 *
 * ------------------------------------------------------Settings-------------------------------------------
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
