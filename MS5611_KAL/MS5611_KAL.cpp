#include "MS5611_KAL.h"
#include <Math.h>

/**
 * MS5611_Kalman.cpp
 * 
 * EDIT: Feb. 23, 2019
 * 
 * Copyright (c) 2019 <xuh14@miamioh.edu>
 * 
 * A library for barometer MS5611 with 2-d Kalman filter
 * This program will implement I2C communication to get the barometer MS5611 data and process with filtering
 * USER_FIL_OPTION = 0 : circular array averaging;
 * USER_FIL_OPTION = 1 : 2-D Kalman filtering
 *
 * Kalman filtering required USER_ENTERED acceleration and velocity for calculations
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

MS5611_KAL::MS5611_KAL(bool USER_FIL_OPTION) {
  Serial.begin(115200);
  FIL_OPTION = USER_FIL_OPTION;
}

void MS5611_KAL::MS5611_Kalman_init(bool StartZero, float USER_Q, float USER_R, float USER_DT) {
  if (StartZero) { xkhat_ = 0; } 
  else { xkhat_ = baro_altitude; }
  Pk = 1;
  dt = USER_DT;
  Q = USER_Q;
  R = USER_R;
}

float MS5611_KAL::Kalman_filter(float measure_acc, float mea_velocity) {   
  float mea_altitude = baro_altitude; 
  // time update
  xkhat_ = xkhat_ + mea_velocity * dt + 0.5 * measure_acc * pow(dt, 2);
  Pk_ = Pk + Q;
  // measurement update
  Kk = Pk_ / (Pk_ + R);
  xkhat = xkhat_ + Kk * (mea_altitude - xkhat_);
  Pk = (1 - Kk) * Pk_;
  return xkhat;
}

void MS5611_KAL::MS5611_setup(bool USER_STARTZERO = false, float USER_Q = 0, float USER_R = 0, float USER_DT = 0) {
  Serial.begin(115200);
  StartZero = USER_STARTZERO;
  TWBR = 12;
  FIL_OPTION = USER_FIL_OPTION;
  Wire.begin();
  // reset the barometer
  Wire.beginTransmission(BARO_ADDRESS);
  Wire.write(0x1E); // reset address
  Wire.endTransmission();

  delay(1000);
  // read the factory calibration data and restore it
  for(int i = 0; i < 6;i++)
  {
    Wire.beginTransmission(BARO_ADDRESS);
    Wire.write(0xA2 + (i * 2));
    Wire.endTransmission(false);
    Wire.requestFrom(BARO_ADDRESS,2);
    while(Wire.available() < 2);
    baro_calibration[i] = Wire.read() << 8 | Wire.read();
  }
  // read the current data 
  data_ready_flag = 1; 

  //calibration
  int N = 1000;
  float sum = 0;
  for (int i = 0; i < N; i++) {
    MS5611_doconversion(RAW_PRESS_COMMAND);
    delay(10); // wait for the pressure data is ready
    MS5611_conversionResult(); 
    MS5611_doconversion(RAW_TEMP_COMMAND);
    delay(10); // wait for the temperture data is ready
    MS5611_conversionResult();
    get_RAW_Altitude();
    sum = sum + baro_altitude;
    delay(20);
  }
  average = sum / N;

  // fill the first CIRAVG with altitude data with SIZE 
  if (FIL_OPTION == 0) {
    for(int i = 0; i < CIRAVG_SIZE; i++)
      {
        MS5611_doconversion(RAW_PRESS_COMMAND);
        delay(10); // wait for the pressure data is ready
        MS5611_conversionResult(); 
        MS5611_doconversion(RAW_TEMP_COMMAND);
        delay(10); // wait for the temperture data is ready
        MS5611_conversionResult();
        get_RAW_Altitude();
        ciravg_arr[i] = baro_altitude;
      }
  } else {
    // KALMAN filtering
    get_RAW_Altitude();
    MS5611_Kalman_init(StartZero, USER_Q, USER_R, USER_DT);
  }
  Serial.println("Finished barometer setup.");
  Serial.println("Current pressure: " + String(baro_pressure ));
  Serial.println("Current temperature: " + String(baro_temp));
  Serial.println("Current Altitude: " + String(baro_altitude));
  Serial.println("Average Altitude: " + String(average));
}

void MS5611_KAL::MS5611_doconversion(uint8_t command) {
  if(data_ready_flag == 1) {
      Wire.beginTransmission(BARO_ADDRESS);
      Wire.write(command);
      Wire.endTransmission();
      baro_conversion_timer = micros();
    
      if(command == RAW_PRESS_COMMAND) {pressure_flag = 1;temp_flag = 0;}
      else if(command == RAW_TEMP_COMMAND) {pressure_flag = 0;temp_flag = 1;}
  }
}

void MS5611_KAL::MS5611_conversionResult() {
  if(micros() - baro_conversion_timer > 10000) {
    // start read sequence
    Wire.beginTransmission(BARO_ADDRESS);
    Wire.write(0);
    Wire.endTransmission(false);
    Wire.requestFrom(BARO_ADDRESS,3); 
    while(Wire.available() < 3);
    if(pressure_flag == 1) {baro_raw_pressure = Wire.read() * 65536 + Wire.read() * 256 + Wire.read();}
    else if(temp_flag == 1) {baro_raw_temp = Wire.read() * 65536 + Wire.read() * 256 + Wire.read();}
    data_ready_flag = 1;
    } else {
      data_ready_flag = 0;
  }
}

float MS5611_KAL::get_RAW_Pressure() {
  return baro_raw_pressure;
}

float MS5611_KAL::get_RAW_Temperature() {
  return baro_raw_temp;
}

float MS5611_KAL::get_RAW_Altitude() {
  getPressure();
  getTemperature();
  baro_altitude = (pow(2.71828, ((log(baro_pressure * 100) + 18.2573) / 5.25885)) - 288.15) / -0.0065;
  if (StartZero) {
    baro_altitude = baro_altitude - average;
  }
  return baro_altitude;
}

float MS5611_KAL::getPressure() {
  int64_t deltaTemp = (baro_raw_temp - (((int32_t)baro_calibration[4]) << 8));
  int64_t off  = (((int64_t)baro_calibration[1]) << 16) + ((baro_calibration[3] * deltaTemp) >> 7);
  int64_t sens = (((int64_t)baro_calibration[0]) << 15) + ((baro_calibration[2] * deltaTemp) >> 8);
  if (baro_temp < 20) {
  	int64_t off2 = 5 * pow((baro_temp - 2000),2) / 2;
  	int64_t sens2 = 5 * pow((baro_temp - 2000), 2) / 4;
  	if (baro_temp < -15) {
  		off2 = off2 + 7 * pow((baro_temp + 1500), 2);
  		sens2 = sens2 + 11 * pow((baro_temp + 1500), 2) / 2;
  	}
  	off = off - off2;
  	sens = sens - sens2;
  }
  baro_pressure = ((((baro_raw_pressure * sens) >> 21) - off) >> (15-EXTRA_PRECISION)) / ((1<<EXTRA_PRECISION) * 100.0);
  return baro_pressure;
}

float MS5611_KAL::getTemperature() {
  int64_t deltaTemp = (baro_raw_temp - (((int32_t)baro_calibration[4]) << 8));
  baro_temp =  ((1<<EXTRA_PRECISION)*2000l + ((deltaTemp * baro_calibration[5]) >> (23-EXTRA_PRECISION))) / ((1<<EXTRA_PRECISION) * 100.0);
  if (baro_temp < 20) {
  	float t2 = pow(deltaTemp, 2) / 2147483648;
  	baro_temp = baro_temp - t2;
  }
  return baro_temp;
}

float MS5611_KAL::getMS5611data(float USER_ACC = 0, float USER_VEL = 0) {
  // ===========================getting barometer data=====================================
  if(pressure_flag == 0) command = RAW_PRESS_COMMAND; // start with pressure conversion first
  else if(temp_flag == 0) command = RAW_TEMP_COMMAND;
  
  MS5611_doconversion(command);
  MS5611_conversionResult(); // if > 10 ms, collect raw pressure data
  get_RAW_Altitude();
  
  if (FIL_OPTION == 0) {
    ciravg_arr[ciravg_pt] = baro_altitude;//getting the average from CIRAVG_SIZE data
    ciravg_pt = (ciravg_pt + 1) % CIRAVG_SIZE;
    float sum = 0;
    for(int i = 0; i < CIRAVG_SIZE; i++)
    {
        sum += ciravg_arr[i];
      }
    current_altitude = sum / CIRAVG_SIZE;
  } else {
      current_altitude = Kalman_filter(USER_ACC, USER_VEL);
  }
  return current_altitude;
}

void MS5611_KAL::update() {
  if(pressure_flag == 0) command = RAW_PRESS_COMMAND; // start with pressure conversion first
  else if(temp_flag == 0) command = RAW_TEMP_COMMAND;
  
  MS5611_doconversion(command);
  MS5611_conversionResult(); // if > 10 ms, collect raw pressure data
}

