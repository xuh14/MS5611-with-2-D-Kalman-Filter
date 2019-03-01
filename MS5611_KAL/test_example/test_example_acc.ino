#include <MS5611_KAL.h>
#include <Gaussian.h> // include Gaunssian for testing
#include <Wire.h>

/** 
 *  1 for Kalman, 0 for circular array averaging
 *  1 to measure relative altitude, 0 for absolute
 */
const int USER_FIL_OPTION = 1;
const int START_ZERO = 0;

/** USER set parameters */
const int dt = 0.025;
const int Q = 0.01;
const int R = 1;
/** USER set parameters  end*/

// =====================accelerometer parameters=========================
#define mpu_addr 0x68
int16_t acc_x, acc_y, acc_z; // raw data
float vel = 0, dis = 0;
//========================================================================

MS5611_KAL baro = MS5611_KAL(USER_FIL_OPTION);

void setup() {
  Serial.begin(115200);
  
  // if circular array is preferred
  // baro.MS5611_setup(START_ZERO);
  baro.MS5611_setup(START_ZERO, Q, R, dt);   // kalman filtering

  acc_setup(); // setup accelerometer
  delay(1000);
}


void loop() {
  double mi = micros();
  double baro_altitude = baro.getMS5611data(0, 0);


  float acc_down = acc_read();
  acc_down -= 1.01222; // bias to 0
  acc_down = acc_down * 9.82;
  vel = vel + acc_down * dt;
  
  //uncommend below two lines to show raw barometer data instead
  baro.update();
  float raw_altitude = baro.get_RAW_Altitude();
  float altitude = baro.getMS5611data(acc_down, vel); // [ACC VEL]
  
  Serial.print("Altitude(raw): "); Serial.print(raw_altitude); 
  Serial.print(" Altitude(Kalman): "); Serial.println(altitude);
  
  // uncommend this line to examinate accelerometer data
  //Serial.print(" acc_z (m/s): "); Serial.print(acc_down); Serial.print(" vel (m/s): "); Serial.print(vel); Serial.print(" distance (m): "); Serial.println(dis);
  
  while(micros() - mi < dt*1000000);
}

void acc_setup() {
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x6B); // acc address
  Wire.write(0);
  Wire.endTransmission();
  
  // set at +- 2g
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x1C);
  Wire.write(0); // set +- 2g
  Wire.endTransmission();

  Wire.beginTransmission(mpu_addr);
  Wire.write(0x1D);
  Wire.write(0x0D);  // bandwidth 5Hz
  Wire.endTransmission();
}

float acc_read() {
  int sum = 0;
  int average = 0;
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x3B); // acc address
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_addr, 6);
  while(Wire.available() < 6); // wait
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();

  acc_z = acc_z - 4499.45; // z_bias
  
  float acc_down = sqrt(pow(acc_x, 2) + pow(acc_y, 2) + pow(acc_z, 2));
  if (acc_z > 0) {
    acc_down = -acc_down;
  }
  float g = acc_down * 0.061 / 1000; // convert to g
  return g;
}
