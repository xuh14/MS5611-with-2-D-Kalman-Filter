#include <MS5611_KAL.h>
#include <Gaussian.h> // include Gaunssian for testing
#include <Wire.h>

/** 
 *  1 for Kalman, 0 for circular array averaging
 *  1 to measure relative altitude, 0 for absolute
 */
const int USER_FIL_OPTION = 0;
const int START_ZERO = 1;

/** USER set parameters */
const int dt = 0.025;
const int Q = 0.01;
const int R = 1;
/** USER set parameters  end*/

MS5611_KAL baro = MS5611_KAL(USER_FIL_OPTION);

void setup() {
  Serial.begin(115200);
  
  // if circular array is preferred
  // baro.MS5611_setup(START_ZERO);
  baro.MS5611_setup(START_ZERO, Q, R, dt);   // kalman filtering
  delay(1000);
}


void loop() {
  double mi = micros();
  double baro_altitude = baro.getMS5611data(0, 0);

  //uncommend below two lines to show raw barometer data instead
  //baro.update();
  //float altitude = baro.get_RAW_Altitude();
  
  float USER_ACC = 0;
  float USER_VEL = 0;
  // using gaussian noise for steady state testing
  float mean = 0;
  float st_dev = 0.1;
  Gaussian g= Gaussian(mean, st_dev);
  float noise = g.random();
  float altitude = baro.getMS5611data(noise, noise * dt); // [ACC VEL]

  Serial.print("Altitude: "); Serial.println(altitude);
  //Serial.println(micros()-mi);
  while(micros() - mi < dt*1000000);
}
