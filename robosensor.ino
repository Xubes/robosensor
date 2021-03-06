#include "Wire.h"

// I2Cdev and MPU9150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9150.h"
#include "helper_3dmath.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9150 sensor;

#define ulong unsigned long

double eps = 1e-9;
double gyro_scale = 131; // divisor for gyro ADC values
int16_t rot, gx, gy, gz;
ulong time;
double angle_delta, velocity, angle_position;
double velocity_x, velocity_y, velocity_z;
int16_t gyro_baseline_x, gyro_baseline_y, gyro_baseline_z;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  
  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(115200);
  
  // initialize device
  //Serial.println("Initializing I2C devices...");
  sensor.initialize();
  
  // verify connection
  //Serial.println("Testing device connections...");
  //Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");
  
  
  time = millis();
  angle_position = 0.0;
  angle_delta = 0.0;
  velocity = 0.0;
  
  int16_t mx,my,mz;
  gyroScanBaseline(sensor, 500, &gyro_baseline_x, &gyro_baseline_y, &gyro_baseline_z);
  gyro_baseline_x /= gyro_scale;
  gyro_baseline_y /= gyro_scale;
  gyro_baseline_z /= gyro_scale;
  
  //Serial.printf("Baselines:\t%d,%d,%d\n", gyro_baseline_x, gyro_baseline_y, gyro_baseline_z);
}

void loop() {
  // get rotation data
  sensor.getRotation(&gx,&gy,&gz);
  velocity_x = (gx-gyro_baseline_x)/gyro_scale;
  velocity_y = (gy-gyro_baseline_y)/gyro_scale;
  velocity_z = (gz-gyro_baseline_z)/gyro_scale;
  
  // check against baseline values
  if(abs(velocity_x) < 0.5){
    velocity_x = 0.0;
  }
  if(abs(velocity_y) < 0.5){
    velocity_y = 0.0;
  }
  if(abs(velocity_z) < 0.5){
    velocity_z = 0.0;
  }
  
  /*
  velocity = sqrt(sq(velocity_x) + sq(velocity_y) + sq(velocity_z));
  Serial.println(sq(velocity_x));
  Serial.println(sq(velocity_y));
  Serial.println(sq(velocity_z));
  Serial.println(velocity);
  Serial.println();
  */
  
  // The current position of the sensor should yield only changes in the gyroscope's y-axis.
  // Verified by testing; remove other velocities since they will only throw the chair rotation off.
  velocity = velocity_y;
  
  // Upper sum to approximate integral of angular velocity.
  ulong now = millis();
  ulong dt = now-time;
  time = now;
  double dtheta = (dt/1000.0) * velocity;
  if(abs(dtheta) > eps) {    // dtheta should be positive anyway...
    angle_delta += abs(dtheta);
    angle_position += dtheta;
    serialOut(angle_position, angle_delta, velocity);
  }
  
  delay(5);
}

/* Whenever there is data on Serial port, read it and handle the command.
   Codes:
     48  -   Reset angle_delta to 0.0
     49  -   Reset angle_position to 0.0
*/
void serialEvent(){
  while (Serial.available()){
    char code = (char)Serial.read();
    switch(code){
      case(48):
      angle_delta = 0.0;
      serialOut(angle_position, angle_delta, velocity);
      break;
      
      case(49):
      angle_position = 0.0;
      serialOut(angle_position, angle_delta, velocity);
      break;
    }
  }
}

/* Collect gyro data for a given duration and return the mean values. */
void gyroScanBaseline(MPU9150 sensor, int duration, int16_t* mx, int16_t* my, int16_t* mz){
  int start = millis();
  sensor.getRotation(mx,my,mz);
  int count = 1;
  int16_t gx,gy,gz;
  while(millis()-start < duration){
    sensor.getRotation(&gx,&gy,&gz);
    count++;
    *mx += (gx-*mx)/count;
    *my += (gy-*my)/count;
    *mz += (gz-*mz)/count;
  }
}

/* Convenience function to write data to the Serial port. */
void serialOut(double dp, double dt, double dtps){
  Serial.print(dp);
  Serial.print(',');
  Serial.print(dt);
  Serial.print(',');
  Serial.print(dtps);
  Serial.print("\n");
}
