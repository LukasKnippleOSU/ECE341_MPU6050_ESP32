
// Reading an MPU6050 accelerometer with an ESP32 over I2C
// Output is indicated using three LEDs to represent the G-force experience by each 
// X, Y, or Z axis

/* Include the necessary libraries*/
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

/* Define the necessary constants (GPIO pins and G-force range)*/
#define X_LED_PIN 4 //GPIO pin #4 on the ESP32
#define Y_LED_PIN 0 //GPIO pin #0 on the ESP32
#define Z_LED_PIN 2 //GPIO pin #2 on the ESP32
#define LOWER_G_RANGE 0
#define HIGHER_G_RANGE 1

/* Define the necessary variables (MPU6050 object, boolean values, gravity setting)*/
Adafruit_MPU6050 mpu;
bool triggerX = false;
bool triggerY = false;
bool triggerZ = false;
bool removeGravity = false;

/* Setup settings */
void setup(void) {
  
  /* Set baud rate */
  Serial.begin(115200);
  
  /* Pause until Serial port opens*/
  while (!Serial)
    delay(10);

  /* Attempt to communicate with MPU6050 */
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  /* Initialize MPU6050 settings (G range, gyro range, and bandwidth)*/
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  Serial.print(" --- Accelerometer range ---- ");
  Serial.println("Range: +-4G");
  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  Serial.println("+- 500 deg/s");

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.println("Filter bandwidth set to: 5 Hz");
  
  //Set all LED pins to output:
  pinMode(X_LED_PIN, OUTPUT);
  pinMode(Y_LED_PIN, OUTPUT);
  pinMode(Z_LED_PIN, OUTPUT);
  Serial.println("");
  delay(100);
}

/* Main looping code */
void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Take the absolute value of the acceleration of each axis*/
  double xG = abs((a.acceleration.x)/9.81);
  double yG = abs((a.acceleration.y)/9.81);
  double zG = abs((a.acceleration.z)/9.81);

  /* Optional normalizing of gravity*/
  if(removeGravity){
    if(xG < 1){
      xG = 0;
    }else{
      xG = xG - 1;
    }
  
    if(yG < 1){
      yG = 0;
    }else{
      yG = yG - 1;
    }
  
    if(zG < 1){
      zG = 0;
    }else{
      zG = zG - 1;
    }
  }
  

  /* Print out the G-force values */
  Serial.print("Acceleration X: ");
  Serial.print(xG);
  Serial.print(", Y: ");
  Serial.print(yG);
  Serial.print(", Z: ");
  Serial.print(zG);
  Serial.println(" G-force");

  Serial.println("");

  /* Set default LED indciators to off */
  digitalWrite(X_LED_PIN, LOW);
  digitalWrite(Y_LED_PIN, LOW);
  digitalWrite(Z_LED_PIN, LOW);

  /* Indicate the G-force experienced with each axis 
      In operation range = LED solid on
      Over operational range = LED blinking
  */ 
  if(xG > LOWER_G_RANGE && xG < HIGHER_G_RANGE){
      digitalWrite(X_LED_PIN, HIGH);
  }else if(xG > HIGHER_G_RANGE){
    if(triggerX == 0){
      digitalWrite(X_LED_PIN, HIGH);
      triggerX = 1;
    }else{
      digitalWrite(X_LED_PIN, LOW);
      triggerX = 0;
    }  
  }else{
      digitalWrite(X_LED_PIN, LOW);
    }

    if(yG > LOWER_G_RANGE && yG < HIGHER_G_RANGE){
      digitalWrite(Y_LED_PIN, HIGH);
    }else if(yG < HIGHER_G_RANGE){
      if(triggerY == 0){
        digitalWrite(Y_LED_PIN, HIGH);
        triggerY = 1;
      }else{
        digitalWrite(Y_LED_PIN, LOW);
        triggerY = 0;
      }
    }else{
      digitalWrite(Y_LED_PIN, LOW);
    }

    if(zG > LOWER_G_RANGE && zG < HIGHER_G_RANGE){
      digitalWrite(Z_LED_PIN, HIGH);
    }else if(zG > HIGHER_G_RANGE){
    if(triggerZ == 0){
      digitalWrite(Z_LED_PIN, HIGH);
      triggerZ = 1;
    }else{
      digitalWrite(Z_LED_PIN, LOW);
      triggerZ = 0;
    }
  }else{
    digitalWrite(Z_LED_PIN, LOW);
  }

  /* Add delay to enable blinking of LEDs */
  delay(100);
}