
// Reading an MPU6050 accelerometer with an ESP32 over I2C
// Output is indicated using three LEDs to represent the G-force experience by each 
// X, Y, or Z axis

/* Include the necessary libraries*/
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

/* Define the necessary constants (GPIO pins and G-force range)*/
#define INRANGE_LED_PIN 2 //GPIO pin #0 on the ESP32
#define OUTRANGE_LED_PIN 4 //GPIO pin #2 on the ESP32

#define LOWER_G_RANGE 0.9
#define HIGHER_G_RANGE 3

/* Define the necessary variables 
(MPU6050 object, G-force variables, boolean controls)*/
Adafruit_MPU6050 mpu;
double xG = 0;
double yG = 0;
double zG = 0;
double magG = 0;
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
  pinMode(INRANGE_LED_PIN, OUTPUT);
  pinMode(OUTRANGE_LED_PIN, OUTPUT);
  Serial.println("");
  delay(100);
}

/* Main looping code */
void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Take the absolute value of the acceleration of each axis*/
  xG = abs((a.acceleration.x)/9.81);
  yG = abs((a.acceleration.y)/9.81);
  zG = abs((a.acceleration.z)/9.81);

  /* Compute the combined magnitude of all 3 axes*/
  magG = sqrt((xG*xG) + (yG*yG) + (zG*zG));

  /* Optional normalizing of gravity */
  if(removeGravity){
    if(magG < 1){
      magG = 0;
    }else{
      magG = magG - 1;
    }
  }


  

  /* Print out the G-force values */
  Serial.print("Acceleration X: ");
  Serial.print(xG);
  Serial.print(", Y: ");
  Serial.print(yG);
  Serial.print(", Z: ");
  Serial.print(zG);
  Serial.print(", Magnitude: ");
  Serial.print(magG);
  Serial.println(" G-force");

  Serial.println("");

  /* Set default LED indciators to off */
  digitalWrite(INRANGE_LED_PIN, LOW);
  digitalWrite(OUTRANGE_LED_PIN, LOW);

  /* Indicate the G-force experienced with each axis 
      In operation range = In-range LED turns on (blue)
      Over operational range = Out-of-range LED turns on (green)
      Below operational range = Both LEDs are off
  */ 
  if(magG >= LOWER_G_RANGE && magG <= HIGHER_G_RANGE){
      digitalWrite(INRANGE_LED_PIN, HIGH);
      digitalWrite(OUTRANGE_LED_PIN, LOW);
  }else if(magG > HIGHER_G_RANGE){
    digitalWrite(INRANGE_LED_PIN, LOW);
    digitalWrite(OUTRANGE_LED_PIN, HIGH);
  }else{
    digitalWrite(INRANGE_LED_PIN, LOW);
    digitalWrite(OUTRANGE_LED_PIN, LOW);
  }


}
