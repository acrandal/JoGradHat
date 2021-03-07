#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    6

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);


#define EARTH_G         9.807
#define MARS_G          3.71
#define BALL_MASS       1.0
#define GYRO_MIN_CUTOFF 1.0

#define IMPULSE_SCALE   20
#define RING_LENGTH     240

#define COFF_FRICTION   0.3

float currVelocity = 0.0;
float lastLocation = 12;

unsigned long lastMillis = 0;
unsigned long currMillis = 0;

#define LIGHT_COUNT 24

int length_per_light = RING_LENGTH / LIGHT_COUNT;


// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LIGHT_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

#define LIGHT_DEGRADE_FACTOR 0.9

uint8_t lightVals[LIGHT_COUNT];

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  currMillis = millis();
  lastMillis = millis();

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(25); // Set BRIGHTNESS to about 1/5 (max = 255)

  colorWipe(strip.Color(255,   0,   0), 50); // Red
  colorWipe(strip.Color(0,   0,   0), 50); // Red

  for( int i = 0; i < LIGHT_COUNT; i++ ) {
    lightVals[i] = 50;
  }
}

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  /* Display the floating point data */
  //Serial.print("X: ");
  //Serial.print(gyro.x());
  //Serial.println(" rad/s");
  //Serial.print(" Y: ");
  //Serial.print(gyro.y());
  //Serial.print(" Z: ");
  //Serial.print(gyro.z());
  //Serial.print("\t\t");

  currMillis = millis();
  float deltaT = (float)(currMillis - lastMillis) / 1000;
  lastMillis = currMillis;
  // Serial.print("deltaT: " + String(deltaT));

  float gyroXrads = gyro.x();
  if( gyroXrads < GYRO_MIN_CUTOFF && gyroXrads > -1 * GYRO_MIN_CUTOFF) {
    gyroXrads = 0.0;
  }
  
  float impulseForce = gyroXrads * IMPULSE_SCALE;

  //Serial.print("\tGyro X: " + String(gyroXrads) + " rad/s");
  //Serial.print("\timpulseForce: " + String(impulseForce) + " N");

  float deltaVimpulse = (deltaT * impulseForce) / BALL_MASS;
  //Serial.print("\tdeltaVimpulse: " + String(deltaVimpulse) + " m/s");
  
  currVelocity += deltaVimpulse;
  //Serial.print("\tcurrVelocity: " + String(currVelocity) + " m/s");

  float forceFriction = COFF_FRICTION * BALL_MASS * EARTH_G;
  //Serial.print("\forceFric: " + String(forceFriction) + " N");

  float deltaVfriction = (deltaT * forceFriction) / BALL_MASS;
  //Serial.print("\tdVFric: " + String(deltaVfriction) + " m/s");

  if( abs(currVelocity) <= deltaVfriction ) {
    currVelocity = 0.0;
  } else if( currVelocity > 0 ) {
    currVelocity -= deltaVfriction;
  } else {
    currVelocity += deltaVfriction;
  }

  Serial.print("\tcurrVelocity: " + String(currVelocity) + " m/s");

  float currLocation = lastLocation + currVelocity;
  while(currLocation < 0 ) {
    currLocation = RING_LENGTH + currLocation;
  }

  while( currLocation > RING_LENGTH ) {
    currLocation = currLocation - RING_LENGTH;
  }


  Serial.print("\tlastLoc: " + String(lastLocation) + " m");
  Serial.print("\tcurrLoc: " + String(currLocation) + " m");
  // Draw over old lights
  // Draw new lights (degrade values?)
  // Array of light brightnesses -> degrade each loop, redraw


  uint32_t colorOff = strip.Color(  0,   0,   0);
  uint32_t colorRed = strip.Color(  127,   0,   0);


  int lastLightIndex = lastLocation / length_per_light;
  int currLightIndex = currLocation / length_per_light;

  Serial.println("\t" + String(lastLightIndex) + "->" + String(currLightIndex));

  if( currLightIndex != lastLightIndex ) {
    int lightIndex = lastLightIndex;
    int dir = 1;
    if(currVelocity < 0) { dir = -1; }

    while(lightIndex != currLightIndex) {
      lightVals[lightIndex] = 255;
      lightIndex += dir;
      if( lightIndex < 0 ) {
        lightIndex = LIGHT_COUNT-1;   // Off bottom
      }
      lightIndex %= LIGHT_COUNT;    // Off top
    }
    
    //strip.setPixelColor(lastLightIndex, colorOff);
    //strip.setPixelColor(currLightIndex, colorRed);
    //strip.show();
  }

  for( int i = 0; i < LIGHT_COUNT; i++ ) {
    lightVals[i] = lightVals[i] * LIGHT_DEGRADE_FACTOR;
    strip.setPixelColor(i, strip.Color( 0, lightVals[i], 0 ));
  }
  strip.show();



  lastLocation = currLocation;


  // TODO: Calc actual delay MS based on calculation updates
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
