
#include "SPI.h"
#include "respeaker.h"

#include <Servo.h>

#include <Adafruit_NeoPixel.h>

#include "Wire.h"

// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9250.h"

MPU9250 accelgyro;
I2Cdev   I2C_M;

uint8_t buffer_m[6];


int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;
float Mxyz[3];
float heading;
float last_heading;
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

#define PIXELS_PIN      11
#define PIXELS_NUM      12
#define PIXELS_SPACE    128

#define SERVO_PIN       A3

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(PIXELS_NUM, PIXELS_PIN, NEO_GRB + NEO_KHZ800);
int pixels_state = 0;
uint8_t ShakeFlag = 0;

const char *pixels_patterns[] = {"sleep", "wakeup", "wait", "answer", "offline", "online"};

char *voicefile[]  = {"madplay /tmp/run/mountd/mmcblk0p1/audio/1.mp3",
                      "madplay /tmp/run/mountd/mmcblk0p1/audio/2.mp3",
                      "madplay /tmp/run/mountd/mmcblk0p1/audio/3.mp3",
                      "madplay /tmp/run/mountd/mmcblk0p1/audio/4.mp3",
                      "madplay /tmp/run/mountd/mmcblk0p1/audio/5.mp3"};


void touch_event(uint8_t id, uint8_t event) {
  //  Serial << "id:" << id << " event:" << event << "\r\n";
}

void spi_event(uint8_t addr, uint8_t *data, uint8_t len)
{
  for (uint8_t i = 0; i < sizeof(pixels_patterns) / sizeof(*pixels_patterns); i++) {
    if (!strcmp(pixels_patterns[i], (char *)data)) {
      pixels_state = i;
      break;
    }
  }
  
}

void setup() {

  Wire.begin();
  accelgyro.initialize();
  pixels.begin();
  for (int i = 0; i < PIXELS_NUM; i++) {
    pixels.setPixelColor(i, 0, 0, 32);
  }
  pixels.show();

  respeaker.begin();
  respeaker.attach_touch_isr(touch_event);
  respeaker.attach_spi_isr(spi_event);

  delay(1000);
  pixels.clear();
  pixels.show();
  
  uint8_t index = random(0,4);
  Serial.print("The index is");
  Serial.println(index);
  respeaker.exec(voicefile[index]);

//  pixels_state = 3;
}

void loop() {
  static uint32_t last_time = 0;
  getCompassDate_calibrated(); // compass data has been calibrated here
  getHeading();               //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
  if (abs(heading - last_heading) >= 10) {
      ShakeFlag = 1;
      last_heading = heading;
  }
   uint32_t current = millis();
  
   if ((ShakeFlag == 1) && (uint32_t)(current - last_time) > 5000) {

        Serial.print("----Heading---------");
        Serial.println(heading);
        Serial.println("----Play voice---------");
        uint8_t index = random(0,4);
        Serial.print("The index is");
        Serial.println(index);
        respeaker.exec(voicefile[index]);
        shakeHead();
        
        ShakeFlag = 0;
        last_time = current;

    }
        

}


void shakeHead() {
  myservo.attach(SERVO_PIN);
  for (pos = 90; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
                // waits 15ms for the servo to reach the position
  }
  delay(800);

  for (pos = 180; pos >= 90; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
                  // waits 15ms for the servo to reach the position
  }
  delay(800);
  myservo.detach();
}

void getHeading(void)
{
    heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
    if (heading < 0) heading += 360;
}

void getCompass_Data(void)
{
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    delay(10);
    I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
    my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
    mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

    Mxyz[0] = (double) mx * 1200 / 4096;
    Mxyz[1] = (double) my * 1200 / 4096;
    Mxyz[2] = (double) mz * 1200 / 4096;
}

void getCompassDate_calibrated ()
{
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;
}