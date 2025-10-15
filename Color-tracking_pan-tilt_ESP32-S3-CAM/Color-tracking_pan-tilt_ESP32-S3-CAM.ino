#include "esp_camera.h"
#include <Servo.h>

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      40
#define SIOD_GPIO_NUM      17
#define SIOC_GPIO_NUM      18
#define Y9_GPIO_NUM        39
#define Y8_GPIO_NUM        41
#define Y7_GPIO_NUM        42
#define Y6_GPIO_NUM        12
#define Y5_GPIO_NUM        3
#define Y4_GPIO_NUM        14
#define Y3_GPIO_NUM        47
#define Y2_GPIO_NUM        48
#define VSYNC_GPIO_NUM     38
#define HREF_GPIO_NUM      21
#define PCLK_GPIO_NUM      11

#define PAN_SERVO_PIN 14   // adjust if used by camera, can use 2,4,15,16 instead
#define TILT_SERVO_PIN 15

Servo panServo;
Servo tiltServo;

int panPos = 90;
int tiltPos = 90;

//  COLOR TARGET (HSV RANGE) - Example: orange object
uint8_t targetHueMin = 5;
uint8_t targetHueMax = 25;
uint8_t targetSatMin = 100;
uint8_t targetValMin = 60;

//  FRAME PARAMETERS 
const int frameWidth = 320;
const int frameHeight = 240;
const int centerX = frameWidth / 2;
const int centerY = frameHeight / 2;

// PID control constants
float Kp = 0.05;

void setup() {
  Serial.begin(115200);

  //  CAMERA CONFIG 
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    while (true);
  }

  //  SERVO SETUP 
  panServo.attach(PAN_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);
  panServo.write(panPos);
  tiltServo.write(tiltPos);

  Serial.println("Color Tracking Start");
}

void loop() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Find average position of pixels in the desired HSV range
  uint32_t sumX = 0, sumY = 0, count = 0;
  for (int y = 0; y < fb->height; y += 2) {
    for (int x = 0; x < fb->width; x += 2) {
      uint16_t pixel = ((uint16_t *)fb->buf)[y * fb->width + x];
      uint8_t r = (pixel >> 11) << 3;
      uint8_t g = ((pixel >> 5) & 0x3F) << 2;
      uint8_t b = (pixel & 0x1F) << 3;
      uint8_t maxc = max(r, max(g, b));
      uint8_t minc = min(r, min(g, b));
      uint8_t diff = maxc - minc;

      float hue = 0;
      if (diff == 0) hue = 0;
      else if (maxc == r) hue = fmod((60 * ((g - b) / (float)diff) + 360), 360);
      else if (maxc == g) hue = fmod((60 * ((b - r) / (float)diff) + 120), 360);
      else hue = fmod((60 * ((r - g) / (float)diff) + 240), 360);

      uint8_t sat = (maxc == 0) ? 0 : (diff * 255 / maxc);
      uint8_t val = maxc;

      if (hue >= targetHueMin && hue <= targetHueMax &&
          sat >= targetSatMin && val >= targetValMin) {
        sumX += x;
        sumY += y;
        count++;
      }
    }
  }

  if (count > 100) {
    int avgX = sumX / count;
    int avgY = sumY / count;

    int errorX = centerX - avgX;
    int errorY = centerY - avgY;

    panPos += Kp * errorX;
    tiltPos -= Kp * errorY;

    panPos = constrain(panPos, 20, 160);
    tiltPos = constrain(tiltPos, 20, 160);

    panServo.write(panPos);
    tiltServo.write(tiltPos);

    Serial.printf("Target @ (%d,%d)  Servos: pan=%d tilt=%d\n", avgX, avgY, panPos, tiltPos);
  } else {
    Serial.println("No target found");
  }

  esp_camera_fb_return(fb);
}
