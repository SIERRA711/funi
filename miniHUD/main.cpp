#include <MPU6050_light.h>
#include <Arduino.h>
#include <U8g2lib.h> // u8g2 library for drawing on OLED display - needs to be installed in Arduino IDE first
#include <Wire.h> // wire library for IIC communication with the OLED display
#include <U8x8lib.h>

const int mpuAddress = 0x68;          // I2C address of the MPU-6050
const int AD0pin = 19;
const int oledAdd = 0x3c;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

MPU6050 mpu(Wire);
unsigned long previousMillis;
const unsigned long interval = 500;
int center_x = 64; // display x center, 64px for the 128x128px display
int center_y = 32; // display y center, 64px for the 128x128px display

float roll = 0;
float yaw = 0;
float pitch= 0;


// Setup
void setup() {
  Serial.begin(115200);
  Wire.begin();
  u8g2.begin();  // begin the u8g2 library
  u8g2.setContrast(255); 

  pinMode(AD0pin,OUTPUT);
  digitalWrite(AD0pin,LOW);
  Serial.println("Initializing MPU6050...");
  if (mpu.begin() != 0) {
    Serial.println("Failed to initialize MPU6050!");
    while (1);  // Halt if initialization fails
  }
  Serial.println("MPU6050 ready.");
  delay(500);
  previousMillis = millis();
}


// drawing the HUD W manually
void drawRotatedW(float roll, float yaw) {
  const int numPoints = 7;
  int origX[numPoints] = {-20, -15, -10, 0, 10, 15, 20}; // More symmetrical
  int origY[numPoints] = {0, 0, 10, 0, 10, 0, 0};

  int rotatedX[numPoints];
  int rotatedY[numPoints];

  // Convert angle from degrees to radians
  float angleRadroll = roll * PI / 180.0;
  float angleRadyaw = yaw * PI / 180.0;

  const float scalingFactor = 1.0;
  int yawOffset = (int)(yaw * scalingFactor);

  // Rotate each point using the 2D rotation formula
  for (int i = 0; i < numPoints; i++) {
    rotatedX[i] = origX[i] * cos(angleRadroll) - origY[i] * sin(angleRadroll);
    rotatedY[i] = origX[i] * sin(angleRadroll) + origY[i] * cos(angleRadroll);

    // Offset by the center of the display
    rotatedX[i] += center_x + yawOffset;
    rotatedY[i] += center_y;

    // Wrap coordinates for continuous boundary condition
    if (rotatedX[i] < 0) rotatedX[i] += 128; // Assuming screen width is 128
    if (rotatedX[i] >= 128) rotatedX[i] -= 128;
    if (rotatedY[i] < 0) rotatedY[i] += 64;  // Assuming screen height is 64
    if (rotatedY[i] >= 64) rotatedY[i] -= 64;
  }

  // Draw lines connecting the points of the "W"
  for (int i = 0; i < numPoints - 1; i++) {
    int x1 = rotatedX[i];
    int y1 = rotatedY[i];
    int x2 = rotatedX[i + 1];
    int y2 = rotatedY[i + 1];

    // Handle line wrapping if endpoints are on opposite sides
    if (abs(x2 - x1) > 64) { // Horizontal wrap
      if (x1 > x2) x2 += 128;
      else x1 += 128;
    }

    if (abs(y2 - y1) > 32) { // Vertical wrap
      if (y1 > y2) y2 += 64;
      else y1 += 64;
    }

    u8g2.drawLine(x1 % 128, y1 % 64, x2 % 128, y2 % 64);
  }
}

void drawHorizon(float pitch) {
  const float scalingFactor = 1.0; // Adjust as needed (pixels per degree)
  int pitchOffset = (int)(-pitch * scalingFactor);
  int lineY = center_y + pitchOffset;
  // Wrap Y coordinate for continuous boundary condition
  while (lineY < 0) lineY += 64;
  lineY = lineY % 64;

  // Draw a horizontal line across the display at lineY
  u8g2.drawHLine(10, lineY, 118);
}

void loop() {
  unsigned long currentMillis = millis();
  mpu.update();
  if(currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    roll = mpu.getAngleX();
    pitch = mpu.getAngleY();
    yaw = mpu.getAngleZ();
    Serial.print(F(" Roll = "));
    Serial.print(roll,0);
    Serial.print(F(" Pitch = "));
    Serial.print(pitch,0);
    Serial.print(F(" Yaw = "));
    Serial.print(yaw,0);
    delay(50);
  }

  u8g2.firstPage();
  do {
    u8g2.drawLine(34, 22, 34, 52);
    drawRotatedW(roll, yaw);
    u8g2.drawLine(94, 22, 94, 52);
    drawHorizon(pitch);
    if(pitch <-90 ) u8g2.print(F("PULL UP"));
    if(pitch >90 ) u8g2.print(F("PULL DOWN"));
  } while (u8g2.nextPage());
  
  delay(50);

}
