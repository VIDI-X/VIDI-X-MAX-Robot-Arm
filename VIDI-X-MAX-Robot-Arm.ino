// ================================================
// ============ VIDI X ROBOT ARM 4 DOF ============
// ===== Touchscreen & Potentiometer control =======
// ==================== v1.0 ======================
// ================================================
// - 4 x MicroServo 9g SC90
// - 4 x Potentiometer 10K
// - I2C 16-bit ADS1115 analog-to-digital converter
// - 3D print Serv-Arm: https://www.thingiverse.com/thing:1684471
// ================================================
// -------------
// 3D print has several drawbacks that need fixing:
// -------------
// 1) The base of the arm has too small an opening for the servo cable, so it should be widened.
//    The servo must have the cable exiting at the bottom; most servos don't, so not all fit this 3D base
//    (either adapt the 3D print for all servos or get compatible ones).
// 2) The servo above the base doesn't fit into the intended opening; either get a special screw with a washer
//    or widen the hole in the 3D joint.
// 3) The next joint has too big a hole, so you'll need to invent a fix to connect it.
// 4) The gripper uses a spring, so a micro servo may lack the strength for a firm grip.
// 5) The whole arm is top-heavy relative to its base – make sure to mount the base securely!
// ================================================
// -------------
// Code improvements / future ideas:
// -------------
// - Add touchscreen calibration so the UI feels equally responsive on all devices.
// - Play, Delete and Save are touch-only; consider mapping them to other buttons as well.
// - In Play mode, visually indicate which position is currently active.
// - Add persistent position saving to SPIFFS.
// - Create a web UI to allow manual editing of servo positions from a PC.
// - Add initial servo calibration so the drawn arm better matches the physical robot.
//
// ================================================

#include <Wire.h>
#include "ADS1115-SOLDERED.h"  // Library for ADS1115 analog-to-digital converter
#include <ESP32Servo.h>        // Library for controlling servo motors
#include <SPI.h>
#include "Adafruit_ILI9341.h"     // Library for TFT display
#include "Adafruit_GFX.h"         // Basic graphics for TFT
#include <XPT2046_Touchscreen.h>  // Library for touchscreen

// ==== TFT & TOUCH DEFINITIONS ====
// Here we define pins for TFT and touchscreen
#define TFT_CS 5
#define TFT_DC 21
#define TS_CS 4
Adafruit_ILI9341 TFT = Adafruit_ILI9341(TFT_CS, TFT_DC);
XPT2046_Touchscreen ts(TS_CS);

// Sliders – these are rectangles you can move by finger or potentiometer to change servo angles
#define SLIDER_WIDTH 150
#define SLIDER_HEIGHT 15
#define SLIDER_X 10
const int SLIDER_Y[4] = { 40, 60, 80, 100 };  // Y positions of each slider on screen

// Touchscreen calibration – change if your touch input is inaccurate
#define TS_MINX 150
#define TS_MINY 130
#define TS_MAXX 3800
#define TS_MAXY 4000

// Colors used in the program
enum { BLACK = 0x0000,
       BLUE = 0x001F,
       RED = 0xF800,
       WHITE = 0xFFFF,
       GRAY = 0x7BEF };

// ==== Memory for servo positions ====
// Here we store the user-saved positions for all servos
#define MAX_POSITIONS 25
int savedPositions[MAX_POSITIONS][4];  // Each position has 4 angles (for 4 servos)
int numPositions = 0;                  // Number of saved positions

// ==== Button definitions ====
// Coordinates and size of each button on screen
#define BTN_W 40
#define BTN_H 18
#define BTN_X1 20
#define BTN_X2 (BTN_X1 + BTN_W + 8)
#define BTN_X3 (BTN_X2 + BTN_W + 8)
#define BTN_Y 117

// ==== Servo definitions ====
// Define pins for your servos (change according to your hardware!)
Servo servos[4];
const int servoPins[4] = { 13, 14, 27, 2 };  // ESP32 pins for servos
int sliderValues[4] = { 90, 90, 90, 90 };    // Initial angle for each servo (0-180)
float smoothSlider[4] = { 90, 90, 90, 90 };  // Variable for smoothing slider movements
#define SERVO_ROTATION 180                   // Maximum rotation angle (adjust for your servos!)
// For smooth animation in PLAY mode
int playCurrent[4] = { 90, 90, 90, 90 };     // Current arm position during PLAY
// inMotion is no longer needed in the new version, but you can leave as comment for future expansion
// bool inMotion = false;

// ==== ADS1115 DEFINITIONS ====
// Definitions for using ADS1115 ADC to read potentiometers
ADS1115 ADS;
volatile bool RDY = false;            // Set by interrupt, signals when reading is done
uint8_t adsChannel = 0;               // Current ADS1115 channel (there are 4)
int16_t adsVals[4] = { 0, 0, 0, 0 };  // Stores ADC readings
#define I2C_SDA 33
#define I2C_SCL 32
#define ADC_INTERRUPT 36  // Pin for interrupt signal from ADS1115 (ALERT/RDY pin)

// ==== Touch only mode ====
// If touchOnly is true, potentiometers are ignored and the arm is moved only by touch
bool touchOnly = true;
bool lastTouchOnlyDrawn = false;

// ==== Button types and status ====
// Track which button is active and for how long (for visual feedback)
enum ButtonType { NONE,
                  BTN_SAVE,
                  BTN_PLAY,
                  BTN_DEL };
ButtonType activeButton = NONE;
unsigned long buttonClickTime = 0;

// ==== PLAY animation mode ====
// When playMode is true, the robot runs through the saved positions as a programmed sequence
bool playMode = false;
unsigned long playTimer = 0;
int playStep = 0;
const unsigned long playPause = 800;  // Pause between positions (ms)



// ==== FUNCTION PROTOTYPES ====
// Only function declarations here; actual implementations are below
void drawSlider(int num, int value);
void readPotsFromADS();
void handleTouch();
void updateServos();
void drawRobotArm();
void handleADSConversion();
void adsReady();
void drawTouchOnlyButton(bool force = false);
void drawButton(ButtonType btn, bool active);
void drawPositions();
void drawServoAngles();

void setup() {
  Serial.begin(115200);  // Serial communication for debugging

  TFT.begin();            // Start TFT display
  ts.begin();             // Start touchscreen
  TFT.setRotation(3);     // Rotate display if needed
  ts.setRotation(1);      // Rotate touch axes if needed
  TFT.fillScreen(BLACK);  // Clear screen

  // ==== Servo initialization ====
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < 4; i++) {
    servos[i].setPeriodHertz(50);  // Standard servo PWM frequency
    // For demo, 500-2500 is the full range for most servos
    servos[i].attach(servoPins[i], 500, 2500);
    // servos[i].attach(servoPins[i], 1000, 2000); // Safer but smaller range for some servos
    // servos[i].attach(servoPins[i], 600, 2400);
  }

  for (int i = 0; i < 4; i++) drawSlider(i, sliderValues[i]);  // Draw all sliders

  // ==== ADS1115 initialization ====
  Wire.begin(I2C_SDA, I2C_SCL);                                             // Start I2C on specified pins
  pinMode(ADC_INTERRUPT, INPUT_PULLUP);                                     // Interrupt pin
  attachInterrupt(digitalPinToInterrupt(ADC_INTERRUPT), adsReady, RISING);  // Set interrupt routine

  ADS.begin();                             // Start ADS1115
  ADS.setGain(0);                          // Voltage range (6.144V, change for your pots!)
  ADS.setDataRate(4);                      // Sampling rate (128 SPS, can increase if needed)
  ADS.setComparatorThresholdHigh(0x8000);  // Thresholds for interrupt
  ADS.setComparatorThresholdLow(0x0000);
  ADS.setComparatorQueConvert(0);
  ADS.setMode(0);           // Continuous mode (keeps reading)
  ADS.readADC(adsChannel);  // Start first reading

  // ==== Draw initial screen ====
  drawTouchOnlyButton(true);
  drawButton(BTN_SAVE, false);
  drawButton(BTN_PLAY, false);
  drawButton(BTN_DEL, false);
  drawServoAngles();
  drawRobotArm();
  drawPositions();
}

void loop() {
  handleADSConversion();  // Update potentiometer readings via ADS1115
  readPotsFromADS();      // Set sliders according to potentiometers, unless touchOnly or playMode
  handleTouch();          // Handle touch input (sliders, buttons)

  // ==== If PLAY mode is on, automatically run through saved positions ====
  if (playMode && numPositions > 0) {
      int* target = savedPositions[playStep];

      // Check if all servos have reached their target angle
      bool reached = true;
      for (int i = 0; i < 4; i++) {
        if (playCurrent[i] != target[i]) reached = false;
      }
      if (reached) {
        playStep++;
        if (playStep >= numPositions) {
          playMode = false;
          playStep = 0;
          drawButton(BTN_PLAY, false);
        } else {
          // Moving to next position, don't return – start new target immediately!
        }
        delay(150); // Small pause before next position
      } else {
        // Move each servo by 1 degree toward its target
        for (int i = 0; i < 4; i++) {
          if (playCurrent[i] < target[i]) playCurrent[i]++;
          else if (playCurrent[i] > target[i]) playCurrent[i]--;
        }
        // Update display and servos
        for (int i = 0; i < 4; i++) {
          sliderValues[i] = playCurrent[i];
          smoothSlider[i] = playCurrent[i];
        }
        updateServos();
        drawRobotArm();
        drawSlider(0, sliderValues[0]);
        drawSlider(1, sliderValues[1]);
        drawSlider(2, sliderValues[2]);
        drawSlider(3, sliderValues[3]);
        drawServoAngles();

        delay(10); // Higher delay = slower, smoother movement
      }
  }
  else {
    updateServos();
    drawRobotArm();
    drawServoAngles();
  }



  // ==== Button visual feedback (button "lights up" for 1 second after click) ====
  if (activeButton != NONE && activeButton != BTN_PLAY) {
    if (millis() - buttonClickTime > 1000) {
      drawButton(activeButton, false);
      activeButton = NONE;
    }
  }
  if (activeButton == BTN_PLAY && !playMode) {
    drawButton(BTN_PLAY, false);
    activeButton = NONE;
  }

  drawPositions();  // Show saved positions (only redraws if there are changes)
  // drawTouchOnlyButton(); // Not needed every loop, only when touchOnly changes
  delay(1);  // Very short delay – smooth display, low MCU load
}

// ==== READ POTS FROM ADS ====
// This function reads ADS1115 values and sets sliders accordingly
void readPotsFromADS() {
  if (touchOnly) return;  // Skip potentiometers if touchOnly mode
  if (playMode) return;   // Skip all if in PLAY mode

  for (int i = 0; i < 4; i++) {
    int value = constrain(adsVals[i], 500, 17500);  // Limit values for stability
    int mapped = map(value, 500, 17500, 0, 180);    // Map to 0-180 for servo

    float alpha = 0.15;  // Smoothing factor: lower = smoother but slower
    smoothSlider[i] = alpha * mapped + (1 - alpha) * smoothSlider[i];

    sliderValues[i] = int(smoothSlider[i]);
    drawSlider(i, sliderValues[i]);  // Draw slider with new value
  }
}

// ==== TOUCH HANDLER ====
// This function handles all touch input (sliders, buttons, touchOnly button)
void handleTouch() {
  if (playMode) return;              // Block manual interaction in PLAY mode
  if (activeButton != NONE) return;  // Wait for feedback to finish if button already clicked

  if (ts.touched()) {                                    // Check for touch input
    TS_Point p = ts.getPoint();                          // Get touch point
    int x = map(p.x, TS_MINX, TS_MAXX, 0, TFT.width());  // Map to screen coordinates
    int y = map(p.y, TS_MINY, TS_MAXY, 0, TFT.height());

    // ==== TouchOnly toggle button ====
    if (x > 5 && x < 29 && y > 5 && y < 29) {
      touchOnly = !touchOnly;  // Switch between potentiometer and touch control
      drawTouchOnlyButton();   // Update button color
      delay(200);              // Debounce
      return;
    }

    // ==== Sliders (if button not clicked) ====
    for (int i = 0; i < 4; i++) {
      if (x > SLIDER_X && x < SLIDER_X + SLIDER_WIDTH && y > SLIDER_Y[i] && y < SLIDER_Y[i] + SLIDER_HEIGHT) {
        sliderValues[i] = map(x, SLIDER_X, SLIDER_X + SLIDER_WIDTH, 0, SERVO_ROTATION);
        smoothSlider[i] = sliderValues[i];
        drawSlider(i, sliderValues[i]);
        drawServoAngles();
      }
    }

    // ==== SAVE button (stores current angles) ====
    if (x > BTN_X1 && x < BTN_X1 + BTN_W && y > BTN_Y && y < BTN_Y + BTN_H) {
      activeButton = BTN_SAVE;            // Mark button active (for color)
      buttonClickTime = millis();         // Record click time
      drawButton(BTN_SAVE, true);         // Draw blue button
      if (numPositions < MAX_POSITIONS) { // If there's space in the array
        for (int i = 0; i < 4; i++)
          savedPositions[numPositions][i] = sliderValues[i];  // Save all 4 angles
        numPositions++;                                       // Increment position count
        drawPositions();                                      // Update position display
      }
      return;  // End function as button was clicked
    }

    // ==== PLAY button (runs position sequence) ====
    if (x > BTN_X2 && x < BTN_X2 + BTN_W && y > BTN_Y && y < BTN_Y + BTN_H) {
      activeButton = BTN_PLAY;
      buttonClickTime = millis();
      drawButton(BTN_PLAY, true);
      if (numPositions > 0) {  // Only if positions saved
        playMode = true;       // This is crucial!
        playStep = 0;
        playTimer = millis();  // Not using -playPause, as you move step by step
        for (int i = 0; i < 4; i++) playCurrent[i] = sliderValues[i];
        // inMotion = false; // No longer needed
      }
      return;
    }

    // ==== DEL. button (removes last position) ====
    if (x > BTN_X3 && x < BTN_X3 + BTN_W && y > BTN_Y && y < BTN_Y + BTN_H) {
      activeButton = BTN_DEL;
      buttonClickTime = millis();
      drawButton(BTN_DEL, true);
      if (numPositions > 0) numPositions--;  // Remove last position
      drawPositions();                       // Update display
      return;
    }
  }
}

// ==== SERVO UPDATE ====
// Sends (angle) values to all servos
void updateServos() {
  for (int i = 0; i < 4; i++) servos[i].write(sliderValues[i]);
}

// ==== DRAW SLIDER ====
// Draws one slider (visual representation and value)
void drawSlider(int num, int value) {
  TFT.fillRect(SLIDER_X, SLIDER_Y[num], SLIDER_WIDTH, SLIDER_HEIGHT, GRAY);  // Gray slider (background)
  int pos = map(value, 0, SERVO_ROTATION, 0, SLIDER_WIDTH);                  // Blue slider (current value)
  TFT.fillRect(SLIDER_X, SLIDER_Y[num], pos, SLIDER_HEIGHT, BLUE);
  TFT.drawRect(SLIDER_X, SLIDER_Y[num], SLIDER_WIDTH, SLIDER_HEIGHT, WHITE);  // Outline
}

// ==== ROBOT ARM DRAW ====
// Draws a simple robot arm (top view) for easy understanding of which joint moves
void drawRobotArm() {
  int baseX = 90;   // Arm base X on display
  int baseY = 180;  // Arm base Y on display

  // Refresh only arm area (avoid screen flicker!)
  TFT.fillRect(30, 140, 110, 100, GRAY);

  // Calculate angles for each joint (convert from degrees to radians)
  float angle1 = radians(sliderValues[0]);
  float angle2 = radians(sliderValues[1] - 90);  // Adjusted for display orientation
  float angle3 = radians(sliderValues[2] - 90);

  // "Bone" lengths on screen
  int length1 = 10, length2 = 15, length3 = 20;

  // Calculate joint positions
  int joint1X = baseX + length1 * cos(angle1);
  int joint1Y = baseY + length1 * sin(angle1);
  int joint2X = joint1X + length2 * cos(angle1 + angle2);
  int joint2Y = joint1Y + length2 * sin(angle1 + angle2);
  int joint3X = joint2X + length3 * cos(angle1 + angle2 + angle3);
  int joint3Y = joint2Y + length3 * sin(angle1 + angle2 + angle3);

  // Draw arm segments (different colors for clarity)
  TFT.drawLine(baseX, baseY, joint1X, joint1Y, RED);
  TFT.drawLine(joint1X, joint1Y, joint2X, joint2Y, BLUE);
  TFT.drawLine(joint2X, joint2Y, joint3X, joint3Y, RED);

  // Draw gripper at end (circle, size by slider 4)
  TFT.fillCircle(joint3X, joint3Y, map(sliderValues[3], 0, 180, 2, 8), BLUE);
}

// ==== SHOW SERVO ANGLES IN TWO ROWS ====
// Shows all 4 slider/servo values
void drawServoAngles() {
  int x = 35, y = 0;
  TFT.setTextColor(WHITE, BLACK);
  TFT.setTextSize(1);
  TFT.setCursor(x, y);
  TFT.print("Servo positions: ");
  TFT.setCursor(x, y + 10);
  TFT.print("SER_1:");
  TFT.print(sliderValues[0]);
  TFT.print(" SER_2:");
  TFT.print(sliderValues[1]);
  TFT.print("  ");
  TFT.setCursor(x, y + 20);
  TFT.print("SER_3:");
  TFT.print(sliderValues[2]);
  TFT.print(" SER_4:");
  TFT.print(sliderValues[3]);
  TFT.print("  ");
}

// ==== DRAW BUTTONS ====
// Draws one button, can be blue (active) or gray (inactive)
void drawButton(ButtonType btn, bool active) {
  int x, y = BTN_Y, w = BTN_W, h = BTN_H;
  const char* label;
  uint16_t color;

  switch (btn) {
    case BTN_SAVE:
      x = BTN_X1;
      label = "SAVE";
      break;
    case BTN_PLAY:
      x = BTN_X2;
      label = "PLAY";
      break;
    case BTN_DEL:
      x = BTN_X3;
      label = "DEL.";
      break;
    default: return;
  }
  color = (active || (btn == BTN_PLAY && playMode)) ? BLUE : GRAY;
  TFT.fillRect(x, y, w, h, color);
  TFT.drawRect(x, y, w, h, WHITE);
  TFT.setTextColor(WHITE);
  TFT.setTextSize(1);
  TFT.setCursor(x + 3, y + 4);
  TFT.print(label);
}

// ==== DRAW TOUCH ONLY BUTTON ====
// Draws button to enable/disable touch-only mode
void drawTouchOnlyButton(bool force) {
  static bool prevState = false;
  if (force || prevState != touchOnly) {
    int x = 5, y = 5, w = 24, h = 24;
    TFT.fillRect(x, y, w, h, touchOnly ? BLUE : GRAY);
    TFT.drawRect(x, y, w, h, WHITE);
    TFT.setCursor(x + 4, y + 8);
    TFT.setTextColor(WHITE);
    TFT.setTextSize(1);
    TFT.print("T");
    prevState = touchOnly;
  }
}

// ==== DRAW POSITIONS ====
// Prints all saved positions on the right side of the display
void drawPositions() {
  static int lastNumPositions = -1;
  static int lastPos[MAX_POSITIONS][4];
  int startY = 0;
  int posH = 9;
  int posBoxH = posH * MAX_POSITIONS + 20;

  // Only updates display when something changes
  if (numPositions != lastNumPositions || memcmp(savedPositions, lastPos, sizeof(lastPos)) != 0) {
    TFT.fillRect(160, startY, 160, posBoxH, BLACK);
    TFT.setCursor(170, startY);
    TFT.setTextColor(WHITE);
    TFT.setTextSize(1);
    TFT.print("SAVED POSITIONS:");
    for (int p = 0; p < numPositions; p++) {
      TFT.setCursor(170, startY + posH + p * posH);
      for (int s = 0; s < 4; s++) {
        TFT.printf("%3d", savedPositions[p][s]);
        if (s < 3) TFT.print(", ");
      }
    }
    memcpy(lastPos, savedPositions, sizeof(lastPos));
    lastNumPositions = numPositions;
  }
}

// ==== ADS1115 INTERRUPT AND READING ====
// This function sets the ready flag when a new reading is available (interrupt from ADS)
void adsReady() {
  RDY = true;
}

// This function fetches data from ADS1115 and moves to the next channel
void handleADSConversion() {
  if (RDY) {
    adsVals[adsChannel] = ADS.getValue();  // Save read value for the current channel
    adsChannel++;
    if (adsChannel >= 4) adsChannel = 0;  // Return to channel 0 after channel 3
    ADS.readADC(adsChannel);              // Start reading next channel
    RDY = false;
  }
}
