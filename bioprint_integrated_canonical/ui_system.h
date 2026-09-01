#ifndef UI_SYSTEM_H
#define UI_SYSTEM_H

// ==================== DISPLAY & TOUCH INCLUDES ====================
#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino_GigaDisplayTouch.h>

// External display object (declared in main .ino)
extern GigaDisplay_GFX display;
extern Arduino_GigaDisplayTouch touchDetector;

// Color constants are #define macros from config.h — no extern declarations needed.

// External state variables (declared in main .ino)
extern Page currentPage;
extern float selectedTemp, selectedVol1, selectedVol2, selectedConc, selectedSpeed;
extern float tempSelection;
extern bool motorsHomed;
extern bool systemReady;
extern bool heatControlEnabled;
extern bool isPrinting;
extern bool syringesTempReached;
extern bool calibrationComplete;
extern unsigned long retractionStartTime;
extern unsigned long shutdownStartTime;
extern unsigned long tempStableTime;
extern SystemConfig config;
extern float extrusionVolume;
extern float printTime;
extern float currentDisplayTemp;
extern float Input_Syringe;
extern float Input_HeatMat;
extern float Output_HeatMat;
extern float Output_Syringe;
extern float Setpoint_Syringe;
extern float currentTemperatures[4];
extern long arduino_pos1, arduino_pos2;
extern float cycleStartDispensed1, cycleStartDispensed2;
extern float cycleTargetVol1, cycleTargetVol2;
extern SystemState current_state;

// Font constants (from GigaDisplay library)
extern const GFXfont FreeSansBold18pt7b;
extern const GFXfont FreeSansBold12pt7b;
extern const GFXfont FreeSans9pt7b;

// Functions declared in main file
extern bool homeMotors();
extern long pendingTargetPos1;
extern long pendingTargetPos2;

extern PendingMove pendingMove;

// ==================== PAGE DRAWING FUNCTIONS ====================
// Forward declarations for helpers called before their definitions in this file
void drawHomeButton(int x, int y, const char* label, float value, const char* unit);
void drawParameterSummary(int x, int y, const char* label, float value, const char* unit);
void getCurrentPageInfo(const char** title, const char** unit);


// label2 is optional — omit it (or pass nullptr) for a single centered line,
// or pass it to stack label/label2 as a two-line centered block.
void drawActionButton(int x, int y, int width, int height, int radius, uint16_t buttonColor, const char* label, const GFXfont* font, uint16_t textColor = BUTTON_TEXT_COLOR, const char* label2 = nullptr){
  display.fillRoundRect(x, y, width, height, radius, buttonColor);
  display.setFont(font);
  display.setTextColor(textColor);

  //Gets the text offset to allow for centering of text in button
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(label, 0, 0, &x1, &y1, &w, &h);

  if (label2 == nullptr) {
    int textX = x + (width - w) / 2 - x1;
    int textY = y + (height - h) / 2 - y1;
    display.setCursor(textX, textY);
    display.print(label);
    return;
  }

  int16_t x1b, y1b;
  uint16_t w2, h2;
  display.getTextBounds(label2, 0, 0, &x1b, &y1b, &w2, &h2);

  int lineSpacing = 10;
  int blockHeight = h + lineSpacing + h2;
  int blockTopY = y + (height - blockHeight) / 2;

  int text1X = x + (width - w) / 2 - x1;
  int text1Y = blockTopY - y1;
  display.setCursor(text1X, text1Y);
  display.print(label);

  int text2X = x + (width - w2) / 2 - x1b;
  int text2Y = blockTopY + h + lineSpacing - y1b;
  display.setCursor(text2X, text2Y);
  display.print(label2);
}

void drawMotorZeroCheckPage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(40, 150);
  display.print("MOTOR SETUP");
  
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(60, 250);
  display.print("Are the motors at");
  display.setCursor(80, 285);
  display.print("zero position?");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 350);
  display.print("Zero = fully retracted");
  display.setCursor(50, 380);
  display.print("(no load on syringes)");
  
  // YES button - large green
  drawActionButton(50, 480, 180, 120, 10,CONFIRM_COLOR, "YES", &FreeSansBold18pt7b);
  
  // NO button - large red
  drawActionButton(250, 480, 180, 120, 10, CANCEL_COLOR, "NO", &FreeSansBold18pt7b);
  
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(50, 650);
  display.print("If NO: Motors will retract");
  display.setCursor(50, 680);
  display.print("slowly for 15 seconds");
  display.endBuffering();
}

void drawRetractingToZeroPage() {
  display.startBuffering();
  unsigned long elapsed = millis() - retractionStartTime;
  unsigned long remaining = RETRACTION_DURATION - elapsed;
  float progress = (float)elapsed / (float)RETRACTION_DURATION * 100.0;
  
  if (progress > 100.0) progress = 100.0;
  
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(40, 150);
  display.print("RETRACTING");
  display.setCursor(90, 190);
  display.print("TO ZERO");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(80, 280);
  display.print("Moving motors slowly...");
  display.setCursor(100, 310);
  display.print("Please wait");
  
  // Progress bar
  display.drawRect(50, 380, 380, 40, TEXT_COLOR);
  int fillWidth = (int)(370.0 * progress / 100.0);
  display.fillRect(55, 385, fillWidth, 30, CONFIRM_COLOR);
  
  // Time remaining
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(140, 470);
  display.print(remaining / 1000);
  display.print(" sec");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(120, 530);
  display.print("Progress: ");
  display.print((int)progress);
  display.print("%");
  
  display.setCursor(50, 630);
  display.print("Slow speed: 0.5 mm/s");
  display.setCursor(50, 660);
  display.print("Safe retraction in progress");
  display.endBuffering();
}

void drawCalibrationInProgressPage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(60, 200);
  display.print("CALIBRATING");
  display.setCursor(130, 240);
  display.print("RANGE");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(80, 320);
  display.print("Establishing 0-15000");
  display.setCursor(100, 350);
  display.print("reference range");
  
  display.setCursor(50, 430);
  display.print("Step 1: Moving to MIN (0)");
  display.setCursor(50, 470);
  display.print("Step 2: Moving to MAX (15000)");
  
  display.setCursor(100, 560);
  display.print("Please wait...");
  
  display.setCursor(80, 650);
  display.print("Current positions:");
  display.setCursor(80, 680);
  display.print("M1: ");
  display.print(arduino_pos1);
  display.print("  M2: ");
  display.print(arduino_pos2);
  display.endBuffering();
}

void drawShutdownConfirmPage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(CANCEL_COLOR);
  
  display.setCursor(80, 150);
  display.print("SHUTDOWN?");
  
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(60, 250);
  display.print("This will:");
  display.setCursor(60, 285);
  display.print("1. Turn off heaters");
  display.setCursor(60, 320);
  display.print("2. Retract motors to zero");
  display.setCursor(60, 355);
  display.print("3. Prepare for power off");
  
  display.setCursor(60, 420);
  display.print("Continue with shutdown?");
  
  // YES button (red, left)
  drawActionButton(50, 520, 160, 100, 10, CANCEL_COLOR, "YES", &FreeSansBold18pt7b);
  
  // NO button (green, right)
  drawActionButton(270, 520, 160, 100, 10, CONFIRM_COLOR, "NO", &FreeSansBold18pt7b);

  display.endBuffering();
}

void drawShuttingDownPage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(60, 150);
  display.print("SHUTTING");
  display.setCursor(120, 200);
  display.print("DOWN");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(80, 300);
  display.print("Heaters: OFF");
  
  display.setCursor(80, 350);
  display.print("Retracting motors...");
  
  display.setCursor(80, 430);
  display.print("Current positions:");
  display.setCursor(80, 465);
  display.print("M1: ");
  display.print(arduino_pos1);
  display.print(" steps");
  display.setCursor(80, 500);
  display.print("M2: ");
  display.print(arduino_pos2);
  display.print(" steps");
  
  display.setCursor(80, 570);
  display.print("Target: 0 steps (zero)");
  
  display.setCursor(60, 650);
  display.print("Please wait - do not power off");
  display.endBuffering();
}

void drawShutdownCompletePage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(CONFIRM_COLOR);
  
  display.setCursor(70, 200);
  display.print("SHUTDOWN");
  display.setCursor(80, 250);
  display.print("COMPLETE");
  
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(80, 350);
  display.print("Motors: Retracted to zero");
  display.setCursor(80, 385);
  display.print("Heaters: OFF");
  display.setCursor(80, 420);
  display.print("System: Safe");
  
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(60, 520);
  display.print("Safe to power off Arduino");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(80, 600);
  display.print("You may now disconnect power");
  display.endBuffering();
}

void drawWelcomePage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(80, 200);
  display.print("BioPrint AM");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(120, 300);
  display.print("v3.1 Integrated");
  
  display.setCursor(70, 400);
  display.print("Temperature Control");
  
  display.setCursor(50, 450);
  display.print("Syringe Range:");
  display.setCursor(50, 475);
  display.print("0mL=1600  10mL=11600");
  
  display.setCursor(80, 520);
  display.print("Heat Mats:");
  display.setCursor(80, 545);
  display.print("A0: ");
  display.print(currentTemperatures[0], 1);
  display.print(" C  A1: ");
  display.print(currentTemperatures[1], 1);
  display.print(" C");
  
  display.setCursor(80, 560);
  if (motorsHomed) {
    display.setTextColor(CONFIRM_COLOR);
    display.print("Motors: HOMED");
  } else {
    display.setTextColor(CANCEL_COLOR);
    display.print("Motors: NOT HOMED!");
  }
  
  drawActionButton(140, 630, 200, 100, 10, BUTTON_COLOR, "START", &FreeSansBold18pt7b);
  
  display.endBuffering();
}

void drawHomingPage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(100, 200);
  display.print("HOMING...");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 280);
  display.print("1. Moving to load position");
  display.setCursor(65, 310);
  display.print("(15000 steps)");
  
  display.setCursor(50, 360);
  display.print("2. Moving to volume position");
  if (selectedVol1 > 0 && selectedVol2 > 0) {
    long target1 = 1600 + mlToSteps(selectedVol1);
    long target2 = 1600 + mlToSteps(selectedVol2);
    display.setCursor(65, 390);
    display.print("M1: ");
    display.print(selectedVol1, 1);
    display.print("mL -> pos ");
    display.print(target1);
    display.setCursor(65, 420);
    display.print("M2: ");
    display.print(selectedVol2, 1);
    display.print("mL -> pos ");
    display.print(target2);
  } else {
    display.setCursor(65, 390);
    display.print("(10mL = position 11600)");
  }
  
  display.setCursor(80, 480);
  display.print("Please wait...");
  
  display.setCursor(80, 540);
  display.print("M1: ");
  display.print(arduino_pos1);
  display.print(" steps");
  display.setCursor(80, 570);
  display.print("M2: ");
  display.print(arduino_pos2);
  display.print(" steps");
  display.endBuffering();
}

void drawLoadingSyringesPage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(40, 150);
  display.print("LOAD SYRINGES");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 230);
  display.print("Instructions:");
  
  display.setCursor(50, 270);
  display.print("1. Press BEGIN HOMING");
  display.setCursor(50, 300);
  display.print("2. Wait for motors (15000)");
  display.setCursor(50, 330);
  display.print("3. Insert & fill syringes");
  display.setCursor(50, 360);
  display.print("4. Secure in holders");
  display.setCursor(50, 390);
  display.print("5. Motors move to volume");
  
  display.setCursor(50, 450);
  display.print("Target volumes:");
  if (selectedVol1 > 0 && selectedVol2 > 0) {
    display.setCursor(50, 480);
    display.print("M1: ");
    display.print(selectedVol1, 1);
    display.print("mL");
    display.setCursor(50, 510);
    display.print("M2: ");
    display.print(selectedVol2, 1);
    display.print("mL");
  } else {
    display.setCursor(50, 480);
    display.print("M1: 10.0 mL (max)");
    display.setCursor(50, 510);
    display.print("M2: 10.0 mL (max)");
  }
  
  drawActionButton(90, 580, 300, 80, 10, CONFIRM_COLOR, "BEGIN", &FreeSansBold18pt7b, BUTTON_TEXT_COLOR, "HOMING");

  drawActionButton(40, 700, 120, 70, 10, CANCEL_COLOR, "BACK", &FreeSans9pt7b);

  drawActionButton(320, 700, 120, 70, 10, CLEAR_COLOR, "HOME", &FreeSans9pt7b);
  
  display.endBuffering();
}

void drawWaitingForSyringesPage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(40, 150);
  display.print("LOAD SYRINGES");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 230);
  display.setTextColor(CONFIRM_COLOR);
  display.print("Motors at loading position!");
  display.setTextColor(TEXT_COLOR);
  display.setCursor(50, 260);
  display.print("(15000 steps)");
  
  display.setCursor(50, 330);
  display.print("Now:");
  display.setCursor(50, 365);
  display.print("1. Insert syringes");
  display.setCursor(50, 395);
  display.print("2. Fill with material");
  display.setCursor(50, 425);
  display.print("3. Secure in holders");
  
  display.setCursor(50, 490);
  display.print("Target volumes:");
  if (selectedVol1 > 0 && selectedVol2 > 0) {
    long target1 = 1600 + mlToSteps(selectedVol1);
    long target2 = 1600 + mlToSteps(selectedVol2);
    display.setCursor(50, 520);
    display.print("M1: ");
    display.print(selectedVol1, 1);
    display.print("mL (pos ");
    display.print(target1);
    display.print(")");
    display.setCursor(50, 550);
    display.print("M2: ");
    display.print(selectedVol2, 1);
    display.print("mL (pos ");
    display.print(target2);
    display.print(")");
  } else {
    display.setCursor(50, 520);
    display.print("M1: 10.0 mL (pos 11600)");
    display.setCursor(50, 550);
    display.print("M2: 10.0 mL (pos 11600)");
  }
  
  drawActionButton(90, 600, 300, 80, 10, CONFIRM_COLOR, "SYRINGES", &FreeSansBold18pt7b, BUTTON_TEXT_COLOR, "LOADED");
  
  drawActionButton(170, 710, 140, 60, 10, CLEAR_COLOR, "HOME", &FreeSans9pt7b);
  display.endBuffering();
}

void drawHomePage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setTextColor(TEXT_COLOR);
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(35, 50);
  display.print("PARAMETER CONTROL");
  
  drawHomeButton(40, 120, "Temperature", selectedTemp, "C");
  drawHomeButton(40, 230, "Volume 1", selectedVol1, "mL");
  drawHomeButton(40, 340, "Volume 2", selectedVol2, "mL");
  drawHomeButton(40, 450, "Concentration", selectedConc, "%");
  
  // CONTINUE button (left)
  drawActionButton(40, 600, 180, 80, 10, CONFIRM_COLOR, "CONTINUE", &FreeSansBold18pt7b, BUTTON_TEXT_COLOR);
  
  // SHUTDOWN button (right, red)
  drawActionButton(260, 600, 180, 80, 10, CANCEL_COLOR, "SHUTDOWN", &FreeSansBold18pt7b, BUTTON_TEXT_COLOR);
  
  display.endBuffering();
}

void drawHomeButton(int x, int y, const char* label, float value, const char* unit) {
  display.fillRoundRect(x, y, 400, 90, 10, BUTTON_COLOR);
  
  display.setFont(&FreeSansBold12pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(x + 15, y + 30);
  display.print(label);
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(x + 15, y + 65);
  if (value < 0) {
    display.print("Not Selected");
  } else {
    display.print("Selected: ");
    
    if (unit[0] == '%') {
      float inverse = 100 - value;
      display.print(value, 1);
      display.print(":");
      display.print(inverse, 1);
      display.print("%");
    } else {
      display.print(value, 1);
      display.print(" ");
      if (unit[0] == 'C') {
        display.write(248);
      }
      display.print(unit);
    }
  }
}

void drawParameterPage(int* options, int numOptions, float currentValue, const char* title, const char* unit) {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setTextColor(TEXT_COLOR);
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(50, 40);
  display.print("Select ");
  display.print(title);
  display.print(":");
  
  int boxX = 40;
  int boxY = 75;
  int boxWidth = 260;
  int boxHeight = 100;
  int buttonSize = 70;
  int buttonSpacing = 10;
  
  drawActionButton(boxX, boxY, buttonSize, boxHeight, 10, CONFIRM_COLOR, "-", &FreeSansBold18pt7b, BUTTON_TEXT_COLOR);

  display.fillRoundRect(boxX + buttonSize + buttonSpacing, boxY, boxWidth, boxHeight, 10, BUTTON_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(BUTTON_TEXT_COLOR);
  
  if (tempSelection < 0) {
    display.setCursor(boxX + buttonSize + buttonSpacing + 45, boxY + 62);
    display.print("Not Set");
  } else {
    String valueStr = "";

    if (unit[0] == '%') {
      float inverse = 100 - tempSelection;
      valueStr = String(tempSelection, 0) + ":" + String(inverse, 0) + "%";
    } else if (unit[0] == 'C') {
      valueStr = String(tempSelection, 1) + " " + String((char)248) + unit;
    } else {
      valueStr = String(tempSelection, 1) + " " + unit;
    }

    //Gets the text offset to allow for centering of text in button
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(valueStr, 0, 0, &x1, &y1, &w, &h);

    int valueBoxX = boxX + buttonSize + buttonSpacing;
    int textX = valueBoxX + (boxWidth - w) / 2 - x1;
    int textY = boxY + (boxHeight - h) / 2 - y1;
    display.setCursor(textX, textY);
    display.print(valueStr);
  }
  
  drawActionButton(boxX + buttonSize + buttonSpacing + boxWidth + buttonSpacing, boxY, buttonSize, boxHeight, 10, CONFIRM_COLOR, "+", &FreeSansBold18pt7b, BUTTON_TEXT_COLOR);
  
  int cols = 3;
  int buttonWidth = 120;
  int buttonHeight = 80;
  int spacingX = 20;
  int spacingY = 20;
  int startX = 40;
  int startY = 200;
  
  for (int i = 0; i < numOptions; i++) {
    int row = i / cols;
    int col = i % cols;
    int bx = startX + col * (buttonWidth + spacingX);
    int by = startY + row * (buttonHeight + spacingY);
    
    bool isSelected = (abs(options[i] - tempSelection) < 0.01);
    uint16_t bgColor = isSelected ? BUTTON_SELECTED_COLOR : BUTTON_COLOR;
    
    display.fillRoundRect(bx, by, buttonWidth, buttonHeight, 10, bgColor);
    display.setFont(&FreeSansBold18pt7b);
    display.setTextColor(BUTTON_TEXT_COLOR);
    
    String valueStr = String(options[i]);
    int charCount = valueStr.length();
    //Gets the text offset to allow for centering of text in button
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(label, 0, 0, &x1, &y1, &w, &h);
    
    display.setCursor(x1, y1);
    display.print(options[i]);
  }
  
  drawActionButton(40, 700, 120, 70, 10, CANCEL_COLOR, "BACK", &FreeSansBold12pt7b);
  drawActionButton(180, 700, 120, 70, 10, CLEAR_COLOR, "HOME", &FreeSansBold12pt7b);
  drawActionButton(320, 700, 120, 70, 10, CONFIRM_COLOR, "CONFIRM", &FreeSansBold12pt7b);

  display.endBuffering();
}

void drawPrintConfirmPage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setTextColor(TEXT_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setCursor(80, 50);
  display.print("Begin Print?");
  
  int yPos = 130;
  int lineHeight = 90;
  
  drawParameterSummary(40, yPos, "Temperature:", selectedTemp, "C");
  yPos += lineHeight;
  
  drawParameterSummary(40, yPos, "Volume 1:", selectedVol1, "mL");
  yPos += lineHeight;
  
  drawParameterSummary(40, yPos, "Volume 2:", selectedVol2, "mL");
  yPos += lineHeight;
  
  drawParameterSummary(40, yPos, "Concentration:", selectedConc, "%");
  yPos += lineHeight;
  
  drawParameterSummary(40, yPos, "Speed:", selectedSpeed, "mm/s");
  
  drawActionButton(140, 630, 200, 70, 10, CONFIRM_COLOR, "CONFIRM", &FreeSansBold12pt7b, BUTTON_TEXT_COLOR);

  drawActionButton(40, 720, 120, 50, 10, CANCEL_COLOR, "BACK", &FreeSans9pt7b, BUTTON_TEXT_COLOR);

  drawActionButton(320, 720, 120, 50, 10, CLEAR_COLOR, "HOME", &FreeSans9pt7b, BUTTON_TEXT_COLOR);

  display.endBuffering();
}

void drawParameterSummary(int x, int y, const char* label, float value, const char* unit) {
  display.setFont(&FreeSansBold12pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(x, y);
  display.print(label);
  
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(x, y + 40);
  if (value < 0) {
    display.print("Not Selected");
  } else {
    if (unit[0] == '%') {
      float inverse = 100 - value;
      display.print(value, 1);
      display.print(":");
      display.print(inverse, 1);
      display.print("%");
    } else {
      display.print(value, 1);
      display.print(" ");
      if (unit[0] == 'C') {
        display.write(248);
      }
      display.print(unit);
    }
  }
}

void drawLoadingPage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(80, 150);
  display.print("LOADING...");
  
  display.setFont(&FreeSans9pt7b);
  
  display.setCursor(50, 200);
  display.print("Syringe Temperature:");
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(50, 235);
  if (currentDisplayTemp > -999.0) {
    display.print(currentDisplayTemp, 1);
    display.print(" C / ");
    display.print(Setpoint_Syringe, 0);
    display.print(" C");
  } else {
    display.print("--- C");
  }
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 280);
  display.print("Heat Mat Temperature:");
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(50, 315);
  if (Input_HeatMat > -999.0) {
    display.print(Input_HeatMat, 1);
    display.print(" C / 80 C");
  } else {
    display.print("--- C");
  }
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 370);
  display.print("PWM Values:");
  display.setCursor(50, 400);
  display.print("Heat Mat PID: ");
  display.print((Output_HeatMat / 255.0) * 100.0, 1);
  display.print("%");
  display.setCursor(50, 430);
  display.print("Syringe PID: ");
  display.print((Output_Syringe / 255.0) * 100.0, 1);
  display.print("%");
  
  display.setCursor(50, 470);
  display.setTextColor(CONFIRM_COLOR);
  if (!syringesTempReached) {
    display.print("Phase 1: Heating to 80C");
  } else {
    display.print("Phase 2: Modulating");
  }
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(50, 520);
  display.print("A0:");
  display.print(currentTemperatures[0], 1);
  display.print(" A1:");
  display.print(currentTemperatures[1], 1);
  
  display.setCursor(50, 550);
  display.print("A2:");
  display.print(currentTemperatures[2], 1);
  display.print(" A3:");
  display.print(currentTemperatures[3], 1);
  
  drawActionButton(140, 680, 200, 80, 10, CANCEL_COLOR, "HOME", &FreeSans9pt7b, BUTTON_TEXT_COLOR);

  display.endBuffering();
}

void drawTempReadyPage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(80, 120);
  display.print("READY?");
  
  display.setFont(&FreeSans9pt7b);
  
  display.setCursor(50, 200);
  display.print("Syringe Temperature:");
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(50, 240);
  if (currentDisplayTemp > -999.0) {
    display.print(currentDisplayTemp, 1);
    display.print(" C / ");
    display.print(Setpoint_Syringe, 0);
    display.print(" C");
  } else {
    display.print("--- C");
  }
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 290);
  display.print("Heat Mat: ");
  if (Input_HeatMat > -999.0) {
    display.print(Input_HeatMat, 1);
    display.print(" C");
  } else {
    display.print("--- C");
  }
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 350);
  
  if (systemReady) {
    display.setTextColor(CONFIRM_COLOR);
    display.print("Temperature reached!");
    display.setTextColor(TEXT_COLOR);
    display.setCursor(50, 380);
    display.print("Actuators initialized");
    display.setCursor(50, 410);
    display.print("Motors homed");
  } else {
    display.print("Waiting for:");
    int yPos = 380;
    if (abs(currentDisplayTemp - Setpoint_Syringe) > TEMP_TOLERANCE) {
      display.setCursor(50, yPos);
      display.print("- Syringe temperature");
      yPos += 30;
    }
    if (current_state != PRIMED) {
      display.setCursor(50, yPos);
      display.print("- Actuator initialization");
      yPos += 30;
    }
    if (!motorsHomed) {
      display.setCursor(50, yPos);
      display.setTextColor(CANCEL_COLOR);
      display.print("- MOTOR HOMING REQUIRED!");
      display.setTextColor(TEXT_COLOR);
      display.setCursor(50, yPos + 25);
      display.print("  (Press HOME on welcome page)");
    }
  }
  
  display.setCursor(50, 510);
  display.print("A0=");
  display.print(currentTemperatures[0], 1);
  display.print(" A1=");
  display.print(currentTemperatures[1], 1);
  
  display.setCursor(50, 540);
  display.print("A2=");
  display.print(currentTemperatures[2], 1);
  display.print(" A3=");
  display.print(currentTemperatures[3], 1);
  
  if (systemReady) {
    drawActionButton(90, 600, 300, 80, 10, CONFIRM_COLOR, "YES", &FreeSansBold18pt7b, BUTTON_TEXT_COLOR);
  }
  
  drawActionButton(140, 700, 200, 60, 10, CANCEL_COLOR, "HOME", &FreeSans9pt7b, BUTTON_TEXT_COLOR);

  display.endBuffering();
}

void drawExtrusionSetupPage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(40, 80);
  display.print("EXTRUSION");
  display.setCursor(100, 120);
  display.print("SETUP");
  
  display.setFont(&FreeSans9pt7b);
  
  // Volume to extrude section
  display.setCursor(50, 180);
  display.print("Volume to Extrude:");
  
  drawActionButton(50, 200, 80, 70, 10, CONFIRM_COLOR, "-", &FreeSansBold18pt7b, BUTTON_TEXT_COLOR);

  display.fillRoundRect(150, 200, 180, 70, 10, BUTTON_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(185, 245);
  display.print(extrusionVolume, 1);
  display.setFont(&FreeSans9pt7b);
  display.print(" mL");

  drawActionButton(350, 200, 80, 70, 10, CONFIRM_COLOR, "+", &FreeSansBold18pt7b, BUTTON_TEXT_COLOR);
  
  // Print Time section
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(50, 310);
  display.print("Print Time:");
  
  drawActionButton(50, 330, 80, 70, 10, CONFIRM_COLOR, "-", &FreeSansBold18pt7b, BUTTON_TEXT_COLOR);

  display.fillRoundRect(150, 330, 180, 70, 10, BUTTON_COLOR);
  display.setTextColor(BUTTON_TEXT_COLOR);
  display.setCursor(185, 375);
  display.print((int)printTime);
  display.setFont(&FreeSans9pt7b);
  display.print(" sec");

  drawActionButton(350, 330, 80, 70, 10, CONFIRM_COLOR, "+", &FreeSansBold18pt7b, BUTTON_TEXT_COLOR);
  
  // Info section
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(50, 440);
  display.print("Remaining:");
  display.setCursor(50, 470);
  display.print("M1: ");
  display.print(config.remaining1, 2);
  display.print(" mL");
  display.setCursor(50, 500);
  display.print("M2: ");
  display.print(config.remaining2, 2);
  display.print(" mL");
  
  // Calculate required volumes
  float reqVol1 = extrusionVolume * config.ratio1;
  float reqVol2 = extrusionVolume * config.ratio2;
  
  display.setCursor(50, 540);
  display.print("Required: M1=");
  display.print(reqVol1, 2);
  display.print(" M2=");
  display.print(reqVol2, 2);
  
  // Buttons
  drawActionButton(90, 600, 300, 80, 10, CONFIRM_COLOR, "START", &FreeSansBold18pt7b, BUTTON_TEXT_COLOR);

  drawActionButton(140, 700, 200, 60, 10, CANCEL_COLOR, "HOME", &FreeSans9pt7b, BUTTON_TEXT_COLOR);
  display.endBuffering();
}

void drawPostExtrusionOptionsPage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(40, 100);
  display.print("EXTRUSION");
  display.setCursor(80, 140);
  display.print("COMPLETE!");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 220);
  display.print("Dispensed this cycle:");
  display.setCursor(50, 250);
  display.print("M1: ");
  float cycle_disp1 = config.dispensed1 - cycleStartDispensed1;
  display.print(cycle_disp1, 2);
  display.print(" mL");
  display.setCursor(50, 280);
  display.print("M2: ");
  float cycle_disp2 = config.dispensed2 - cycleStartDispensed2;
  display.print(cycle_disp2, 2);
  display.print(" mL");
  
  display.setCursor(50, 330);
  display.print("Remaining:");
  display.setCursor(50, 360);
  display.print("M1: ");
  display.print(config.remaining1, 2);
  display.print(" mL");
  display.setCursor(50, 390);
  display.print("M2: ");
  display.print(config.remaining2, 2);
  display.print(" mL");
  
  // Three button options
  drawActionButton(40, 450, 120, 70, 10, CONFIRM_COLOR, "SAME", &FreeSansBold12pt7b, BUTTON_TEXT_COLOR);

  drawActionButton(180, 450, 120, 70, 10, BUTTON_COLOR, "ADJUST", &FreeSansBold12pt7b, BUTTON_TEXT_COLOR);

  drawActionButton(320, 450, 120, 70, 10, CLEAR_COLOR, "FINISH", &FreeSansBold12pt7b, BUTTON_TEXT_COLOR);

  drawActionButton(140, 600, 200, 70, 10, CANCEL_COLOR, "HOME", &FreeSans9pt7b, BUTTON_TEXT_COLOR);
  display.endBuffering();
}

void drawReadyToPrintPage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(60, 100);
  display.print("READY!");
  
  display.setFont(&FreeSans9pt7b);
  
  display.setCursor(50, 180);
  display.print("Temperature: ");
  display.print(Input_Syringe, 1);
  display.print(" / ");
  display.print(Setpoint_Syringe, 0);
  display.print(" C");
  
  display.setCursor(50, 230);
  display.print("Remaining:");
  display.setCursor(50, 260);
  display.print("M1: ");
  display.print(config.remaining1, 2);
  display.print(" mL");
  display.setCursor(50, 290);
  display.print("M2: ");
  display.print(config.remaining2, 2);
  display.print(" mL");
  
  display.setCursor(50, 340);
  display.print("Dispensed:");
  display.setCursor(50, 370);
  display.print("M1: ");
  display.print(config.dispensed1, 2);
  display.print(" mL");
  display.setCursor(50, 400);
  display.print("M2: ");
  display.print(config.dispensed2, 2);
  display.print(" mL");
  
  drawActionButton(90, 500, 300, 120, 10, CONFIRM_COLOR, "PRINT", &FreeSansBold18pt7b, BUTTON_TEXT_COLOR);

  drawActionButton(140, 670, 200, 70, 10, CANCEL_COLOR, "HOME", &FreeSans9pt7b);
  display.endBuffering();
}

void drawPrintingPage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(80, 100);
  display.print("PRINTING");
  
  display.setFont(&FreeSans9pt7b);
  
  display.setCursor(50, 160);
  display.print("Current Temperature:");
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(50, 195);
  if (currentDisplayTemp > -999.0) {
    display.print(currentDisplayTemp, 1);
    display.print(" C");
  } else {
    display.print("--- C");
  }
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 235);
  display.print("Selected Temperature:");
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(50, 270);
  display.print(Setpoint_Syringe, 1);
  display.print(" C");
  
  display.setFont(&FreeSans9pt7b);
  display.setCursor(50, 320);
  display.print("Remaining:");
  display.setCursor(50, 350);
  display.print("M1: ");
  display.print(config.remaining1, 2);
  display.print(" mL");
  display.setCursor(50, 380);
  display.print("M2: ");
  display.print(config.remaining2, 2);
  display.print(" mL");
  
  display.setCursor(50, 430);
  display.print("This Cycle:");
  display.setCursor(50, 460);
  display.print("M1: ");
  float cycle_disp1 = config.dispensed1 - cycleStartDispensed1;
  display.print(cycle_disp1, 2);
  display.print(" / ");
  display.print(cycleTargetVol1, 2);
  display.print(" mL");
  display.setCursor(50, 490);
  display.print("M2: ");
  float cycle_disp2 = config.dispensed2 - cycleStartDispensed2;
  display.print(cycle_disp2, 2);
  display.print(" / ");
  display.print(cycleTargetVol2, 2);
  display.print(" mL");
  
  // Calculate progress based on PRIMARY syringe (whichever has larger target)
  float progress = 0.0;
  if (cycleTargetVol1 >= cycleTargetVol2 && cycleTargetVol1 > 0) {
    progress = (cycle_disp1 / cycleTargetVol1) * 100.0;
  } else if (cycleTargetVol2 > 0) {
    progress = (cycle_disp2 / cycleTargetVol2) * 100.0;
  }
  progress = constrain(progress, 0.0, 100.0);
  
  display.setCursor(50, 530);
  display.print("Cycle Progress: ");
  display.print(progress, 0);
  display.print(" %");
  
  drawActionButton(140, 600, 200, 80, 10, CANCEL_COLOR, "STOP", &FreeSansBold12pt7b, BUTTON_TEXT_COLOR);
  display.endBuffering();
}

void drawPrintDonePage() {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(CONFIRM_COLOR);
  display.setCursor(60, 200);
  display.print("PRINT DONE!");
  
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  
  display.setCursor(80, 300);
  display.print("Total Dispensed:");
  display.setCursor(80, 340);
  display.print("M1: ");
  display.print(config.dispensed1, 2);
  display.print(" mL");
  display.setCursor(80, 370);
  display.print("M2: ");
  display.print(config.dispensed2, 2);
  display.print(" mL");
  
  display.setCursor(80, 430);
  display.print("Final Temperature:");
  display.setCursor(80, 460);
  display.print(currentDisplayTemp, 1);
  display.print(" C");
  
  // FINISH button (return to start)
  drawActionButton(40, 550, 180, 80, 10, CLEAR_COLOR, "FINISH", &FreeSansBold18pt7b, BUTTON_TEXT_COLOR);

  // HOME button
  drawActionButton(260, 550, 180, 80, 10, CONFIRM_COLOR, "HOME", &FreeSansBold18pt7b, BUTTON_TEXT_COLOR);
  display.endBuffering();
}

void drawErrorPage(const char* message) {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(CANCEL_COLOR);
  display.setCursor(100, 200);
  display.print("ERROR");
  
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  display.setCursor(80, 300);
  display.print(message);
  
  drawActionButton(165, 450, 150, 70, 10, BUTTON_COLOR, "HOME", &FreeSans9pt7b, BUTTON_TEXT_COLOR);
  display.endBuffering();
}

void drawValidationErrorPage(String error_msg, String suggestions) {
  display.startBuffering();
  display.fillScreen(BG_COLOR);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(CANCEL_COLOR);
  display.setCursor(50, 100);
  display.print("CANNOT");
  display.setCursor(80, 140);
  display.print("EXTRUDE");
  
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  
  // Parse and display error message (split by newlines)
  int yPos = 220;
  int lineStart = 0;
  for (int i = 0; i <= error_msg.length(); i++) {
    if (i == error_msg.length() || error_msg.charAt(i) == '\n') {
      String line = error_msg.substring(lineStart, i);
      display.setCursor(50, yPos);
      display.print(line);
      yPos += 30;
      lineStart = i + 1;
    }
  }
  
  // Suggestions section
  yPos += 20;
  display.setTextColor(CONFIRM_COLOR);
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(50, yPos);
  display.print("Suggestions:");
  
  yPos += 40;
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(TEXT_COLOR);
  
  // Parse suggestions
  lineStart = 0;
  for (int i = 0; i <= suggestions.length(); i++) {
    if (i == suggestions.length() || suggestions.charAt(i) == '\n') {
      String line = suggestions.substring(lineStart, i);
      display.setCursor(50, yPos);
      display.print(line);
      yPos += 30;
      lineStart = i + 1;
    }
  }
  
  // BACK button
  drawActionButton(140, 680, 200, 70, 10, BUTTON_COLOR, "BACK", &FreeSansBold12pt7b, BUTTON_TEXT_COLOR);
  display.endBuffering();
}

void goToPrintConfirmPage() {
  drawPrintConfirmPage();
  currentPage = PRINT_CONFIRM;
}

//Used when motor movement fails
void goToErrorPage(){
  drawErrorPage(pendingMove.failureMessage.c_str());
  currentPage = ERROR_PAGE;
}

void onCalibrationArrived() {
  motorsHomed = true;
  calibrationComplete = true;
  currentPage = WELCOME;
  drawWelcomePage();
}

void onReturnToLoading(){
  motorsHomed = true;
  currentPage = MOTOR_ZERO_CHECK;
  drawMotorZeroCheckPage();
}

void onShutdown(){
    currentPage = SHUTDOWN_COMPLETE;
    drawShutdownCompletePage();
}

void waitingForSyringes() {
  motorsHomed = true;
  currentPage = WAITING_FOR_SYRINGES;
  drawWaitingForSyringesPage();
}

// ==================== TOUCH HANDLERS ====================

void handleMotorZeroCheckTouch(int x, int y) {
  // YES button
  if (x >= 50 && x <= 230 && y >= 480 && y <= 600) {
    Serial.println("User confirmed motors at zero");
    currentPage = CALIBRATION_IN_PROGRESS;
    drawCalibrationInProgressPage();
    delay(500);  // Show calibration page briefly
    
    pendingMove.target1_phase1 = 0;
    pendingMove.target2_phase1 = 0;
    pendingMove.speed1_phase1 = 1.0;
    pendingMove.speed2_phase1 = 1.0;

    pendingMove.target1_phase2 = LOAD_POSITION;
    pendingMove.target2_phase2 = LOAD_POSITION;
    pendingMove.speed1_phase2 = 2.0;
    pendingMove.speed2_phase2 = 2.0;
    
    pendingMove.phaseCount = 2;
    pendingMove.phaseIndex = 0;
    pendingMove.phaseStarted = false;
    pendingMove.active = true;
    pendingMove.onProgress = drawCalibrationInProgressPage;
    pendingMove.onArrived = onCalibrationArrived;
    pendingMove.onFailed = goToErrorPage;
    pendingMove.failureMessage = "Calibration failed: motors could not confirm position";

    return;
  }
  
  // NO button
  if (x >= 250 && x <= 430 && y >= 480 && y <= 600) {
    Serial.println("User indicated motors NOT at zero - starting retraction");
    currentPage = RETRACTING_TO_ZERO;
    retractionStartTime = millis();
    
    // Start retraction using large negative target position
    // This will move motors backward slowly for 15 seconds
    Serial.println("Starting retraction to position -5000 at 0.5 mm/s");
    
    drawRetractingToZeroPage();
    return;
  }
}

void handleShutdownConfirmTouch(int x, int y) {
  // YES button - proceed with shutdown
  if (x >= 50 && x <= 210 && y >= 520 && y <= 620) {
    Serial.println("=== SHUTDOWN INITIATED ===");
    
    // Turn off heaters immediately
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    Serial.println("Heaters OFF");
    
    // Go to shutting down page
    currentPage = SHUTTING_DOWN;

    pendingMove.target1_phase1 = 0;
    pendingMove.target2_phase1 = 0;
    pendingMove.speed1_phase1 = 0.5;
    pendingMove.speed2_phase1 = 0.5;
    
    pendingMove.phaseCount = 1;
    pendingMove.phaseIndex = 0;
    pendingMove.phaseStarted = false;
    pendingMove.active = true;
    pendingMove.onProgress = drawShuttingDownPage;
    pendingMove.onArrived = onShutdown;
    pendingMove.onFailed = goToErrorPage;
    pendingMove.failureMessage = "Shutdown failed: motors could not confirm position";
    return;
  }
  
  // NO button - cancel shutdown
  if (x >= 270 && x <= 430 && y >= 520 && y <= 620) {
    Serial.println("Shutdown cancelled");
    currentPage = HOME;
    drawHomePage();
    return;
  }
}

void handleWelcomeTouch(int x, int y) {
  if (x >= 140 && x <= 340 && y >= 630 && y <= 730) {
    currentPage = HOME;
    drawHomePage();
  }
}

void handleHomeTouch(int x, int y) {
  if (x >= 40 && x <= 440) {
    if (y >= 120 && y <= 210) {
      currentPage = TEMPERATURE_PAGE;
      tempSelection = selectedTemp;
      drawParameterPage((int*)TEMP_OPTIONS, NUM_TEMP_OPTIONS, tempSelection, "Temperature", "C");
    } else if (y >= 230 && y <= 320) {
      currentPage = VOLUME1;
      tempSelection = selectedVol1;
      drawParameterPage((int*)VOL_OPTIONS, NUM_VOL_OPTIONS, tempSelection, "Volume 1", "mL");
    } else if (y >= 340 && y <= 430) {
      currentPage = VOLUME2;
      tempSelection = selectedVol2;
      drawParameterPage((int*)VOL_OPTIONS, NUM_VOL_OPTIONS, tempSelection, "Volume 2", "mL");
    } else if (y >= 450 && y <= 540) {
      currentPage = CONCENTRATION;
      tempSelection = selectedConc;
      drawParameterPage((int*)CONC_OPTIONS, NUM_CONC_OPTIONS, tempSelection, "Concentration", "%");
    }
  }
  
  // CONTINUE button
  if (x >= 40 && x <= 220 && y >= 600 && y <= 680) {
    if (selectedTemp < 0 || selectedVol1 < 0 || selectedVol2 < 0 || selectedConc < 0) {
      currentPage = ERROR_PAGE;
      drawErrorPage("Please specify all parameters");
    } else {
      currentPage = LOADING_SYRINGES;
      drawLoadingSyringesPage();
    }
  }
  
  // SHUTDOWN button
  if (x >= 260 && x <= 440 && y >= 600 && y <= 680) {
    currentPage = SHUTDOWN_CONFIRM;
    drawShutdownConfirmPage();
  }
}

void handleParameterTouch(int x, int y, int* options, int numOptions, float* targetVar) {
  int boxX = 40;
  int boxY = 75;
  int boxWidth = 260;
  int boxHeight = 100;
  int buttonSize = 70;
  int buttonSpacing = 10;
  
  float step = 1.0f;
  float minVal = 0;
  float maxVal = 100;
  
  if (currentPage == TEMPERATURE_PAGE) {
    step = 1.0f;
    minVal = TEMP_OPTIONS[0];
    maxVal = TEMP_OPTIONS[NUM_TEMP_OPTIONS - 1];
  } else if (currentPage == VOLUME1 || currentPage == VOLUME2) {
    step = 0.1f;
    minVal = 0.1f;
    maxVal = 10.0f;
  } else if (currentPage == CONCENTRATION) {
    step = 5.0f;
    minVal = CONC_OPTIONS[0];
    maxVal = CONC_OPTIONS[NUM_CONC_OPTIONS - 1];
  } else if (currentPage == SPEED) {
    step = 0.1f;
    minVal = SPEED_OPTIONS[0];
    maxVal = SPEED_OPTIONS[NUM_SPEED_OPTIONS - 1];
  }
  
  if (x >= boxX && x <= boxX + buttonSize && y >= boxY && y <= boxY + boxHeight) {
    if (tempSelection >= 0) {
      tempSelection -= step;
      if (tempSelection < minVal) tempSelection = minVal;
      if (step < 1.0f) {
        tempSelection = round(tempSelection * 10.0) / 10.0;
      } else {
        tempSelection = round(tempSelection);
      }
      const char* title;
      const char* unit;
      getCurrentPageInfo(&title, &unit);
      drawParameterPage(options, numOptions, tempSelection, title, unit);
    }
    return;
  }
  
  int plusX = boxX + buttonSize + buttonSpacing + boxWidth + buttonSpacing;
  if (x >= plusX && x <= plusX + buttonSize && y >= boxY && y <= boxY + boxHeight) {
    if (tempSelection >= 0) {
      tempSelection += step;
      if (tempSelection > maxVal) tempSelection = maxVal;
      if (step < 1.0f) {
        tempSelection = round(tempSelection * 10.0) / 10.0;
      } else {
        tempSelection = round(tempSelection);
      }
      const char* title;
      const char* unit;
      getCurrentPageInfo(&title, &unit);
      drawParameterPage(options, numOptions, tempSelection, title, unit);
    } else {
      tempSelection = options[0];
      const char* title;
      const char* unit;
      getCurrentPageInfo(&title, &unit);
      drawParameterPage(options, numOptions, tempSelection, title, unit);
    }
    return;
  }
  
  if (x >= 40 && x <= 160 && y >= 700 && y <= 770) {
    currentPage = HOME;
    drawHomePage();
    return;
  }
  
  if (x >= 180 && x <= 300 && y >= 700 && y <= 770) {
    currentPage = WELCOME;
    drawWelcomePage();
    return;
  }
  
  if (x >= 320 && x <= 440 && y >= 700 && y <= 770) {
    if (currentPage == TEMPERATURE_PAGE && tempSelection > 0) {
      selectedTemp = tempSelection;
      Setpoint_Syringe = selectedTemp;
    } else if (tempSelection >= 0) {
      *targetVar = tempSelection;
    }
    currentPage = HOME;
    drawHomePage();
    return;
  }
  
  int cols = 3;
  int buttonWidth = 120;
  int buttonHeight = 80;
  int spacingX = 20;
  int spacingY = 20;
  int startX = 40;
  int startY = 200;
  
  for (int i = 0; i < numOptions; i++) {
    int col = i % cols;
    int row = i / cols;
    int bx = startX + col * (buttonWidth + spacingX);
    int by = startY + row * (buttonHeight + spacingY);
    
    if (x >= bx && x <= bx + buttonWidth && y >= by && y <= by + buttonHeight) {
      tempSelection = options[i];
      const char* title;
      const char* unit;
      getCurrentPageInfo(&title, &unit);
      drawParameterPage(options, numOptions, tempSelection, title, unit);
      break;
    }
  }
}

void handlePrintConfirmTouch(int x, int y) {
  if (x >= 140 && x <= 340 && y >= 630 && y <= 700) {
    Setpoint_Syringe = selectedTemp;
    heatControlEnabled = true;
    systemReady = false;
    
    currentPage = LOADING_PAGE;
    drawLoadingPage();
    delay(500);
    
    // Skip executeLoad - motors are already positioned from homing!
    current_state = LOAD;  // Just set the state
    
    if (!executeSetup(config, selectedVol1, selectedVol2, selectedConc)) {
      drawErrorPage("Setup failed");
      currentPage = ERROR_PAGE;
      return;
    }
    drawLoadingPage();
    delay(500);

    if (!executePrime(config)) {
      drawErrorPage("Prime failed");
      currentPage = ERROR_PAGE;
      return;
    }
    drawLoadingPage();
    delay(500);
    
    currentPage = TEMP_READY;
    tempStableTime = 0;
    systemReady = false;
    drawTempReadyPage();
  }
  
  if (x >= 40 && x <= 160 && y >= 720 && y <= 770) {
    currentPage = LOADING_SYRINGES;
    drawLoadingSyringesPage();
  }
  
  if (x >= 320 && x <= 440 && y >= 720 && y <= 770) {
    currentPage = WELCOME;
    drawWelcomePage();
  }
}

void handleTempReadyTouch(int x, int y) {
  if (x >= 90 && x <= 390 && y >= 600 && y <= 680 && systemReady) {
    currentPage = EXTRUSION_SETUP;
    drawExtrusionSetupPage();
  }
  
  if (x >= 140 && x <= 340 && y >= 700 && y <= 760) {
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    currentPage = HOME;
    drawHomePage();
  }
}

void handleExtrusionSetupTouch(int x, int y) {
  // Volume controls
  if (x >= 50 && x <= 130 && y >= 200 && y <= 270) {
    // Decrease volume
    extrusionVolume -= 0.1;
    if (extrusionVolume < 0.1) extrusionVolume = 0.1;
    extrusionVolume = round(extrusionVolume * 10.0) / 10.0;
    drawExtrusionSetupPage();
    return;
  }
  
  if (x >= 350 && x <= 430 && y >= 200 && y <= 270) {
    // Increase volume
    extrusionVolume += 0.1;
    if (extrusionVolume > 20.0) extrusionVolume = 20.0;  // Max 20mL
    extrusionVolume = round(extrusionVolume * 10.0) / 10.0;
    drawExtrusionSetupPage();
    return;
  }
  
  // Print Time controls
  if (x >= 50 && x <= 130 && y >= 330 && y <= 400) {
    // Decrease time
    printTime -= 1.0;
    if (printTime < 2.0) printTime = 2.0;
    drawExtrusionSetupPage();
    return;
  }
  
  if (x >= 350 && x <= 430 && y >= 330 && y <= 400) {
    // Increase time
    printTime += 1.0;
    if (printTime > 15.0) printTime = 15.0;
    drawExtrusionSetupPage();
    return;
  }
  
  // START button
  if (x >= 90 && x <= 390 && y >= 600 && y <= 680) {
    // VALIDATE EXTRUSION BEFORE STARTING
    ExtrusionValidation validation = validateExtrusion(config, extrusionVolume, printTime);
    
    if (!validation.is_valid) {
      // Show validation error with suggestions
      Serial.println("=== EXTRUSION VALIDATION FAILED ===");
      Serial.println(validation.error_message);
      Serial.println("Suggestions:");
      Serial.println(validation.suggestion);
      
      drawValidationErrorPage(validation.error_message, validation.suggestion);
      // Stay on same page type (will go back on BACK button)
      return;
    }
    
    // Validation passed - proceed to print
    Serial.println("=== EXTRUSION VALIDATED ===");
    currentPage = READY_TO_PRINT;
    drawReadyToPrintPage();
    return;
  }
  
  // HOME button
  if (x >= 140 && x <= 340 && y >= 700 && y <= 760) {
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    currentPage = HOME;
    drawHomePage();
    return;
  }
}

void handlePostExtrusionOptionsTouch(int x, int y) {
  // SAME EXTRUSION button
  if (x >= 40 && x <= 160 && y >= 450 && y <= 520) {
    // Go back to READY_TO_PRINT with same settings
    currentPage = READY_TO_PRINT;
    drawReadyToPrintPage();
    return;
  }
  
  // ADJUST PARAMETERS button
  if (x >= 180 && x <= 300 && y >= 450 && y <= 520) {
    // Go to EXTRUSION_SETUP to change volume/time
    currentPage = EXTRUSION_SETUP;
    drawExtrusionSetupPage();
    return;
  }
  
  // FINISH button
  if (x >= 320 && x <= 440 && y >= 450 && y <= 520) {
    Serial.println("FINISH pressed - returning motors to load position");
    // Move motors to 15000 (LOAD_POSITION)
      pendingMove.target1_phase1 = LOAD_POSITION;
      pendingMove.target2_phase1 = LOAD_POSITION;
      pendingMove.speed1_phase1 = 3.0;
      pendingMove.speed2_phase1 = 3.0;
    
      pendingMove.phaseCount = 1;
      pendingMove.phaseIndex = 0;
      pendingMove.phaseStarted = false;
      pendingMove.active = true;
      pendingMove.onProgress = drawCalibrationInProgressPage;
      pendingMove.onArrived = onReturnToLoading;
      pendingMove.onFailed = goToErrorPage;
      pendingMove.failureMessage = "Failed to return to load position";
      
      // Turn off heat
      heatControlEnabled = false;
      analogWrite(MOSFET_PIN, 0);
    return;
  }
  
  // HOME button
  if (x >= 140 && x <= 340 && y >= 600 && y <= 670) {
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    currentPage = HOME;
    drawHomePage();
    return;
  }
}

void handleReadyToPrintTouch(int x, int y) {
  if (x >= 90 && x <= 390 && y >= 500 && y <= 620) {
    // Initialize cycle tracking
    cycleStartDispensed1 = config.dispensed1;
    cycleStartDispensed2 = config.dispensed2;
    cycleTargetVol1 = extrusionVolume * config.ratio1;
    cycleTargetVol2 = extrusionVolume * config.ratio2;
    
    isPrinting = true;
    currentPage = PRINTING_PAGE;
    drawPrintingPage();
  }
  
  if (x >= 140 && x <= 340 && y >= 670 && y <= 740) {
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    currentPage = HOME;
    drawHomePage();
  }
}

void handlePrintingTouch(int x, int y) {
  if (x >= 140 && x <= 340 && y >= 600 && y <= 680) {
    isPrinting = false;
    
    tic1.haltAndHold();
    tic2.haltAndHold();
    delay(100);
    
    long stopped_pos1 = tic1.getCurrentPosition();
    long stopped_pos2 = tic2.getCurrentPosition();
    
    arduino_pos1 = stopped_pos1;
    arduino_pos2 = stopped_pos2;
    
    float actualDispensed1 = (config.prime_pos1 - arduino_pos1) / STEPS_PER_ML;
    float actualDispensed2 = (config.prime_pos2 - arduino_pos2) / STEPS_PER_ML;
    
    config.remaining1 = config.syringe_vol1 - actualDispensed1;
    config.remaining2 = config.syringe_vol2 - actualDispensed2;
    config.dispensed1 = actualDispensed1;
    config.dispensed2 = actualDispensed2;
    
    current_state = COMPLETE;
    currentPage = READY_TO_PRINT;
    drawReadyToPrintPage();
  }
}

void handleLoadingTouch(int x, int y) {
  if (x >= 140 && x <= 340 && y >= 680 && y <= 760) {
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    currentPage = HOME;
    drawHomePage();
  }
}

void handlePrintDoneTouch(int x, int y) {
  // FINISH button (left)
  if (x >= 40 && x <= 220 && y >= 550 && y <= 630) {
    Serial.println("FINISH pressed from PRINT_DONE - returning to start");
    // Move motors to LOAD_POSITION
      pendingMove.target1_phase1 = LOAD_POSITION;
      pendingMove.target2_phase1 = LOAD_POSITION;
      pendingMove.speed1_phase1 = 3.0;
      pendingMove.speed2_phase1 = 3.0;
    
      pendingMove.phaseCount = 1;
      pendingMove.phaseIndex = 0;
      pendingMove.phaseStarted = false;
      pendingMove.active = true;
      pendingMove.onProgress = drawCalibrationInProgressPage;
      pendingMove.onArrived = onReturnToLoading;
      pendingMove.onFailed = goToErrorPage;
      pendingMove.failureMessage = "Failed to return to load position";
      
      // Turn off heat
      heatControlEnabled = false;
      analogWrite(MOSFET_PIN, 0);
    return;
  }
  
  // HOME button (right)
  if (x >= 260 && x <= 440 && y >= 550 && y <= 630) {
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    currentPage = HOME;
    drawHomePage();
  }
}

void handleErrorTouch(int x, int y) {
  // Original error page HOME button
  if (x >= 165 && x <= 315 && y >= 450 && y <= 520) {
    heatControlEnabled = false;
    analogWrite(MOSFET_PIN, 0);
    currentPage = HOME;
    drawHomePage();
    return;
  }
  
  // Validation error page BACK button (goes back to extrusion setup)
  if (x >= 140 && x <= 340 && y >= 680 && y <= 750) {
    currentPage = EXTRUSION_SETUP;
    drawExtrusionSetupPage();
    return;
  }
}

void handleLoadingSyringesTouch(int x, int y) {
  if (x >= 90 && x <= 390 && y >= 580 && y <= 660) {    
    pendingMove.target1_phase1 = LOAD_POSITION;
    pendingMove.target2_phase1 = LOAD_POSITION;
    pendingMove.speed1_phase1 = 2.0;
    pendingMove.speed2_phase1 = 2.0;

    pendingMove.phaseCount = 1;
    pendingMove.phaseIndex = 0;
    pendingMove.phaseStarted = false;
    pendingMove.active = true;
    pendingMove.onArrived = waitingForSyringes; 
    pendingMove.onFailed = goToErrorPage;
    pendingMove.onProgress = drawHomingPage;
    pendingMove.failureMessage = "Failed to reach loading position";
    
    pendingMove.onProgress();
    currentPage = HOMING_PAGE; 
    return;
  }
  
  if (x >= 40 && x <= 160 && y >= 700 && y <= 770) {
    currentPage = HOME;
    drawHomePage();
  }
  
  if (x >= 320 && x <= 440 && y >= 700 && y <= 770) {
    currentPage = WELCOME;
    drawWelcomePage();
  }
}

void handleWaitingForSyringesTouch(int x, int y) {
  if (x >= 90 && x <= 390 && y >= 600 && y <= 680) {

    // Move motors to the target volume positions
    long target1 = 1600 + mlToSteps(selectedVol1);
    long target2 = 1600 + mlToSteps(selectedVol2);
    
    pendingMove.target1_phase1 = target1;
    pendingMove.target2_phase1 = target2;
    pendingMove.speed1_phase1 = 2.0;
    pendingMove.speed2_phase1 = 2.0;

    pendingMove.phaseCount = 1;
    pendingMove.phaseIndex = 0;
    pendingMove.phaseStarted = false;
    pendingMove.active = true;
    pendingMove.onArrived = goToPrintConfirmPage;
    pendingMove.onFailed = goToErrorPage;
    pendingMove.onProgress = drawHomingPage;
    pendingMove.failureMessage = "Failed to reach volume position";
    
    pendingMove.onProgress();
    currentPage = HOMING_PAGE;
    return;    
  }
  
  if (x >= 170 && x <= 310 && y >= 710 && y <= 770) {
    currentPage = HOME;
    drawHomePage();
  }
}

void drawHeatIndicator(){
  static bool wasHeatOn = false;
  if (heatControlEnabled) {
    drawActionButton(400, 10, 60, 40, 10, HEAT_ACTIVE_COLOR, "HEAT", &FreeSans9pt7b, BUTTON_TEXT_COLOR);
    wasHeatOn = true;
    return;
  } else if (wasHeatOn){
    display.fillRoundRect(400, 10, 60, 40, 10, BG_COLOR);
    wasHeatOn = false;
    return;
  } 
}

#endif // UI_SYSTEM_H
