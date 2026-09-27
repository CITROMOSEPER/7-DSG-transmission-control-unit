// 7DSG support 
// ESP32
// Nemes Dániel


#include "PINS.h"
#include <Wire.h>
#include <U8g2lib.h>
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // For SSD1306
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);    // For SH1106


// Physical Constants
const float AIR_DENSITY = 1.225f;   // kg/m^3
const float GRAVITY     = 9.81f;    // m/s^2


// Vehicle Parameters (Adjust to match your car)
struct VehicleParams {
  float mass;            // kg
  float Cd;              // drag coefficient
  float frontalArea;     // m^2
  float rollingCoeff;    // rolling resistance coefficient
  float wheelRadius;     // meters
  float maxMotorForce;   // N
  float maxBrakeForce;   // N
  float regenStrength;   // fraction of brake that is regen
  float stallRPM;        // where the IC engine stalls
  float redlineRPM;      // where the IC engine redlines
};
VehicleParams car = {
  1800.0f,   // mass (kg)
  0.24f,     // Cd
  2.2f,      // frontal area (m^2)
  0.012f,    // rolling resistance coefficient
  0.33f,     // wheel radius (m)
  4000.0f,   // max motor force (N)
  12000.0f,  // max braking force (N)
  0.4f,      // regen proportion
  700.0f,    // stall RPM
  8000.0f    // redline RPM
};


// Simulation State
struct VehicleState {
  float velocity;     // m/s
  float rpm;          // revolutions/minute
  float position;     // meters
  float acceleration; // m/s^2
};
VehicleState state = {0.0f, 0.0f, 0.0f, 0.0f};


// Inputs (0.0 - 1.0)
float throttle = 0.0f;
float brake    = 0.0f;
float clutch1  = 0.0f;
float clutch2  = 0.0f;


// Road grade in radians (0 = flat)
float roadGrade = 0.0f;


// Timing
const float dt = 0.02f;   // 20 ms physics timestep
unsigned long lastStep = 0;
unsigned long lastTime = 0;

// 1-4    2-32
// GEAR SPEED RANGES
uint8_t SELECTED_GEAR = 0;
//float gearMin[8] =     {0,  0,  10, 30, 50,  70,  90,  130};
//float gearMax[8] =     {30, 30, 60, 80, 120, 140, 200, 300};

float gearMin[8] =     {0,  0,  10,  30,  50,  70,  90,  110};
float gearMax[8] =     {30, 50, 100, 150, 200, 250, 300, 350};

bool engagedGears[8] = {0,  0,  0,  0,  0,   0,   0,   0};
float gearForceMult[8] =
{
  10.0f,   // reverse
  10.0f,   // 1
  5.0f,    // 2
  3.75f,   // 3
  2.5f,    // 4
  2.14f,   // 5
  1.5f,    // 6
  1.0f     // 7
};
int activeGear = 1;
int lastActiveGear = 1;


// Other input states
bool KICKDOWN = 0;
bool ERROR = 0;

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



// Physics Functions -----------------------------------------------------------------------------------------------------------------------------
float calcDrag(float v)
{
  // F_drag = 0.5 * rho * Cd * A * v^2
  return 0.5f * AIR_DENSITY * car.Cd * car.frontalArea * v * v;
}

float calcRollingResistance()
{
  // F_roll = Crr * m * g
  return car.rollingCoeff * car.mass * GRAVITY;
}

float calcGradeForce()
{
  // F_grade = m * g * sin(theta)
  return car.mass * GRAVITY * sin(roadGrade);
}

float torqueCurve(float rpm)
{
  float x = rpm / car.redlineRPM;
  return 1.2f * sin(x * PI);  // example
}

float calcMotorForce(float velocity)
{
  const float idleRPM = 750.0f;
  const float stallRPM = car.stallRPM;
  const float redlineRPM = car.redlineRPM;

  // Determine active gear
  int gearOdd  = 1;
  int gearEven = 1;
  for (int i = 0; i < 8; i++)
  {
    if (engagedGears[i])
    {
      if (i % 2 == 1) gearOdd = i;
      else            gearEven = i;
    }
  }
  lastActiveGear = activeGear;
  float clutchEngagement = 0.0f;
  if (clutch1 < 0.05f && clutch2 < 0.05f)
  {
    activeGear = lastActiveGear;
    clutchEngagement = clutch1;
  }
  else if (clutch1 > clutch2)
  {
    activeGear = gearOdd;
    clutchEngagement = clutch1;
  }
  else
  {
    activeGear = gearEven;
    clutchEngagement = clutch2;
  }

  // Launch control
  if (SELECTED_GEAR >= 3 && state.velocity == 0.0f && throttle > 0.1f && brake > 0.5f && !ERROR)
  {
    state.rpm += (6000 - state.rpm) * 0.1f;
    return 0.0f;
  }

  // Free rev behaviour
  float freeRevRPM = idleRPM + throttle * (redlineRPM - idleRPM);
  if (activeGear < 0 || clutchEngagement < 0.05f || SELECTED_GEAR == 0 || SELECTED_GEAR == 2)
  {
    state.rpm += (freeRevRPM - state.rpm) * 0.1f;
    if (state.rpm > 7500.0f && !KICKDOWN)  state.rpm = 7000.0f;
    return 0.0f;
  }

  // Map speed <-> RPM based on gear
  float vMin = gearMin[activeGear] / 3.6f; // km/h --> m/s
  float vMax = gearMax[activeGear] / 3.6f;

  float coupledRPM = stallRPM + (velocity - vMin) * (redlineRPM - stallRPM) / (vMax - vMin);

  // Slip blending
  float slipFactor = 1.0f - clutchEngagement;

  float targetRPM = slipFactor * freeRevRPM + clutchEngagement * coupledRPM;

  // Idle drop when loading engine
  if (velocity < 1.0f && clutchEngagement > 0.1f)  targetRPM -= clutchEngagement * 50.0f;

  // Smooth RPM
  state.rpm += (targetRPM - state.rpm) * 0.15f;
  if (state.rpm > 7500.0f && !KICKDOWN && activeGear == 1)  state.rpm = 7000.0f;

  // Clamp RPM
  if (state.rpm < stallRPM) state.rpm = stallRPM;
  if (state.rpm > redlineRPM) state.rpm = redlineRPM;


  // Torque reduction near redline
  float rpmNorm = (state.rpm - stallRPM) / (redlineRPM - stallRPM);
  float torqueFactor = 1.0f;
  if (rpmNorm > 0.85f)  torqueFactor = 1.0f - (rpmNorm - 0.85f) * 6.0f;
  if (torqueFactor < 0.0f) torqueFactor = 0.0f;

  if (velocity > 83.33f)  return 0.0f;   // 83.33m/s = 300km/h limiter
  if (brake > 0.1f)  return 0.0f;   // if braking stop delivering power

  if (state.velocity >= vMax)  return 0.0f;   // with slipped clutch you cant go faster than with closed clutch
  float mForce = car.maxMotorForce;
  if (ERROR)  mForce = 2000.0f;
  float engineForce = throttle * torqueFactor * (mForce + KICKDOWN * 800) * gearForceMult[activeGear] * clutchEngagement;
  engineForce *= torqueCurve(state.rpm);
  return engineForce;
}

float calcBrakeForce(float brakeInput, float velocity)
{
  // regen works only when moving forward
  float regen = brakeInput * car.regenStrength * car.maxBrakeForce;
  float friction = brakeInput * (1.0f - car.regenStrength) * car.maxBrakeForce;

  if (velocity <= 0.1f) regen = 0.0f;

  return regen + friction;
}
// -----------------------------------------------------------------------------------------------------------------------------------------------



// I/O handling ----------------------------------------------------------------------------------------------------------------------------------
void updateInputs()
{
  throttle = analogRead(THROTTLE_POSITION) / 4095.0f;
  if (ERROR && throttle > 0.3f)  throttle = 0.3f;
  brake    = analogRead(BRAKE_POSITION)    / 4095.0f;
  clutch1 = analogRead(CLUTCH_ONE) / 4095.0f; // odd gears
  clutch2 = analogRead(CLUTCH_TWO) / 4095.0f; // even + reverse

  engagedGears[0] = digitalRead(REVERSE_GEAR);
  engagedGears[1] = digitalRead(FIRST_GEAR);
  engagedGears[2] = digitalRead(SECOND_GEAR);
  engagedGears[3] = digitalRead(THIRD_GEAR);
  engagedGears[4] = digitalRead(FOURTH_GEAR);
  engagedGears[5] = digitalRead(FIFTH_GEAR);
  engagedGears[6] = digitalRead(SIXTH_GEAR);
  engagedGears[7] = digitalRead(SEVENTH_GEAR);

  if ( digitalRead(P_LEVER) && !digitalRead(R_LEVER) && !digitalRead(N_LEVER) && !digitalRead(D_LEVER) && !digitalRead(M_LEVER))  SELECTED_GEAR = 0;
  if (!digitalRead(P_LEVER) &&  digitalRead(R_LEVER) && !digitalRead(N_LEVER) && !digitalRead(D_LEVER) && !digitalRead(M_LEVER))  SELECTED_GEAR = 1;
  if (!digitalRead(P_LEVER) && !digitalRead(R_LEVER) &&  digitalRead(N_LEVER) && !digitalRead(D_LEVER) && !digitalRead(M_LEVER))  SELECTED_GEAR = 2;
  if (!digitalRead(P_LEVER) && !digitalRead(R_LEVER) && !digitalRead(N_LEVER) &&  digitalRead(D_LEVER) && !digitalRead(M_LEVER))  SELECTED_GEAR = 3;
  if (!digitalRead(P_LEVER) && !digitalRead(R_LEVER) && !digitalRead(N_LEVER) && !digitalRead(D_LEVER) &&  digitalRead(M_LEVER))  SELECTED_GEAR = 4;

  KICKDOWN = digitalRead(KICKDOWN_PIN);
  ERROR = digitalRead(ERROR_PIN);

  //Serial.print("throttle: ");
  //Serial.println(throttle);
  //Serial.print("brake: ");
  //Serial.println(brake);
  //Serial.print("clutch1: ");
  //Serial.println(clutch1);
  //Serial.print("clutch2: ");
  //Serial.println(clutch2);
  //Serial.print("kickdown: ");
  //Serial.println(KICKDOWN);
  //Serial.print("error: ");
  //Serial.println(ERROR);
  //Serial.print("selected drive mode: ");
  //Serial.println(SELECTED_GEAR);
  //
  //Serial.print("engaged gears: ");
  //for (int i = 0; i < 8; i++)
  //{
  //  Serial.print(engagedGears[i]);
  //}
  //Serial.println();
}

void updateOutputs()
{
  float scaledSpeed = state.velocity * 255.0f / 83.33f;
  float scaledRPM = state.rpm * 255.0f / 8000.0;

  dacWrite(VEHICLE_SPEED, constrain(round(scaledSpeed), 0, 255));
  dacWrite(INPUT_RPM, constrain(round(scaledRPM), 0, 255));

  ledcWrite(CLUTCH_ONE_DISPLAY, round(clutch1 * 255.0f));
  ledcWrite(CLUTCH_TWO_DISPLAY, round(clutch2 * 255.0f));

  //Serial.print("speed: ");
  //Serial.println(constrain(round(scaledSpeed), 0, 255));
  //Serial.print("RPM: ");
  //Serial.println(constrain(round(scaledRPM), 0, 255));
  //Serial.print("clutch1 feedback: ");
  //Serial.println(round(clutch1 * 255.0f));
  //Serial.print("clutch1 feedback: ");
  //Serial.println(round(clutch2 * 255.0f));
}
// -----------------------------------------------------------------------------------------------------------------------------------------------



// Main Physics Step
void updateVehicle()
{
  float v = state.velocity;

  float F_motor = calcMotorForce(v);
  float F_drag  = calcDrag(v);
  float F_roll  = calcRollingResistance();
  float F_grade = calcGradeForce();
  float F_brake = calcBrakeForce(brake, v);

  // Net force (forward positive)
  float F_total = F_motor - F_drag - F_roll - F_grade - F_brake;

  // Newton's Second Law
  state.acceleration = F_total / car.mass;

  // Integrate velocity
  state.velocity += state.acceleration * dt;

  // Prevent reverse motion for this simple model
  if (state.velocity < 0) state.velocity = 0;

  // Integrate position
  state.position += state.velocity * dt;
}



// Screen handling (core 0)-----------------------------------------------------------------------------------------------------------------------
void RENDER_MENU(void *pvParameters)
{
  u8g2.clearBuffer();           // Clear the internal memory


  // Circle gauge
  u8g2.drawCircle(64, 32, 29);
  u8g2.drawCircle(64, 32, 30);
  u8g2.setDrawColor(0);   // black (clear pixels)
  u8g2.drawBox(36, 48, 59, 16);   // erase this area
  u8g2.setDrawColor(1);   // back to normal drawing


  // Drive mode display
  String gears[5] = {"P", "R", "N", "D", "M"};
  String gear = gears[SELECTED_GEAR];
  u8g2.setFont(u8g2_font_logisoso16_tr);
  u8g2.setCursor(0, 18);
  u8g2.print(gear);


  // ERROR and KICKDOWN texts
  bool flasher = (millis() / 500) % 2 == 1;
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setCursor(11, 10);
  if (KICKDOWN && flasher)  u8g2.print("BOOST");
  u8g2.setCursor(11, 20);
  if (ERROR && flasher)  u8g2.print("ERR");


  // Gear indicator
  // Determine available gears
  bool availableGears[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 1; i < 8; i++)
  {
    availableGears[i] = (state.velocity * 3.6f >= gearMin[i] && state.velocity * 3.6f <= gearMax[i]);
  }

  if (SELECTED_GEAR >= 3)
  {
    u8g2.setFont(u8g2_font_logisoso16_tr);
    u8g2.setCursor(94, 18);
    //u8g2.print("3");
    if (availableGears[activeGear - 1])  u8g2.print(String(activeGear - 1));

    u8g2.drawBox(105, 1, 12, 18);
    u8g2.setCursor(106, 18);
    u8g2.setDrawColor(0);   // black (clear pixels)
    u8g2.print(String(activeGear));
    u8g2.setDrawColor(1);   // back to normal drawing

    u8g2.setCursor(118, 18);
    //u8g2.print("3");
    if (availableGears[activeGear + 1])  u8g2.print(String(activeGear + 1));
  }
  else if (SELECTED_GEAR == 1)
  {
    u8g2.setFont(u8g2_font_logisoso16_tr);
    u8g2.drawBox(105, 1, 12, 18);
    u8g2.setCursor(106, 18);
    u8g2.setDrawColor(0);   // black (clear pixels)
    u8g2.print("R");
    u8g2.setDrawColor(1);   // back to normal drawing
  }


  // Gauge needle
  int needleRange = 240;
  float currentNeedlePosition =            radians(map(round(state.rpm), 700, 8000, 0, needleRange) - 0  - 210);
  float currentNeedlePositionMinusNinety = radians(map(round(state.rpm), 700, 8000, 0, needleRange) - 90 - 210);
  float currentNeedlePositionPlusNinety =  radians(map(round(state.rpm), 700, 8000, 0, needleRange) + 90 - 210);
  u8g2.drawLine(64 + cos(currentNeedlePositionMinusNinety) * 3, 32 + sin(currentNeedlePositionMinusNinety) * 3, 64 + cos(currentNeedlePositionPlusNinety) * 3, 32 + sin(currentNeedlePositionPlusNinety) * 3);       // small part at bottom (0 long in x, 6 long in y)
  u8g2.drawLine(64 + cos(currentNeedlePositionMinusNinety) * 3, 32 + sin(currentNeedlePositionMinusNinety) * 3, 64 + cos(currentNeedlePosition) * 29, 32 + sin(currentNeedlePosition) * 29);       // left long (29 long in x, 3 long in y)
  u8g2.drawLine(64 + cos(currentNeedlePositionPlusNinety) * 3,  32 + sin(currentNeedlePositionPlusNinety) * 3,  64 + cos(currentNeedlePosition) * 29, 32 + sin(currentNeedlePosition) * 29);       // right long (29 long in x, 3 long in y)


  // RPM display
  u8g2.setFont(u8g2_font_6x10_tr);
  int st = round(state.rpm);
  String rpmString = String(st);
  if      (rpmString.length() == 1)  u8g2.setCursor(60, 20);
  else if (rpmString.length() == 2)  u8g2.setCursor(59, 20);
  else if (rpmString.length() == 3)  u8g2.setCursor(56, 20);
  else if (rpmString.length() == 4)  u8g2.setCursor(53, 20);
  else                               u8g2.setCursor(53, 20);
  u8g2.setDrawColor(2);   // XOR
  u8g2.print(rpmString);
  u8g2.setDrawColor(1);   // back to normal drawing


  // Speed display
  u8g2.setFont(u8g2_font_logisoso16_tr);
  st = round(state.velocity * 3.6f);
  String speedString = String(st);
  if      (speedString.length() == 1)  u8g2.setCursor(59, 62);
  else if (speedString.length() == 2)  u8g2.setCursor(53, 62);
  else if (speedString.length() == 3)  u8g2.setCursor(47, 62);
  else                                 u8g2.setCursor(47, 62);
  u8g2.print(speedString);


  // Pedal and clutch positions
  int mappedClutchOne = map(round(clutch1  * 100.0f), 0, 100, 0, 40) + 1;
  int mappedClutchTwo = map(round(clutch2  * 100.0f), 0, 100, 0, 40) + 1;
  int mappedThrottle =  map(round(throttle * 100.0f), 0, 100, 0, 40) + 1;
  int mappedBrake =     map(round(brake    * 100.0f), 0, 100, 0, 40) + 1;
  u8g2.drawBox(0,   64 - mappedClutchOne, 11, mappedClutchOne);
  u8g2.drawBox(15,  64 - mappedClutchTwo, 11, mappedClutchTwo);
  u8g2.drawBox(117, 64 - mappedThrottle,  11, mappedThrottle);
  u8g2.drawBox(102, 64 - mappedBrake,     11, mappedBrake);

  u8g2.sendBuffer();            // Send buffer content to the display
}

void CORE_0_TASK(void*parameter)
{
  while (true)
  {
    RENDER_MENU(NULL);
    //vTaskDelay(pdMS_TO_TICKS(50));
  }
}
// -----------------------------------------------------------------------------------------------------------------------------------------------



// -----------------------------------------------------------------------------------------------------------------------------------------------
void setup()
{
  //Serial.begin(115200);

  pinMode(P_LEVER, INPUT);
  pinMode(R_LEVER, INPUT);
  pinMode(N_LEVER, INPUT);
  pinMode(D_LEVER, INPUT);
  pinMode(M_LEVER, INPUT);

  pinMode(KICKDOWN_PIN, INPUT);
  pinMode(ERROR_PIN, INPUT);

  pinMode(INPUT_RPM, OUTPUT);
  pinMode(VEHICLE_SPEED, OUTPUT);

  pinMode(FIRST_GEAR, INPUT);
  pinMode(SECOND_GEAR, INPUT);
  pinMode(THIRD_GEAR, INPUT);
  pinMode(FOURTH_GEAR, INPUT);
  pinMode(FIFTH_GEAR, INPUT);
  pinMode(SIXTH_GEAR, INPUT);
  pinMode(SEVENTH_GEAR, INPUT);
  pinMode(REVERSE_GEAR, INPUT);

  ledcAttachChannel(CLUTCH_ONE_DISPLAY, 1000, 8, 1);
  ledcAttachChannel(CLUTCH_TWO_DISPLAY, 1000, 8, 2);


  Wire.begin(SDA, SCK);
  u8g2.begin();
  u8g2.clearDisplay();
  
  // FreeRTOS task to render dashboard
  xTaskCreatePinnedToCore(
    CORE_0_TASK,            // Function to run on the core
    "CORE_0_TASK",          // Name of the task
    70000,                  // Stack size (bytes)  //Have to be careful with this otherwise memory leak
    NULL,                   // Task parameter
    0,                      // Task priority
    NULL,                   // Task handle
    0                       // Core to run the task on (Core 0)
  );
}


void loop() {

  if (millis() - lastStep >= (dt * 1000)) {

    lastStep = millis();

    updateInputs();
    updateVehicle();
    updateOutputs();
  }
}
