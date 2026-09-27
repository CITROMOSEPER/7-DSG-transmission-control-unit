// Internal combustion engine sound generator
// Arduino nano
// Nemes Dániel
// 2025/12/28


#include "PINS.h"
#define MIN_RPM 700
#define MAX_RPM 8000


int RPM = 0;
int THROTTLE = 0;
int BRAKE = 0;
bool ERROR = 0;
bool KICKDOWN = 0;

int  CYLINDERS = 6;
char LAYOUT = 'V';           // 'I' or 'V'
int  BANK_ANGLE_DEG = 180;    // Only used if LAYOUT == 'V'
String CRANK_TYPE = "FLAT";  // FLAT or CROSS

// Firing event list (degrees in 720° cycle)
float firingAngles[16];
int firingCount = 0;
int currentFire = 0;
unsigned long lastFireTime = 0;

bool LAUNCH_MODE = false;
unsigned long lastModulationTime = 0;
bool ignitionEnabled = true;
int launchCutFrequency = 80;  // Hz, controls machine-gun speed
int cutDuty = 50;             // % ignition on time

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



// -----------------------------------------------------------------------------------------------------------------------------------------------
void generateFiringAngles(int cylinders, char layout, int bank_angle_deg, String crank_type)
{
  firingCount = cylinders;

  if (layout == 'I' || crank_type == "FLAT")
  {
    // Even firing (inline or flat-plane)
    float spacing = 720.0 / cylinders;
    for (int i = 0; i < cylinders; i++)
    {
      firingAngles[i] = i * spacing;
    }
  }
  else
  {
    // Cross-plane V engine (simplified V8-style model)
    // Typical cross-plane crank angles
    float baseAngles[4] = {0, 90, 180, 270};

    int idx = 0;
    for (int bank = 0; bank < 2; bank++)
    {
      float bankOffset = (bank == 0) ? 0 : bank_angle_deg;
      for (int i = 0; i < cylinders / 4; i++)
      {
        firingAngles[idx++] = baseAngles[i] * 2 + bankOffset;
      }
    }

    // Normalize to 0–720
    for (int i = 0; i < firingCount; i++)
    {
      while (firingAngles[i] >= 720)  firingAngles[i] -= 720;
    }

    // Sort angles
    for (int i = 0; i < firingCount - 1; i++)
    {
      for (int j = i + 1; j < firingCount; j++)
      {
        if (firingAngles[j] < firingAngles[i])
        {
          float t = firingAngles[i];
          firingAngles[i] = firingAngles[j];
          firingAngles[j] = t;
        }
      }
    }
  }
}


void updateInputs()
{
  RPM = map(analogRead(RPM_INPUT_PIN), 64, 632, MIN_RPM, MAX_RPM);
  THROTTLE = map(analogRead(THROTTLE_INPUT_PIN), 0, 680, 0, 100);
  BRAKE = map(analogRead(BRAKE_INPUT_PIN), 0, 680, 0, 100);
  ERROR = (analogRead(ERROR_INPUT_PIN) > 500);
  KICKDOWN = (analogRead(KICKDOWN_INPUT_PIN) > 500);

  //Serial.println(RPM);
}
// -----------------------------------------------------------------------------------------------------------------------------------------------



// -----------------------------------------------------------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(9600);

  pinMode(SOUND_OUTPUT_PIN, OUTPUT);
}


void loop()
{
  updateInputs();

  if (RPM > 5800 && RPM < 6200 && THROTTLE > 10 && BRAKE > 50)  LAUNCH_MODE = true;
  else                                            LAUNCH_MODE = false;
  if (!LAUNCH_MODE)                               generateFiringAngles(CYLINDERS, LAYOUT, BANK_ANGLE_DEG, CRANK_TYPE);

  if (ERROR)                                      generateFiringAngles(CYLINDERS, LAYOUT, BANK_ANGLE_DEG, "CROSS");

  if (KICKDOWN)  RPM = RPM + 500;


  if (LAUNCH_MODE)
  {
    unsigned long now = micros();
    unsigned long period = 1000000UL / launchCutFrequency;

    if (now - lastModulationTime >= period)
    {
      lastModulationTime = now;

      // Random duty and frequency jitter
      cutDuty = random(30, 70);
      launchCutFrequency = random(60, 120);

      ignitionEnabled = !ignitionEnabled;
    }
  }
  else
  {
    ignitionEnabled = true;
  }

  
  unsigned long now = micros();

  float cycleTimeUs = (60.0 / RPM) * 2.0 * 1000000.0; // 720° cycle
  float currentAngle = firingAngles[currentFire];
  float nextAngle = firingAngles[(currentFire + 1) % firingCount];

  float deltaAngle = nextAngle - currentAngle;
  if (deltaAngle < 0)  deltaAngle += 720;

  unsigned long fireIntervalUs = (deltaAngle / 720.0) * cycleTimeUs;

  if (now - lastFireTime >= fireIntervalUs)
  {
    lastFireTime = now;
    currentFire = (currentFire + 1) % firingCount;

    int freq = map(RPM, MIN_RPM, MAX_RPM, 90, 260);

    if (LAUNCH_MODE)
    {
      // aggressive pitch spikes
      freq += random(-40, 80);

      // turbo burst clusters
      if (random(0, 100) > 70)  freq += 120;
    }

    if (ignitionEnabled)
    {
      tone(SOUND_OUTPUT_PIN, freq);

      // shorter, sharper pulses in launch
      if (LAUNCH_MODE)  delayMicroseconds(random(300, 900));
      else              delayMicroseconds(1800);

      noTone(SOUND_OUTPUT_PIN);
    }
  }
}