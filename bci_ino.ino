#include <ax12.h>
#include "poses.h"
#include "robot.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <TimerOne.h>

const uint8_t EMG_PIN = A0;
float emgEnv = 0.0f;

int   emgRest  = 0;
float restAbs  = 0.0f;
float thrClose = 0.0f;
float thrOpen  = 0.0f;

bool gripperClosed = false;
bool emgAllowOpen  = true;
bool emgDebug      = false;
bool emgEnabled    = true;

unsigned long aboveSince = 0;
unsigned long belowSince = 0;

unsigned long gripperLockoutUntil = 0;
const unsigned long GRIPPER_LOCKOUT = 700;

unsigned long emgIgnoreUntil = 0;
volatile bool g_emgSampleDue = false;

void ISR_EMG()
{
  g_emgSampleDue = true;
}

void MenuOptions();
void TestAllJoints(void);
int8_t MoveSpecificJoint(void);

void Implementation(void);                // Hybrid Route 1
void Implementation_Route2(void);         // Hybrid Route 2

void Implementation_Part2(void);          // Direct Route 1
void Implementation_Part2_Route2(void);   // Direct Route 2

void RunHybridRoute(bool altRoute, const char* label);
void RunDirectRoute(bool altRoute, const char* label);

void EMG_Init();
void EMG_Calibrate();
void EMG();
void WaitEMG(unsigned long ms);
bool GripperClosedTimeout(unsigned long timeoutMs);
bool GripperOpenTimeout(unsigned long timeoutMs);

void SetRobotBusy(bool busy);
void RestorePostRouteState();

void setup()
{
  ROBOT_Init();

  Serial.begin(115200);
  delay(200);
  Serial.println("###########################");
  Serial.println("Serial Communication Established.");
  Serial.println("###########################");
  delay(300);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  emgIgnoreUntil = millis() + 1500;

  EMG_Init();

  MenuOptions();
}

void loop()
{
  EMG();

  if (Serial.available() > 0)
  {
    int inByte = Serial.read();

    switch (inByte)
    {
      case '0':
        SERVOS_ServosOff();
        break;

      case '1':
        SERVOS_ServosOn();
        break;

      case '2':
      {
        double tmpJointAngles[DOFs];
        ROBOT_GetJointsPos(tmpJointAngles);
        Serial.print("Base: ");     Serial.println(tmpJointAngles[0]);
        Serial.print("Shoulder: "); Serial.println(tmpJointAngles[1]);
        Serial.print("Elbow: ");    Serial.println(tmpJointAngles[2]);
        Serial.print("Wrist: ");    Serial.println(tmpJointAngles[3]);
        Serial.print("WristRot: "); Serial.println(tmpJointAngles[4]);
      }
      break;

      case '3':
        ROBOT_GripperClose();
        gripperClosed = true;
        gripperLockoutUntil = millis() + GRIPPER_LOCKOUT;
        break;

      case '4':
        ROBOT_GripperOpen();
        gripperClosed = false;
        gripperLockoutUntil = millis() + GRIPPER_LOCKOUT;
        break;

      case '5':
        TestAllJoints();
        break;

      case '6':
        MoveSpecificJoint();
        break;

      case 'A':
        Implementation();
        break;

      case 'H':
        Implementation_Route2();
        break;

      case 'C':
        EMG_Calibrate();
        break;

      case 'D':
        Implementation_Part2();
        break;

      case 'G':
        Implementation_Part2_Route2();
        break;

      case 'B':
        emgDebug = !emgDebug;
        Serial.print("EMG Debug = ");
        Serial.println(emgDebug ? "ON" : "OFF");
        break;
    }

    MenuOptions();
  }
}

void MenuOptions()
{
  Serial.println("###########################");
  Serial.println("0) Relax Servos (torque OFF)  [ojo gravedad]");
  Serial.println("1) Hold Servos  (torque ON)");
  Serial.println("2) Get Joints Pos");
  Serial.println("3) Gripper Close (manual)");
  Serial.println("4) Gripper Open  (manual)");
  Serial.println("5) TestAllJoints");
  Serial.println("6) MoveSpecificJoint");
  Serial.println("A) Hybrid Route 1 (EMG-assisted)");
  Serial.println("H) Hybrid Route 2 (EMG-assisted, alternate path)");
  Serial.println("B) Plot EMG");
  Serial.println("C) Recalibrate EMG");
  Serial.println("D) Direct Route 1 (EEG-only validation)");
  Serial.println("G) Direct Route 2 (EEG-only validation, alternate path)");
  Serial.println("###########################");
}

void TestAllJoints(void)
{
  double fIncQ      = 0.01;
  uint16_t unSteps  = 100;
  uint8_t i, j;

  ROBOT_SetSingleTrajectory(m_fCoordTest, 1200, LINEAR);
  WaitEMG(1500);

  for (i = 0; i < 5; i++)
  {
    for (j = 0; j < unSteps; j++)
    {
      ROBOT_SetJointPos(i, m_fCoordTest[i] + (double)j * fIncQ);
      WaitEMG(25);
    }

    for (j = unSteps; j > 0; j--)
    {
      ROBOT_SetJointPos(i, m_fCoordTest[i] + (double)j * fIncQ);
      WaitEMG(25);
    }
  }

  ROBOT_GripperClose(); gripperClosed = true;
  WaitEMG(600);
  ROBOT_GripperOpen();  gripperClosed = false;
  WaitEMG(600);

  ROBOT_SetSingleTrajectory(m_fCoordRelax, 3000, CUBIC1);
  WaitEMG(3300);
}

int8_t MoveSpecificJoint(void)
{
  Serial.println("Base: 0");
  Serial.println("Shoulder: 1");
  Serial.println("Elbow: 2");
  Serial.println("Wrist: 3");
  Serial.println("Wrist Rot: 4");
  Serial.println("");
  Serial.println("Joint to move?: ");

  while (Serial.available() <= 0) { EMG(); }
  uint8_t unJoint = Serial.parseInt();

  Serial.print("Joint selected: ");
  Serial.println(unJoint);
  Serial.println("");

  Serial.print("Position to move?: ");
  while (Serial.available() <= 0) { EMG(); }
  float fPosition = Serial.parseFloat();

  Serial.print("Moving Joint: ");
  Serial.print(unJoint);
  Serial.print(" to position: ");
  Serial.println(fPosition);

  ROBOT_SetJointPos(unJoint, fPosition);
  return 0;
}

// =========================================================
// ROUTE WRAPPERS
// =========================================================
void Implementation(void)
{
  RunHybridRoute(false, "Route 1");
}

void Implementation_Route2(void)
{
  RunHybridRoute(true, "Route 2");
}

void Implementation_Part2(void)
{
  RunDirectRoute(false, "Route 1");
}

void Implementation_Part2_Route2(void)
{
  RunDirectRoute(true, "Route 2");
}

// =========================================================
// BUSY / POST-ROUTE HELPERS
// =========================================================
void SetRobotBusy(bool busy)
{
  Serial.print("ROBOT_BUSY=");
  Serial.println(busy ? 1 : 0);
}

void RestorePostRouteState()
{
  emgEnabled = true;
  emgAllowOpen = true;
  emgIgnoreUntil = millis() + 1500;
  gripperLockoutUntil = millis() + GRIPPER_LOCKOUT;
  aboveSince = 0;
  belowSince = 0;
}

// =========================================================
// HYBRID ROUTES (EEG starts route, EMG controls gripper)
// =========================================================
void RunHybridRoute(bool altRoute, const char* label)
{
  double q_start[]   = {0.0,  2.89, -2.89,  0.0,  0.0};
  double q_grasp[]   = {0.37, 1.86, -2.63,  0.67, 1.48};
  double q_via_1[]   = {0.75, 2.01, -1.38, -0.63, 1.57};
  double q_via_2[]   = {1.10, 1.55, -1.20, -1.10, 2.30};  // alternate path
  double q_release[] = {1.54, 1.18, -0.98, -1.77, 3.11};

  double* q_via = altRoute ? q_via_2 : q_via_1;

  Serial.print("=== Hybrid ");
  Serial.print(label);
  Serial.println(" ===");
  SetRobotBusy(true);

  SERVOS_ServosOn();

  emgEnabled   = true;
  emgAllowOpen = false;

  ROBOT_GripperOpen();
  gripperClosed = false;
  gripperLockoutUntil = millis() + GRIPPER_LOCKOUT;
  WaitEMG(700);

  ROBOT_SetSingleTrajectory(q_start, 1500, CUBIC1);
  WaitEMG(2000);

  ROBOT_SetSingleTrajectory(q_grasp, 2500, CUBIC1);
  WaitEMG(2800);

  // allow EMG close at grasp point
  emgAllowOpen = true;
  if (!GripperClosedTimeout(10000))
  {
    Serial.println("Hybrid timeout waiting for EMG CLOSE.");
    ROBOT_SetSingleTrajectory(m_fCoordRelax, 3000, CUBIC1);
    WaitEMG(3300);
    RestorePostRouteState();
    SetRobotBusy(false);
    return;
  }

  // while transporting, do not allow opening
  emgAllowOpen = false;

  ROBOT_SetDoubleTrajectory(q_via, q_release, 2000, 3500, CUBIC2);
  WaitEMG(6000);

  // allow EMG open at release point
  emgAllowOpen = true;
  if (!GripperOpenTimeout(10000))
  {
    Serial.println("Hybrid timeout waiting for EMG OPEN.");
    ROBOT_SetSingleTrajectory(m_fCoordRelax, 3000, CUBIC1);
    WaitEMG(3300);
    RestorePostRouteState();
    SetRobotBusy(false);
    return;
  }

  ROBOT_SetSingleTrajectory(m_fCoordRelax, 3000, CUBIC1);
  WaitEMG(3300);

  RestorePostRouteState();
  Serial.print("=== Hybrid ");
  Serial.print(label);
  Serial.println(" finished ===");
  SetRobotBusy(false);
}

// =========================================================
// DIRECT ROUTES (EEG-only validation; no EMG action)
// =========================================================
void RunDirectRoute(bool altRoute, const char* label)
{
  double q_start[]   = {0.0,  2.89, -2.89,  0.0,  0.0};
  double q_grasp[]   = {0.37, 1.86, -2.63,  0.67, 1.48};
  double q_via_1[]   = {0.75, 2.01, -1.38, -0.63, 1.57};
  double q_via_2[]   = {1.10, 1.55, -1.20, -1.10, 2.30};  // alternate path
  double q_release[] = {1.54, 1.18, -0.98, -1.77, 3.11};

  double* q_via = altRoute ? q_via_2 : q_via_1;

  Serial.print("=== Direct ");
  Serial.print(label);
  Serial.println(" ===");
  SetRobotBusy(true);

  // disable EMG-triggered gripper actions during direct sequence
  emgEnabled   = false;
  emgAllowOpen = false;

  SERVOS_ServosOn();

  ROBOT_GripperOpen();
  gripperClosed = false;
  delay(700);

  ROBOT_SetSingleTrajectory(q_start, 1500, CUBIC1);
  delay(1800);

  ROBOT_SetSingleTrajectory(q_grasp, 2500, CUBIC1);
  delay(2800);

  ROBOT_GripperClose();
  gripperClosed = true;
  delay(800);

  ROBOT_SetDoubleTrajectory(q_via, q_release, 2000, 3500, CUBIC2);
  delay(5900);

  ROBOT_GripperOpen();
  gripperClosed = false;
  delay(800);

  ROBOT_SetSingleTrajectory(m_fCoordRelax, 3000, CUBIC1);
  delay(3300);

  RestorePostRouteState();
  Serial.print("=== Direct ");
  Serial.print(label);
  Serial.println(" finished ===");
  SetRobotBusy(false);
}

// =========================================================
// EMG
// =========================================================
void EMG_Init()
{
  EMG_Calibrate();

  Timer1.initialize(5000);      // 5000 us = 200 Hz
  Timer1.attachInterrupt(ISR_EMG);
}

void EMG_Calibrate()
{
  Serial.println("=== EMG: Calibration ===");
  Serial.println("RELAX...");
  delay(1200);

  const int N1 = 400;
  long sum = 0;
  for (int i = 0; i < N1; i++) { sum += analogRead(EMG_PIN); delay(5); }
  emgRest = (int)(sum / N1);

  long sumAbs = 0;
  for (int i = 0; i < N1; i++) {
    int v = analogRead(EMG_PIN);
    sumAbs += abs(v - emgRest);
    delay(5);
  }
  restAbs = (float)sumAbs / N1;

  Serial.print("Rest mean="); Serial.print(emgRest);
  Serial.print("  restAbs="); Serial.println(restAbs);

  Serial.println("CONTRACT...");
  delay(900);

  const int N2 = 400;
  long sumAbsFlex = 0;
  for (int i = 0; i < N2; i++) {
    int v = analogRead(EMG_PIN);
    sumAbsFlex += abs(v - emgRest);
    delay(5);
  }
  float flexAbs = (float)sumAbsFlex / N2;

  float range = flexAbs - restAbs;
  if (range < 10) range = 10;

  thrClose = restAbs + 0.45f * range;
  thrOpen  = restAbs + 0.25f * range;

  Serial.print("flexAbs=");  Serial.println(flexAbs);
  Serial.print("thrClose="); Serial.println(thrClose);
  Serial.print("thrOpen=");  Serial.println(thrOpen);

  float ratio = (restAbs > 1.0f) ? (flexAbs / restAbs) : 999.0f;
  float diff  = flexAbs - restAbs;

  Serial.println("=== EMG results  ===");
  Serial.print("ratio flex/rest = "); Serial.println(ratio, 2);
  Serial.print("diff  flex-rest = "); Serial.println(diff, 2);

  emgEnv = 0.0f;
  aboveSince = belowSince = 0;
  gripperLockoutUntil = millis() + GRIPPER_LOCKOUT;
  emgIgnoreUntil = millis() + 1500;
}

void EMG()
{
  if (!emgEnabled) return;
  unsigned long now = millis();
  if (now < emgIgnoreUntil) return;

  if (!g_emgSampleDue) return;

  noInterrupts();
  g_emgSampleDue = false;
  interrupts();

  long sumSq = 0;
  const int windowSize = 8;
  for (int i = 0; i < windowSize; i++) {
    int val = analogRead(EMG_PIN) - emgRest;
    sumSq += (long)val * val;
  }

  float currentRMS = sqrt(sumSq / windowSize);
  emgEnv = emgEnv + 0.05f * (currentRMS - emgEnv);

  digitalWrite(LED_BUILTIN, (emgEnv > thrClose) ? HIGH : LOW);

  if (now < gripperLockoutUntil) return;

  if (!gripperClosed)
  {
    if (emgEnv > thrClose)
    {
      if (aboveSince == 0) aboveSince = now;
      if (now - aboveSince >= 100)
      {
        ROBOT_GripperClose();
        gripperClosed = true;
        aboveSince = belowSince = 0;
        gripperLockoutUntil = now + GRIPPER_LOCKOUT;
      }
    }
    else {
      aboveSince = 0;
    }
  }
  else
  {
    if (!emgAllowOpen) {
      belowSince = 0;
      return;
    }

    if (emgEnv < thrOpen)
    {
      if (belowSince == 0) belowSince = now;
      if (now - belowSince >= 180)
      {
        ROBOT_GripperOpen();
        gripperClosed = false;
        aboveSince = belowSince = 0;
        gripperLockoutUntil = now + GRIPPER_LOCKOUT;
      }
    }
    else {
      belowSince = 0;
    }
  }

  if (emgDebug) {
    static unsigned long lastPrint = 0;
    if (now - lastPrint > 25) {
      lastPrint = now;
      Serial.print("Env_RMS:");      Serial.print(emgEnv);
      Serial.print(",Thr_Close:");   Serial.print(thrClose);
      Serial.print(",Thr_Open:");    Serial.println(thrOpen);
    }
  }
}

bool GripperClosedTimeout(unsigned long timeoutMs)
{
  unsigned long t0 = millis();

  while (!gripperClosed)
  {
    EMG();
    if (millis() - t0 >= timeoutMs) return false;
  }
  return true;
}

bool GripperOpenTimeout(unsigned long timeoutMs)
{
  unsigned long t0 = millis();

  while (gripperClosed)
  {
    EMG();
    if (millis() - t0 >= timeoutMs) return false;
  }
  return true;
}

void WaitEMG(unsigned long ms)
{
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    EMG();
  }
}