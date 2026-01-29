#include <AccelStepper.h>
#include <Bounce2.h>
#include <Encoder.h>
#include <avr/pgmspace.h>
#include <math.h>

#include <map>

// Firmware version
const char* VERSION = "2.1.0";

// Model of the AR4, i.e. mk1, mk2, mk3
String MODEL = "";

///////////////////////////////////////////////////////////////////////////////
// Physical Params
///////////////////////////////////////////////////////////////////////////////

const int NUM_JOINTS = 6;
// User-defined calibration offsets in DEGREES (positive means add angle)
float CAL_OFFSET_DEG[NUM_JOINTS] = { 6.2, -3.8, 0, 0, 0, 0 };

const int ESTOP_PIN = 39;
const int STEP_PINS[] = { 0, 2, 4, 6, 8, 10 };
const int DIR_PINS[] = { 1, 3, 5, 7, 9, 11 };
const int LIMIT_PINS[] = { 26, 27, 28, 29, 30, 31 };

std::map<String, const float*> MOTOR_STEPS_PER_DEG;
const float MOTOR_STEPS_PER_DEG_MK1[] = { 44.44444444, 55.55555556, 55.55555556,
                                          42.72664356, 21.86024888, 22.22222222 };
const float MOTOR_STEPS_PER_DEG_MK2[] = { 44.44444444, 55.55555556, 55.55555556,
                                          49.77777777, 21.86024888, 22.22222222 };
const float MOTOR_STEPS_PER_DEG_MK3[] = { 44.44444444, 55.55555556, 55.55555556,
                                          49.77777777, 21.86024888, 22.22222222 };
const float MOTOR_STEPS_PER_DEG_MK4[] = { 88.888, 111.111, 111.111,
                                          99.555, 43.720, 44.444 };
const float MOTOR_STEPS_PER_DEG_MK5[] = { 88.888, 111.111, 111.111,
                                          99.555, 43.720, 44.444 };



// set encoder pins
Encoder encPos[6] = { Encoder(14, 15), Encoder(17, 16), Encoder(19, 18),
                      Encoder(20, 21), Encoder(23, 22), Encoder(24, 25) };
// +1 if encoder direction matches motor direction, -1 otherwise
int ENC_DIR[] = { -1, 1, 1, 1, 1, 1 };
// +1 if encoder max value is at the minimum joint angle, 0 otherwise
int ENC_MAX_AT_ANGLE_MIN[] = { 1, 0, 1, 0, 0, 1 };
// motor steps * ENC_MULT = encoder steps (4000 steps/rev)
const float ENC_MULT_MK1_3[] = { 10, 10, 10, 10, 5, 10 };
const float ENC_MULT_MK4_5[] = { 5, 5, 5, 5, 2.5, 5 };


// define axis limits in degrees, for calibration
std::map<String, const int*> JOINT_LIMIT_MIN;
int JOINT_LIMIT_MIN_MK1[] = { -170, -42, -89, -165, -105, -155 };
int JOINT_LIMIT_MIN_MK2[] = { -170, -42, -89, -165, -105, -155 };
int JOINT_LIMIT_MIN_MK3[] = { -170, -42, -89, -180, -105, -180 };
int JOINT_LIMIT_MIN_MK4[] = { -170, -42, -89, -180, -105, -180 };
int JOINT_LIMIT_MIN_MK5[] = { -160, -42, -89, -180, -105, -180 };

std::map<String, const int*> JOINT_LIMIT_MAX;
int JOINT_LIMIT_MAX_MK1[] = { 170, 90, 52, 165, 105, 155 };
int JOINT_LIMIT_MAX_MK2[] = { 170, 90, 52, 165, 105, 155 };
int JOINT_LIMIT_MAX_MK3[] = { 170, 90, 52, 180, 105, 180 };
int JOINT_LIMIT_MAX_MK4[] = { 170, 90, 52, 180, 105, 180 };
int JOINT_LIMIT_MAX_MK5[] = { 160, 90, 52, 180, 105, 180 };

std::map<String, const uint8_t*> LIMIT_SENSOR_PRESSED_STATE;
const uint8_t LIMIT_SENSOR_PRESSED_STATE_MK1[] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };
const uint8_t LIMIT_SENSOR_PRESSED_STATE_MK2[] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };
const uint8_t LIMIT_SENSOR_PRESSED_STATE_MK3[] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };
const uint8_t LIMIT_SENSOR_PRESSED_STATE_MK4[] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };
const uint8_t LIMIT_SENSOR_PRESSED_STATE_MK5[] = { LOW, LOW, LOW, HIGH, HIGH, HIGH };

std::map<String, const float*> ENC_MULT_BY_MODEL;
const float* ENC_MULT_ACTIVE = ENC_MULT_MK1_3;  // safe default until MODEL is known



///////////////////////////////////////////////////////////////////////////////
// ROS Driver Params
///////////////////////////////////////////////////////////////////////////////

// roughly equals 0, 0, 0, 0, 0, 0 degrees without any user-defined offsets.
std::map<String, const int*> REST_MOTOR_STEPS;
const int REST_MOTOR_STEPS_MK1[] = { 7555, 2333, 4944, 7049, 2295, 3431 };
const int REST_MOTOR_STEPS_MK2[] = { 7555, 2333, 4944, 7049, 2295, 3431 };
const int REST_MOTOR_STEPS_MK3[] = { 7555, 2333, 4944, 8960, 2295, 4000 };
const int REST_MOTOR_STEPS_MK4[] = { 15111, 4667, 9889, 17920, 4591, 8000 };
const int REST_MOTOR_STEPS_MK5[] = { 14222, 4667, 9889, 17920, 4591, 8000 };

enum SM { STATE_TRAJ,
          STATE_ERR };
SM STATE = STATE_TRAJ;


AccelStepper stepperJoints[NUM_JOINTS];
Bounce2::Button limitSwitches[NUM_JOINTS];
const int DEBOUNCE_INTERVAL = 10;  // ms

// calibration settings
const int CAL_DIR[] = { -1, -1, 1,
                        -1, -1, 1 };  // joint rotation direction to limit switch
const int CAL_SPEED = 1000;           // motor steps per second
const float CAL_SPEED_MULT[] = {
  1, 1, 1, 1, 0.5, 1
};  // multiplier to account for motor steps/rev
// num of encoder steps in range of motion of jointconst int REST_MOTOR_STEPS_MK5[] = { 14222, 4667, 9889, 17920, 4591, 8000 };
int ENC_RANGE_STEPS[NUM_JOINTS];

// speed and acceleration settings
float JOINT_MAX_SPEED[] = { 60.0, 60.0, 60.0, 60.0, 60.0, 60.0 };  // deg/s
float JOINT_MAX_ACCEL[] = { 30.0, 30.0, 30.0, 30.0, 30.0, 30.0 };  // deg/s^2
char JOINT_NAMES[] = { 'A', 'B', 'C', 'D', 'E', 'F' };

bool estop_pressed = false;

void estopPressed() {
  // Check ESTOP 3 times to avoid false positives due to electrical noise
  for (int i = 0; i < 3; i++) {
    if (digitalRead(ESTOP_PIN) != LOW) {
      return;  // Not really pressed
    }
  }
  estop_pressed = true;
}

void resetEstop() {
  // if ESTOP button is pressed still, do not reset the flag!
  if (digitalRead(ESTOP_PIN) == LOW) {
    return;
  }

  // reset any previously set MT commands
  for (int i = 0; i < NUM_JOINTS; ++i) {
    // NOTE: This may seem redundant but is the only permitted way to set
    // _stepInterval and _n to 0, which is required to avoid a jerk resume
    // when Estop is reset after interruption of an accelerated motion
    stepperJoints[i].setCurrentPosition(stepperJoints[i].currentPosition());
    stepperJoints[i].setSpeed(0);
  }

  estop_pressed = false;
}

bool safeRun(AccelStepper& stepperJoint) {
  if (estop_pressed) return false;
  return stepperJoint.run();
}

bool safeRunSpeed(AccelStepper& stepperJoint) {
  if (estop_pressed) return false;
  return stepperJoint.runSpeed();
}

void setup() {
  MOTOR_STEPS_PER_DEG["mk1"] = MOTOR_STEPS_PER_DEG_MK1;
  MOTOR_STEPS_PER_DEG["mk2"] = MOTOR_STEPS_PER_DEG_MK2;
  MOTOR_STEPS_PER_DEG["mk3"] = MOTOR_STEPS_PER_DEG_MK3;
  MOTOR_STEPS_PER_DEG["mk4"] = MOTOR_STEPS_PER_DEG_MK4;
  MOTOR_STEPS_PER_DEG["mk5"] = MOTOR_STEPS_PER_DEG_MK5;

  JOINT_LIMIT_MIN["mk1"] = JOINT_LIMIT_MIN_MK1;
  JOINT_LIMIT_MIN["mk2"] = JOINT_LIMIT_MIN_MK2;
  JOINT_LIMIT_MIN["mk3"] = JOINT_LIMIT_MIN_MK3;
  JOINT_LIMIT_MIN["mk4"] = JOINT_LIMIT_MIN_MK4;
  JOINT_LIMIT_MIN["mk5"] = JOINT_LIMIT_MIN_MK5;

  JOINT_LIMIT_MAX["mk1"] = JOINT_LIMIT_MAX_MK1;
  JOINT_LIMIT_MAX["mk2"] = JOINT_LIMIT_MAX_MK2;
  JOINT_LIMIT_MAX["mk3"] = JOINT_LIMIT_MAX_MK3;
  JOINT_LIMIT_MAX["mk4"] = JOINT_LIMIT_MAX_MK4;
  JOINT_LIMIT_MAX["mk5"] = JOINT_LIMIT_MAX_MK5;

  REST_MOTOR_STEPS["mk1"] = REST_MOTOR_STEPS_MK1;
  REST_MOTOR_STEPS["mk2"] = REST_MOTOR_STEPS_MK2;
  REST_MOTOR_STEPS["mk3"] = REST_MOTOR_STEPS_MK3;
  REST_MOTOR_STEPS["mk4"] = REST_MOTOR_STEPS_MK4;
  REST_MOTOR_STEPS["mk5"] = REST_MOTOR_STEPS_MK5;

  LIMIT_SENSOR_PRESSED_STATE["mk1"] = LIMIT_SENSOR_PRESSED_STATE_MK1;
  LIMIT_SENSOR_PRESSED_STATE["mk2"] = LIMIT_SENSOR_PRESSED_STATE_MK2;
  LIMIT_SENSOR_PRESSED_STATE["mk3"] = LIMIT_SENSOR_PRESSED_STATE_MK3;
  LIMIT_SENSOR_PRESSED_STATE["mk4"] = LIMIT_SENSOR_PRESSED_STATE_MK4;
  LIMIT_SENSOR_PRESSED_STATE["mk5"] = LIMIT_SENSOR_PRESSED_STATE_MK5;

  ENC_MULT_BY_MODEL["mk1"] = ENC_MULT_MK1_3;
  ENC_MULT_BY_MODEL["mk2"] = ENC_MULT_MK1_3;
  ENC_MULT_BY_MODEL["mk3"] = ENC_MULT_MK1_3;
  ENC_MULT_BY_MODEL["mk4"] = ENC_MULT_MK4_5;
  ENC_MULT_BY_MODEL["mk5"] = ENC_MULT_MK4_5;



  for (int i = 0; i < NUM_JOINTS; ++i) {
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i], OUTPUT);
    pinMode(LIMIT_PINS[i], INPUT);
  }

  for (int i = 0; i < NUM_JOINTS; ++i) {
    limitSwitches[i] = Bounce2::Button();
    limitSwitches[i].attach(LIMIT_PINS[i], INPUT);
    limitSwitches[i].interval(DEBOUNCE_INTERVAL);
    limitSwitches[i].setPressedState(HIGH);
  }

  pinMode(ESTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopPressed, FALLING);

  Serial.begin(115200);
}

void setupSteppersMK1() {
  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(true, false, false);  // DM542T CW
    stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] * MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] * MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMinPulseWidth(10);
  }
  stepperJoints[3].setPinsInverted(false, false, false);  // J4 DM320T CCW
}

void setupSteppersMK2() {
  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(false, false,
                                     false);  // DM320T / DM332T --> CW
    stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] * MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] * MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMinPulseWidth(10);
  }
}

void setupSteppersMK3() {
  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(false, false,
                                     false);  // DM320T / DM332T --> CW
    stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] * MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] * MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMinPulseWidth(10);
  }
}

void setupSteppersMK4() {
  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(false, false,
                                     false);  // DM320T / DM332T --> CW
    stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] * MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] * MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMinPulseWidth(10);
  }
}

void setupSteppersMK5() {
  // initialise AccelStepper instance
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(false, false,
                                     false);  // DM320T / DM332T --> CW
    stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] * MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] * MOTOR_STEPS_PER_DEG[MODEL][i]);
    stepperJoints[i].setMinPulseWidth(10);
  }
}

void applyLimitSwitchPressedStateForModel() {
  if (MODEL == "") return;

  const uint8_t* pressed = LIMIT_SENSOR_PRESSED_STATE[MODEL];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    limitSwitches[i].setPressedState(pressed[i]);
  }
}

void applyLimitSwitchInputModeForModel() {
  // Default (mk1–mk4): all active-high switches → use INPUT (as you had before)
  bool mk5 = (MODEL == "mk5");

  for (int i = 0; i < NUM_JOINTS; ++i) {
    uint8_t mode = INPUT;

    // MK5: J1–J3 hall sensors need pullups, J4–J6 are active-high mech switches
    if (mk5 && i < 3) {
      mode = INPUT_PULLUP;
    }

    pinMode(LIMIT_PINS[i], mode);

    // Re-attach Bounce using mode
    limitSwitches[i].attach(LIMIT_PINS[i], mode);

    // debounce interval
    limitSwitches[i].interval(DEBOUNCE_INTERVAL);
  }
}

int REST_MOTOR_STEPS_ACTIVE[NUM_JOINTS] = { 0, 0, 0, 0, 0, 0 };

void computeRestMotorStepsActiveForModel() {
  const int* base = REST_MOTOR_STEPS.at(MODEL);
  const float* steps_per_deg = MOTOR_STEPS_PER_DEG.at(MODEL);  // you already have this map

  for (int i = 0; i < NUM_JOINTS; ++i) {
    // Convert the user offset (deg) into steps and add to base rest
    int offset_steps = (int)lroundf(CAL_OFFSET_DEG[i] * steps_per_deg[i]);
    REST_MOTOR_STEPS_ACTIVE[i] = base[i] + offset_steps;
  }
}


// initialize stepper motors and constants based on the model. Also verifies
// that the software version matches the firmware version
bool initStateTraj(String inData) {
  // parse initialisation message
  int idxVersion = inData.indexOf('A');
  int idxModel = inData.indexOf('B');
  String softwareVersion = inData.substring(idxVersion + 1, idxModel);
  int versionMatches = (softwareVersion == VERSION);

  String model = inData.substring(idxModel + 1, inData.length() - 1);
  int modelMatches = false;
  if (model == "mk1" || model == "mk2" || model == "mk3" || model == "mk4" || model == "mk5") {
    modelMatches = true;
    MODEL = model;
    applyLimitSwitchInputModeForModel();
    applyLimitSwitchPressedStateForModel();
    ENC_MULT_ACTIVE = ENC_MULT_BY_MODEL.at(MODEL);
    computeRestMotorStepsActiveForModel();



    for (int i = 0; i < NUM_JOINTS; ++i) {
      int joint_range = JOINT_LIMIT_MAX[MODEL][i] - JOINT_LIMIT_MIN[MODEL][i];
      ENC_RANGE_STEPS[i] = static_cast<int>(MOTOR_STEPS_PER_DEG[MODEL][i] * joint_range * ENC_MULT_ACTIVE[i]);
    }

    if (model == "mk1") {
      setupSteppersMK1();
    } else if (model == "mk2") {
      setupSteppersMK2();
    } else if (model == "mk3") {
      setupSteppersMK3();
    } else if (model == "mk4") {
      setupSteppersMK4();
    } else if (model == "mk5") {
      setupSteppersMK5();
    }
  }

  // return acknowledgement with result
  String msg = String("ST") + "A" + versionMatches + "B" + VERSION + "C" + modelMatches + "D" + MODEL;
  Serial.println(msg);

  if (versionMatches && modelMatches) {
    return true;
  }
  return false;
}

template<typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

void readMotorSteps(int* motorSteps) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    motorSteps[i] = encPos[i].read() / ENC_MULT_ACTIVE[i];
  }
}

//void encStepsToJointPos(int* encSteps, double* jointPos) {
//  for (int i = 0; i < NUM_JOINTS; ++i) {
//    jointPos[i] = (encSteps[i] + (CAL_OFFSET_DEG[i] * MOTOR_STEPS_PER_DEG[MODEL][i])) / MOTOR_STEPS_PER_DEG[MODEL][i] * ENC_DIR[i];
//  }
//}



void encStepsToJointPos(int* encSteps, double* jointPos) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    jointPos[i] =
      ((double)encSteps[i] / (double)MOTOR_STEPS_PER_DEG[MODEL][i]) * (double)ENC_DIR[i]
      - (double)CAL_OFFSET_DEG[i] * (double)ENC_DIR[i];
  }
}




void jointPosToEncSteps(double* jointPos, int* encSteps) {
  const int* jmin = JOINT_LIMIT_MIN.at(MODEL);
  const float* spd = MOTOR_STEPS_PER_DEG.at(MODEL);

  for (int i = 0; i < NUM_JOINTS; ++i) {
    double effective_min = (double)jmin[i] - (double)CAL_OFFSET_DEG[i];
    encSteps[i] = (int)lround((jointPos[i] - effective_min) * (double)spd[i]);
  }
}



String JointPosToString(double* jointPos) {
  String out;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    out += JOINT_NAMES[i];
    out += String(jointPos[i], 6);
  }
  return out;
}

String JointVelToString(double* lastVelocity) {
  String out;

  for (int i = 0; i < NUM_JOINTS; ++i) {
    out += JOINT_NAMES[i];
    out += String(lastVelocity[i], 6);
  }

  return out;
}

void ParseMessage(String& inData, double* cmdJointPos) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    bool lastJoint = i == NUM_JOINTS - 1;
    int msgIdxJ_S, msgIdxJ_E = 0;
    msgIdxJ_S = inData.indexOf(JOINT_NAMES[i]);
    msgIdxJ_E = (lastJoint) ? -1 : inData.indexOf(JOINT_NAMES[i + 1]);
    if (msgIdxJ_S == -1) {
      Serial.printf("ER: panic, missing joint %c\n", JOINT_NAMES[i]);
      return;
    }
    if (msgIdxJ_E != -1) {
      cmdJointPos[i] = inData.substring(msgIdxJ_S + 1, msgIdxJ_E).toFloat();
    } else {
      cmdJointPos[i] = inData.substring(msgIdxJ_S + 1).toFloat();
    }
  }
}

void MoveVelocity(String inData) {
  double cmdJointVel[NUM_JOINTS];
  ParseMessage(inData, cmdJointVel);

  for (int i = 0; i < NUM_JOINTS; i++) {
    if (abs(cmdJointVel[i]) > JOINT_MAX_SPEED[i]) {
      Serial.printf("DB: joint %c speed %f > %f, clipping.\n", JOINT_NAMES[i],
                    cmdJointVel[i], JOINT_MAX_SPEED[i]);
      cmdJointVel[i] = sgn(cmdJointVel[i]) * JOINT_MAX_SPEED[i];
    }
    cmdJointVel[i] *= MOTOR_STEPS_PER_DEG[MODEL][i];
    stepperJoints[i].setMaxSpeed(abs(cmdJointVel[i]));
    stepperJoints[i].setSpeed(cmdJointVel[i]);
    stepperJoints[i].move(sgn(cmdJointVel[i]) * __LONG_MAX__);
  }
}

void MoveTo(const int* cmdSteps, int* motorSteps) {
  setAllMaxSpeeds();
  for (int i = 0; i < NUM_JOINTS; ++i) {
    int diffEncSteps = cmdSteps[i] - motorSteps[i];
    if (abs(diffEncSteps) > 2) {
      int diffMotSteps = diffEncSteps * ENC_DIR[i];
      stepperJoints[i].move(diffMotSteps);
    }
  }
}

void MoveTo(String inData, int* motorSteps) {
  double cmdJointPos[NUM_JOINTS] = { 0 };
  ParseMessage(inData, cmdJointPos);

  for (int i = 0; i < NUM_JOINTS; i++) {
    if (abs(cmdJointPos[i] > 380.0)) {
      Serial.printf("ER: panic, joint %c value %f out of range\n",
                    JOINT_NAMES[i], cmdJointPos[i]);
      return;
    }
  }

  // get current joint position
  double curJointPos[NUM_JOINTS];
  encStepsToJointPos(motorSteps, curJointPos);

  // update target joint position
  int cmdEncSteps[NUM_JOINTS] = { 0 };
  jointPosToEncSteps(cmdJointPos, cmdEncSteps);

  MoveTo(cmdEncSteps, motorSteps);
}


void ParkMove(int* curMotorSteps) {
  // Park joint angles in degrees:
  // J1=0, J2=0, J3=-90, J4=0, J5=0, J6=0
  double parkJointPos[NUM_JOINTS] = { 0 };
  parkJointPos[0] = 0.0;
  parkJointPos[1] = 0.0;
  parkJointPos[2] = -90.0;
  parkJointPos[3] = 0.0;
  parkJointPos[4] = 0.0;
  parkJointPos[5] = 0.0;

  int parkEncSteps[NUM_JOINTS] = { 0 };
  jointPosToEncSteps(parkJointPos, parkEncSteps);

  MoveTo(parkEncSteps, curMotorSteps);
}


bool AtPosition(const int* targetMotorSteps, const int* currMotorSteps,
                const int maxDiff) {
  bool allDone = true;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    int diffEncSteps = targetMotorSteps[i] - currMotorSteps[i];
    if (abs(diffEncSteps) > maxDiff) {
      allDone = false;
    }
  }
  return allDone;
}

void setAllMaxSpeeds() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] * MOTOR_STEPS_PER_DEG[MODEL][i]);
  }
}

void updateAllLimitSwitches() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    limitSwitches[i].update();
  }
}

bool moveToLimitSwitches(int* calJoints) {
  // check which joints to calibrate
  bool calAllDone = false;
  bool calJointsDone[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    calJointsDone[i] = !calJoints[i];
  }

  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setMaxSpeed(abs(CAL_SPEED));
    stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i]);
  }
  unsigned long startTime = millis();
  while (!calAllDone) {
    updateAllLimitSwitches();
    calAllDone = true;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      // if joint is not calibrated yet
      if (!calJointsDone[i]) {
        // check limit switches
        if (!limitSwitches[i].isPressed()) {
          // limit switch not reached, continue moving
          safeRunSpeed(stepperJoints[i]);
          calAllDone = false;
        } else {
          // limit switch reached
          stepperJoints[i].setSpeed(0);  // redundancy
          calJointsDone[i] = true;
        }
      }
    }

    if (millis() - startTime > 40000) {
      return false;
    }
  }
  delay(1000);
  return true;
}

bool moveAwayFromLimitSwitch(int* calJoints) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (calJoints[i]) {
      stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i] * -1);
    }
  }

  bool limitSwitchesActive = true;
  unsigned long startTime = millis();
  while (limitSwitchesActive || millis() - startTime < 2500) {
    limitSwitchesActive = false;
    updateAllLimitSwitches();
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (calJoints[i]) {
        if (limitSwitches[i].isPressed()) {
          limitSwitchesActive = true;
        }
        safeRunSpeed(stepperJoints[i]);
      }
    }

    if (millis() - startTime > 10000) {
      return false;
    }
  }

  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setSpeed(0);  // redundancy
  }
  delay(1000);
  return true;
}

bool moveLimitedAwayFromLimitSwitch(int* calJoints) {
  // move the ones that already hit a limit away from it before start of
  // calibration
  int limitedJoints[NUM_JOINTS] = { 0 };
  bool hasLimitedJoints = false;
  bool doCalibrationRoutineSequence;
  updateAllLimitSwitches();
  for (int i = 0; i < NUM_JOINTS; i++) {
    limitedJoints[i] = (calJoints[i] && limitSwitches[i].isPressed());
    hasLimitedJoints = hasLimitedJoints || limitedJoints[i];
  }
  if (hasLimitedJoints) {
    return moveAwayFromLimitSwitch(limitedJoints);
  }
  return true;
}

bool doCalibrationRoutineSequence(String& outputMsg, String& inputMsg) {
  if (inputMsg.length() != 7) {
    outputMsg = "ER: Invalid sequence length.";
    return false;
  }

  // define sequence storage
  int calibSeq[7];
  // convert inputMsg string to int array
  for (int i = 0; i < 7; i++) {
    calibSeq[i] = inputMsg[i] - '0';
  }

  // implement sequence in calJoints
  int calJoints[6][NUM_JOINTS] = { 0 };
  int numGroups = 0;

  switch (calibSeq[0]) {
    case 0:
      numGroups = 1;
      for (int i = 0; i < NUM_JOINTS; i++) {
        calJoints[0][calibSeq[i + 1]] = 1;
      }
      break;
    case 1:
      numGroups = 2;
      for (int i = 0; i < NUM_JOINTS - 3; i++) {
        calJoints[0][calibSeq[i + 1]] = 1;
        calJoints[1][calibSeq[i + 4]] = 1;
      }
      break;
    case 2:
      numGroups = 3;
      for (int i = 0; i < NUM_JOINTS - 4; i++) {
        calJoints[0][calibSeq[i + 1]] = 1;
        calJoints[1][calibSeq[i + 3]] = 1;
        calJoints[2][calibSeq[i + 5]] = 1;
      }
      break;
    case 3:
      numGroups = NUM_JOINTS;
      for (int i = 0; i < NUM_JOINTS; i++) {
        calJoints[i][calibSeq[i + 1]] = 1;
      }
      break;
    default:
      outputMsg = "ER: Invalid calibration sequence.";
      return false;  // Early exit if an invalid value is detected
  }

  // calibrate joints
  int calSteps[6];
  for (int step = 0; step < numGroups; ++step) {
    if (!doCalibrationRoutine(outputMsg, calJoints[step], calSteps)) {
      return false;
    }
  }

  // calibration done, send calibration values
  // N.B. calibration values aren't used right now
  outputMsg = String("JC") + "A" + calSteps[0] + "B" + calSteps[1] + "C" + calSteps[2] + "D" + calSteps[3] + "E" + calSteps[4] + "F" + calSteps[5];
  return true;
}

bool doCalibrationRoutineMask(String& outputMsg, String& maskMsg) {
  if (maskMsg.length() != 6) {
    outputMsg = "ER: Invalid mask length. Expected 6 chars (J1..J6).";
    return false;
  }

  int calJoints[NUM_JOINTS] = { 0 };
  int selectedCount = 0;

  for (int i = 0; i < NUM_JOINTS; i++) {
    char c = maskMsg[i];
    if (c != '0' && c != '1') {
      outputMsg = "ER: Mask must contain only 0 or 1.";
      return false;
    }
    if (c == '1') {
      calJoints[i] = 1;
      selectedCount++;
    }
  }

  if (selectedCount == 0) {
    outputMsg = "ER: Mask selects no joints.";
    return false;
  }

  // Run the SAME calibration routine used by JC startup, but only for selected joints.
  int calSteps[NUM_JOINTS] = { 0 };
  bool ok = doCalibrationRoutine(outputMsg, calJoints, calSteps);

  if (!ok) return false;

  // Return JC so ROS driver parser stays happy
  outputMsg = String("JC") + "A" + calSteps[0] + "B" + calSteps[1] + "C" + calSteps[2] + "D" + calSteps[3] + "E" + calSteps[4] + "F" + calSteps[5];

  return true;
}




bool doCalibrationRoutine(String& outputMsg, int calJoints[NUM_JOINTS],
                          int calSteps[]) {
  if (!moveLimitedAwayFromLimitSwitch(calJoints)) {
    outputMsg = "ER: Failed to move away from limit switches at the start.";
    return false;
  }

  if (!moveToLimitSwitches(calJoints)) {
    outputMsg = "ER: Failed to move to limit switches.";
    return false;
  }

  // record encoder steps
  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (calJoints[i]) {
      calSteps[i] = encPos[i].read();
    }
  }


  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (!calJoints[i]) continue;

    float spd = MOTOR_STEPS_PER_DEG[MODEL][i];
    float mult = ENC_MULT_ACTIVE[i];

    long base = (long)ENC_RANGE_STEPS[i] * (long)ENC_MAX_AT_ANGLE_MIN[i];
    long offsetEnc = lroundf(CAL_OFFSET_DEG[i] * spd * mult);

    // If we set to max-at-min, apply offset in the opposite direction
    long encWrite = base + (ENC_MAX_AT_ANGLE_MIN[i] ? -offsetEnc : offsetEnc);

    // optional safety clamp
    if (encWrite < 0) encWrite = 0;
    if (encWrite > (long)ENC_RANGE_STEPS[i]) encWrite = (long)ENC_RANGE_STEPS[i];

    encPos[i].write(encWrite);
  }





  //for (int i = 0; i < NUM_JOINTS; ++i) {
  //  if (calJoints[i]) {
  //    encPos[i].write(ENC_RANGE_STEPS[i] * ENC_MAX_AT_ANGLE_MIN[i]);
  //  }
  //}

  // move away from the limit switches a bit so that if the next command
  // is in the wrong direction, the limit switches will not be run over
  // immediately and become damaged.
  if (!moveAwayFromLimitSwitch(calJoints)) {
    outputMsg = "ER: Failed to move away from limit switches.";
    return false;
  }

  // return to original position
  unsigned long startTime = millis();
  int curMotorSteps[NUM_JOINTS];
  readMotorSteps(curMotorSteps);

  while (!AtPosition(REST_MOTOR_STEPS_ACTIVE, curMotorSteps, 5)) {
    if (millis() - startTime > 12000) {
      // Note: this occasionally happens but doesn't affect calibration result
      Serial.println(
        "WN: Failed to return to original position post calibration.");
      break;
    }

    readMotorSteps(curMotorSteps);
    for (int i = 0; i < NUM_JOINTS; ++i) {
      if (!calJoints[i]) {
        curMotorSteps[i] = REST_MOTOR_STEPS_ACTIVE[i];
      }
    }
    MoveTo(REST_MOTOR_STEPS_ACTIVE, curMotorSteps);

    for (int i = 0; i < NUM_JOINTS; ++i) {
      safeRun(stepperJoints[i]);
    }
  }

  // restore original max speed
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] * MOTOR_STEPS_PER_DEG[MODEL][i]);
  }
  return true;
}

void updateMotorVelocities(int* motorSteps, int* lastMotorSteps,
                           int* checksteps, unsigned long* lastVelocityCalc,
                           double* lastVelocity) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    // for really small velocities we still get
    // artifacts, but quite manageable now!

    if (micros() - lastVelocityCalc[i] < 5000) {
      // we want to trigger calculation only after x ms but
      // immediately when steps change after that
      checksteps[i] = motorSteps[i];
      continue;
    }

    // have to add some sort of outlier-filter here , maybe moving average ..
    if (abs(stepperJoints[i].speed() / MOTOR_STEPS_PER_DEG[MODEL][i]) < 5) {
      // NB! trying to fix artifacts at low velocity
      if (abs(motorSteps[i] - checksteps[i]) > 0) {
        lastVelocity[i] =
          stepperJoints[i].speed() / MOTOR_STEPS_PER_DEG[MODEL][i];
        lastMotorSteps[i] = motorSteps[i];
        lastVelocityCalc[i] = micros();
      } else if (stepperJoints[i].speed() == 0) {
        lastVelocity[i] = 0;
      }
    } else {
      unsigned long currentMicros = micros();
      double delta = (currentMicros - lastVelocityCalc[i]);
      if (abs(motorSteps[i] - checksteps[i]) > 0) {
        // calculate TRUE motor velocity
        lastVelocity[i] = ENC_DIR[i] * (motorSteps[i] - lastMotorSteps[i]) / MOTOR_STEPS_PER_DEG[MODEL][i] / (delta / 1000000.0);
        lastMotorSteps[i] = motorSteps[i];
        lastVelocityCalc[i] = currentMicros;
      }
    }
  }
}

void stateTRAJ() {
  // clear message
  String inData = "";

  // initialise joint steps
  double curJointPos[NUM_JOINTS];
  int curMotorSteps[NUM_JOINTS];
  int lastMotorSteps[NUM_JOINTS];
  int checksteps[NUM_JOINTS];
  double lastVelocity[NUM_JOINTS];
  unsigned long lastVelocityCalc[NUM_JOINTS];

  readMotorSteps(curMotorSteps);

  for (int i = 0; i < NUM_JOINTS; ++i) {
    lastVelocityCalc[i] = micros();
    lastMotorSteps[i] = curMotorSteps[i];
  }

  // start loop
  while (STATE == STATE_TRAJ) {
    char received = '\0';
    // check for message from host
    if (Serial.available()) {
      received = Serial.read();
      inData += received;
    }

    if (MODEL != "") {
      readMotorSteps(curMotorSteps);
      updateMotorVelocities(curMotorSteps, lastMotorSteps, checksteps,
                            lastVelocityCalc, lastVelocity);
    }

    // process message when new line character is received
    if (received == '\n') {
      String function = inData.substring(0, 2);
      if (function == "ST") {
        if (!initStateTraj(inData)) {
          STATE = STATE_ERR;
          return;
        }
      } else if (MODEL == "") {
        // if model is not set, do not proceed with any other function
        STATE = STATE_ERR;
        return;
      }

      if (function == "MT") {
        // clear speed counter
        for (int i = 0; i < NUM_JOINTS; i++) {
          if (stepperJoints[i].speed() == 0) {
            lastVelocityCalc[i] = micros();
          }
        }

        MoveTo(inData, curMotorSteps);

        // update the host about estop state
        String msg = String("ES") + estop_pressed;
        Serial.println(msg);

      } else if (function == "MV") {
        // clear speed counter
        for (int i = 0; i < NUM_JOINTS; i++) {
          if (stepperJoints[i].speed() == 0) {
            lastVelocityCalc[i] = micros();
          }
        }

        MoveVelocity(inData);

        // update the host about estop state
        String msg = String("ES") + estop_pressed;
        Serial.println(msg);


      } else if (function == "PK") {
        // Build park target steps
        double parkJointPos[NUM_JOINTS] = { 0 };
        parkJointPos[0] = 0.0;
        parkJointPos[1] = 0.0;
        parkJointPos[2] = -90.0;
        parkJointPos[3] = 0.0;
        parkJointPos[4] = 0.0;
        parkJointPos[5] = 0.0;

        int parkEncSteps[NUM_JOINTS] = { 0 };
        jointPosToEncSteps(parkJointPos, parkEncSteps);

        // Start move
        MoveTo(parkEncSteps, curMotorSteps);

        // BLOCK until we arrive (or timeout)
        unsigned long startTime = millis();
        while (true) {
          if (millis() - startTime > 60000) {  // 60s timeout
            Serial.println("ER: Park timeout.");
            break;
          }

          readMotorSteps(curMotorSteps);

          if (AtPosition(parkEncSteps, curMotorSteps, 5)) {
            Serial.println("PK");  // only acknowledge when done
            break;
          }

          // execute motion
          for (int i = 0; i < NUM_JOINTS; ++i) {
            safeRun(stepperJoints[i]);
          }
        }




      } else if (function == "JP") {
        readMotorSteps(curMotorSteps);
        encStepsToJointPos(curMotorSteps, curJointPos);
        String msg = String("JP") + JointPosToString(curJointPos);
        Serial.println(msg);




      } else if (function == "JV") {
        String msg = String("JV") + JointVelToString(lastVelocity);
        Serial.println(msg);

      } else if (function == "JC") {
        String msg, inputMsg;
        inputMsg = inData.substring(2, 9);
        if (!doCalibrationRoutineSequence(msg, inputMsg)) {
          for (int i = 0; i < NUM_JOINTS; ++i) {
            stepperJoints[i].setSpeed(0);
          }
        }
        Serial.println(msg);

      } else if (function == "JM") {
        String msg, maskMsg;
        // Expect "JM" + 6 chars => total 8 before newline
        maskMsg = inData.substring(2, 8);

        if (!doCalibrationRoutineMask(msg, maskMsg)) {
          for (int i = 0; i < NUM_JOINTS; ++i) {
            stepperJoints[i].setSpeed(0);
          }
        }
        Serial.println(msg);

      } else if (function == "RE") {
        resetEstop();
        // update host with Estop status after trying to reset it
        String msg = String("ES") + estop_pressed;
        Serial.println(msg);
      }

      inData = "";  // clear message
    }

    for (int i = 0; i < NUM_JOINTS; ++i) {
      safeRun(stepperJoints[i]);
    }
  }
}

void stateERR() {
  // enter holding state
  for (int i = 0; i < NUM_JOINTS; ++i) {
    digitalWrite(STEP_PINS[i], LOW);
  }

  while (STATE == STATE_ERR) {
    Serial.println("ER: Unrecoverable error state entered. Please reset.");
    delay(1000);
  }
}

void loop() {
  STATE = STATE_TRAJ;

  switch (STATE) {
    case STATE_ERR:
      stateERR();
      break;
    default:
      stateTRAJ();
      break;
  }
}
