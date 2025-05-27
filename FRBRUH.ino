#include <POP32.h>
#include <POP32_Huskylens.h>

POP32_Huskylens huskylens;

// -------------------- CONSTANTS --------------------
#define DEG_TO_RAD 0.0174533f
#define SIN_30 sin(30.f * DEG_TO_RAD)
#define COS_30 cos(30.f * DEG_TO_RAD)

// PID for rotation
#define ROT_KP 0.8
#define ROT_KI 0.02
#define ROT_KD 0.1
#define ROT_SP 160
#define ROT_ERROR_GAP 10
#define IDLE_ROT_SPD 30

// PID for forward movement
#define FLI_KP 1.2
#define FLI_KI 0.02
#define FLI_KD 0.1
#define FLI_ERROR_GAP 15
float FLI_SP = 160;

// Heading control
#define HEAD_KP 2.3f
#define HEAD_KI 0.01f
#define HEAD_KD 0.5f

// Shooting
#define LIM_PIN A0
#define RELOAD_SPD 60
#define SHOOT_DELAY 150
#define RELOAD_TIMEOUT 2000
#define ALIGN_ERROR_GAP 4

// IMU
#define YAW_FILTER_ALPHA 0.1f
#define IMU_TIMEOUT 1000

// Debug
#define DEBUG true

// -------------------- STATE VARIABLES --------------------
float rot_error, rot_pError, rot_i, rot_d, rot_w;
float fli_error, fli_pError, fli_i, fli_d, fli_spd;
float head_error, head_pError, head_i, head_d, head_w;
float thetaRad, vx, vy, spd1, spd2, spd3;

float filteredYaw = 0;
float pvYaw, lastYaw;
uint8_t rxCnt = 0, rxBuf[8];

int ballPosX, ballPosY;
int discoverState = 1;
unsigned long lostBallTimer = 0;

// -------------------- IMU FUNCTIONS --------------------
bool getIMU() {
  unsigned long start = millis();
  while (millis() - start < IMU_TIMEOUT) {
    while (Serial1.available()) {
      if (rxCnt < sizeof(rxBuf)) {
        rxBuf[rxCnt] = Serial1.read();
        if (rxCnt == 0 && rxBuf[0] != 0xAA) return false;
        rxCnt++;
        if (rxCnt == 8) {
          rxCnt = 0;
          if (rxBuf[0] == 0xAA && rxBuf[7] == 0x55) {
            float rawYaw = (int16_t)(rxBuf[1] << 8 | rxBuf[2]) / 100.f;
            pvYaw = filteredYaw = (1 - YAW_FILTER_ALPHA) * filteredYaw + YAW_FILTER_ALPHA * rawYaw;
            return true;
          }
        }
      } else {
        rxCnt = 0;
      }
    }
  }
  return false;
}

void zeroYaw() {
  Serial1.begin(115200);
  delay(100);
  Serial1.write(0xA5); delay(100); Serial1.write(0x54);
  Serial1.write(0xA5); delay(100); Serial1.write(0x55);
  Serial1.write(0xA5); delay(100); Serial1.write(0x52);
}

void Auto_zero() {
  zeroYaw();
  getIMU();
  unsigned long timer = millis();

  oled.clear();
  oled.text(1, 2, "Setting zero");

  while (abs(pvYaw) > 0.02) {
    if (getIMU()) {
      oled.text(3, 6, "Yaw: %.2f", pvYaw);
      oled.show();
      beep();
      if (millis() - timer > 5000) {
        zeroYaw();
        timer = millis();
      }
    }
  }

  oled.clear();
  oled.show();
}

// -------------------- MOVEMENT --------------------
void wheel(int s1, int s2, int s3) {
  motor(1, s1);
  motor(2, s2);
  motor(3, s3);
}

void holonomic(float spd, float theta, float omega) {
  thetaRad = theta * DEG_TO_RAD;
  vx = spd * cos(thetaRad);
  vy = spd * sin(thetaRad);
  spd1 = vy * COS_30 - vx * SIN_30 + omega;
  spd2 = -vy * COS_30 - vx * SIN_30 + omega;
  spd3 = vx + omega;
  wheel(spd1, spd2, spd3);
}

void heading(float spd, float theta, float spYaw) {
  head_error = spYaw - pvYaw;
  head_i += head_error;
  head_i = constrain(head_i, -180, 180);
  head_d = head_error - head_pError;
  head_w = HEAD_KP * head_error + HEAD_KI * head_i + HEAD_KD * head_d;
  head_w = constrain(head_w, -100, 100);
  holonomic(spd, theta, head_w);
  head_pError = head_error;
}

// -------------------- SHOOTING --------------------
void shoot() {
  motor(4, RELOAD_SPD);
  delay(SHOOT_DELAY);
  motor(4, 0);
  delay(50);
  beep();
}

void reload() {
  motor(4, RELOAD_SPD);
  unsigned long start = millis();

  while (millis() - start < RELOAD_TIMEOUT && analogRead(LIM_PIN) < 700);

  if (millis() - start >= RELOAD_TIMEOUT) {
    motor(4, -RELOAD_SPD);
    delay(500);
    motor(4, RELOAD_SPD);
    start = millis();
    while (millis() - start < RELOAD_TIMEOUT && analogRead(LIM_PIN) < 700);
  }

  motor(4, 0);
}

// -------------------- DEBUG --------------------
void debugPrint() {
  if (DEBUG) {
    oled.clear();
    oled.text(0, 0, "Yaw: %.2f", pvYaw);
    oled.text(1, 0, "BallX: %d", ballPosX);
    oled.text(2, 0, "BallY: %d", ballPosY);
    oled.show();
  }
}

// -------------------- MAIN --------------------
void setup() {
  reload();
  oled.begin();

  zeroYaw();
  oled.text(0, 0, "Ready. Press A");
  oled.show();

  lostBallTimer = millis();

  while (!SW_A()) {
    getIMU();
    oled.text(1, 0, "Yaw = %.2f", pvYaw);
    oled.show();
    delay(100); // debounce
  }

  holonomic(50, 90, 0);
  delay(800);
  wheel(0, 0, 0);
  delay(200);
}

void loop() {
  if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
    ballPosX = huskylens.blockInfo[1][0].x;
    ballPosY = huskylens.blockInfo[1][0].y;

    lostBallTimer = millis();

    for (int i = 0; i < 8; i++) getIMU();

    if (discoverState) {
      rot_error = ROT_SP - ballPosX;
      rot_i += rot_error;
      rot_d = rot_error - rot_pError;
      rot_w = ROT_KP * rot_error + ROT_KI * rot_i + ROT_KD * rot_d;
      rot_w = constrain(rot_w, -100, 100);
      rot_pError = rot_error;

      fli_error = FLI_SP - ballPosY;
      fli_i += fli_error;
      fli_d = fli_error - fli_pError;
      fli_spd = FLI_KP * fli_error + FLI_KI * fli_i + FLI_KD * fli_d;
      fli_spd = constrain(fli_spd, -100, 100);
      fli_pError = fli_error;

      holonomic(fli_spd, 90, rot_w);

      if (abs(rot_error) < ROT_ERROR_GAP && abs(fli_error) < FLI_ERROR_GAP) {
        wheel(0, 0, 0);
        lastYaw = pvYaw;
        discoverState = 0;
      }
    } else {
      float vecCurve = lastYaw < 0 ? 0 : 180;
      float radCurve = lastYaw < 0 ? 15 : -15;
      holonomic(40, vecCurve, radCurve);

      if (abs(pvYaw) < ALIGN_ERROR_GAP) {
        rot_error = ROT_SP - ballPosX;
        if (abs(rot_error) < ROT_ERROR_GAP) {
          holonomic(90, 90, 0);
          delay(1000);
          beep();
          shoot();
          reload();
        }
        discoverState = 1;
      }
    }
  } else {
    int sideRot = ROT_SP - ballPosX;
    int direction = (sideRot == 0) ? 1 : sideRot / abs(sideRot);
    holonomic(0, 0, direction * IDLE_ROT_SPD);

    if (millis() - lostBallTimer > 5000) {
      reload();
      zeroYaw();
      lostBallTimer = millis();
    }

    discoverState = 1;
  }

  debugPrint();
}
