#include <Wire.h>
#include <Arduino.h>
#include <math.h>

// PPM Input Handling
volatile unsigned long a, b, c;
volatile int x[15], ch1[15], ch[7];
volatile int i = 0;

// MPU6050 Variables
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float RateRoll, RatePitch, RateYaw;

// Kalman Filter Variables
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
const float t = 0.004; // 4ms loop time

// PID Variables
float DesiredAngleRoll, DesiredAnglePitch, InputThrottle;
float ErrorAngleRoll, ErrorAnglePitch;
float PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll;
float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;

float PRateRoll = 0.625, IRateRoll = 2.1, DRateRoll = 0.0088;
float PRatePitch = 0.625, IRatePitch = 2.1, DRatePitch = 0.0088;
float PAngleRoll = 2, IAngleRoll = 0.5, DAngleRoll = 0.007;
float PAnglePitch = 2, IAnglePitch = 0.5, DAnglePitch = 0.007;

float DesiredRateRoll, DesiredRatePitch;
float ErrorRateRoll, ErrorRatePitch;
float PrevErrorRateRoll, PrevErrorRatePitch;
float PrevItermRateRoll, PrevItermRatePitch;
float InputRoll, InputPitch;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  
  // Setup MPU6050
  setupMPU6050();

  // Setup PPM Input (Pin 2)
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), readPPM, FALLING);
}

void loop() {
  // Read MPU6050 data
  readMPU6050();

  // Apply Kalman Filter
  applyKalmanFilter();

  // Read PPM values
  readRC();

  // Apply PID Control
  calculatePID();

  // Output data to Serial Monitor
  outputData();

  delay(100); // 4ms loop time
}

// ==================== PPM INPUT HANDLING ====================
void readPPM() {
  a = micros();
  c = a - b;
  b = a;
  x[i] = c;
  i++;
  if (i == 15) {
    for (int j = 0; j < 15; j++) {
      ch1[j] = x[j];
    }
    i = 0;
  }
}

void readRC() {
  int j = 0;
  for (int k = 14; k >= 0; k--) {
    if (ch1[k] > 3200) {
      j = k;
      break;
    }
  }
  for (int i = 1; i <= 6; i++) {
    ch[i] = ch1[i + j] - 1000; // Center PPM values around 1500
  }
}

// ==================== MPU6050 HANDLING ====================
void setupMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}

void readMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);

  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  AccX = AccXLSB / 4096.0;
  AccY = AccYLSB / 4096.0;
  AccZ = AccZLSB / 4096.0;

  RateRoll = GyroX / 65.5;
  RatePitch = GyroY / 65.5;
  RateYaw = GyroZ / 65.5;

  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 57.2958;
  AnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 57.2958;
}

// ==================== KALMAN FILTER ====================
void applyKalmanFilter() {
  KalmanAngleRoll += t * RateRoll;
  KalmanUncertaintyAngleRoll += t * t * 16;
  float KalmanGainRoll = KalmanUncertaintyAngleRoll / (KalmanUncertaintyAngleRoll + 9);
  KalmanAngleRoll += KalmanGainRoll * (AngleRoll - KalmanAngleRoll);
  KalmanUncertaintyAngleRoll *= (1 - KalmanGainRoll);

  KalmanAnglePitch += t * RatePitch;
  KalmanUncertaintyAnglePitch += t * t * 16;
  float KalmanGainPitch = KalmanUncertaintyAnglePitch / (KalmanUncertaintyAnglePitch + 9);
  KalmanAnglePitch += KalmanGainPitch * (AnglePitch - KalmanAnglePitch);
  KalmanUncertaintyAnglePitch *= (1 - KalmanGainPitch);

  KalmanAngleRoll = constrain(KalmanAngleRoll, -20, 20);
  KalmanAnglePitch = constrain(KalmanAnglePitch, -20, 20);

DesiredAngleRoll = 0.1 * (ch[1] - 500); // Center at 500
DesiredAnglePitch = 0.1 * (ch[2] - 500); // Center at 500

}


void calculatePID() {
  // === Roll PID ===
 // Inlined PID equation for Roll
ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
PtermRoll = PAngleRoll * ErrorAngleRoll;
ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
DesiredRateRoll = PIDOutputRoll;
PrevErrorAngleRoll = ErrorAngleRoll;
PrevItermAngleRoll = ItermRoll;

ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
PtermPitch = PAnglePitch * ErrorAnglePitch;
ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
DesiredRatePitch = PIDOutputPitch;
PrevErrorAnglePitch = ErrorAnglePitch;
PrevItermAnglePitch = ItermPitch;

// Compute errors
ErrorRateRoll = DesiredRateRoll - RateRoll;
ErrorRatePitch = DesiredRatePitch - RatePitch;

// Roll Axis PID
PtermRoll = PRateRoll * ErrorRateRoll;
ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);

// Update output and previous values for Roll
InputRoll = PIDOutputRoll;
PrevErrorRateRoll = ErrorRateRoll;
PrevItermRateRoll = ItermRoll;

// Pitch Axis PID
PtermPitch = PRatePitch * ErrorRatePitch;
ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);

// Update output and previous values for Pitch
InputPitch = PIDOutputPitch;
PrevErrorRatePitch = ErrorRatePitch;
PrevItermRatePitch = ItermPitch;

}

// ==================== OUTPUT ====================
void outputData() {
  //Serial.print(ch[1]);Serial.print("\t");
  //Serial.print(ch[2]);Serial.print("\t");
  Serial.println("  KalmanAngleRoll: ");
  Serial.println(KalmanAngleRoll);
  //Serial.print("  KalmanAnglePitch: ");
  //Serial.println(KalmanAnglePitch);

  Serial.println("Roll: ");
  Serial.println(InputRoll);
  //Serial.print("\tPitch: ");
  //Serial.println(InputPitch);
}
