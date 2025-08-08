#include <Wire.h>
#include <math.h>

// Kalman filter variables
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};

float RateRoll, RatePitch, RateYaw;

float t = 0.004; // Time cycle in seconds (4ms)

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Fast I2C mode
  delay(250);

  // Initialize MPU6050
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

void loop() {
  readMPU6050();

  // Apply Kalman Filter for Roll
  KalmanAngleRoll += t * RateRoll;
  KalmanUncertaintyAngleRoll += t * t * 16; // IMU variance = 4 deg/s -> 4^2 = 16
  float KalmanGainRoll = KalmanUncertaintyAngleRoll / (KalmanUncertaintyAngleRoll + 9); // Measurement variance = 3 deg -> 3^2 = 9
  KalmanAngleRoll += KalmanGainRoll * (AngleRoll - KalmanAngleRoll);
  KalmanUncertaintyAngleRoll *= (1 - KalmanGainRoll);

  // Apply Kalman Filter for Pitch
  KalmanAnglePitch += t * RatePitch;
  KalmanUncertaintyAnglePitch += t * t * 16;
  float KalmanGainPitch = KalmanUncertaintyAnglePitch / (KalmanUncertaintyAnglePitch + 9);
  KalmanAnglePitch += KalmanGainPitch * (AnglePitch - KalmanAnglePitch);
  KalmanUncertaintyAnglePitch *= (1 - KalmanGainPitch);

  // Constrain angles to ±20 degrees
  KalmanAngleRoll = constrain(KalmanAngleRoll, -20, 20);
  KalmanAnglePitch = constrain(KalmanAnglePitch, -20, 20);

  // Output to serial monitor
  Serial.print("AngleRoll: ");
  Serial.print(AngleRoll);
  Serial.print("  AnglePitch: ");
  Serial.print(AnglePitch);
  Serial.print("  KalmanAngleRoll: ");
  Serial.print(KalmanAngleRoll);
  Serial.print("  KalmanAnglePitch: ");
  Serial.println(KalmanAnglePitch);

  delay(4); // 4ms loop time
}

void readMPU6050() {
  // Read accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Read gyroscope data
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // Convert raw data to physical units
  AccX = (float)AccXLSB / 4096.0;
  AccY = (float)AccYLSB / 4096.0;
  AccZ = (float)AccZLSB / 4096.0;

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  // Compute angles from accelerometer (in degrees)
  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 57.2958; // 57.2958 = 180/π
  AnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 57.2958;
}
