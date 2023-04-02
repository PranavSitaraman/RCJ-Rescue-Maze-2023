#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <MeMegaPi.h>
#include <Adafruit_TCS34725.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <avr/wdt.h>
namespace Dir {
enum : uint8_t {
  N,
  E,
  S,
  W
};
}
namespace Coord {
enum : uint8_t {
  X,
  Y,
  Z
};
}
namespace Move {
enum : uint8_t {
  BLACK,
  SUCCESS,
  RAMP,
  SILVER
};
}
constexpr auto RESET = Dir::W + 1;
MeMegaPiDCMotor motors[2] = { MeMegaPiDCMotor(PORT1B), MeMegaPiDCMotor(PORT2B) };
VL53L0X tof;
Adafruit_TCS34725 color = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
Adafruit_BNO055 bno(55);
Servo servo;
constexpr uint8_t VLX[]{ 6, 7, 1, 0};
constexpr uint8_t BOS[]{ 5 };
constexpr uint8_t COLOR[]{ 2 };
constexpr uint8_t ENC = 18;
constexpr uint8_t DIST_THRESH = 12;
constexpr uint8_t DIST_THRESH2 = 15;
constexpr uint8_t LED = 12;
constexpr uint8_t SERVOPIN = 9;
constexpr double WHEEL_RAD = 3.6;
constexpr uint16_t TICKS_PER_ROTATION = 368;
constexpr double DEFAULT_MOTOR = 0.3;
volatile uint16_t encoder = 0;
void encoderISR() {
  encoder++;
}
void tcaselect(uint16_t i) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();
}
void motorReset() {
  motors[0].stop();
  motors[1].stop();
  encoder = 0;
}
void resetFunc() {
  MCUSR = 0;
  wdt_enable(WDTO_250MS);
  while (true)
    ;
}
void drop(int side) {
  if (side == 0) {
    servo.write(120);
    delay(1000);
    servo.write(45);
    delay(1000);
    servo.write(60);
  } else {
    servo.write(0);
    delay(1000);
    servo.write(75);
    delay(1000);
    servo.write(60);
  }
}
void handleVictim() {
  uint8_t val = Serial1.read();
  uint8_t side = Serial1.read();
  digitalWrite(LED, HIGH);
  delay(5000);
  digitalWrite(LED, LOW);
  for (uint8_t i = 0; i < val; i++)
    drop(side);
}
uint16_t distance(uint16_t port = VLX[0]) {
  tcaselect(port);
  return tof.readRangeSingleMillimeters();
}
int16_t orientation(uint8_t coord, uint16_t port = BOS[0]) {
  tcaselect(port);
  sensors_event_t event;
  bno.getEvent(&event);
  if (coord == Coord::X)
    return event.orientation.x;
  if (coord == Coord::Y)
    return event.orientation.y;
  return event.orientation.z;
}
uint8_t move(const bool dir[2], double a, double motorSpeed) {
  bool alreadysilver = false;
  static constexpr uint16_t kp = 1.3;
  double b = motorSpeed;
  motorSpeed *= 255;
  for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
    motors[i].run(motorSpeed * (dir[i] ? 1 : -1));
  while (encoder < ((TICKS_PER_ROTATION * a) / (2 * PI * WHEEL_RAD))) {
    if (abs(orientation(Coord::Y, BOS[0])) > 20) {
      while (abs(orientation(Coord::Y, BOS[0])) > 20) {
        if (orientation(Coord::Y, BOS[0]) < -20)
          for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
            motors[i].run(motorSpeed * 1.5 * (dir[i] ? 1 : -1));
        else
          for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
            motors[i].run(motorSpeed * 0.5 * (dir[i] ? 1 : -1));
      }
      motorReset();
      return Move::RAMP;
    }
    uint16_t left = distance(VLX[Dir::W]) / 10;
    uint16_t right = distance(VLX[Dir::E]) / 10;
    uint16_t up = distance(VLX[Dir::N]) / 10;
    uint16_t down = distance(VLX[Dir::S]) / 10;
    if (dir[0] && up < DIST_THRESH2)
      break;
    if (!dir[0] && down < DIST_THRESH2)
      break;
    if (left <= 2 * DIST_THRESH && right <= 2 * DIST_THRESH) {
      motors[0].run((motorSpeed + kp * (left - right)) * (dir[0] ? 1 : -1));
      motors[1].run((motorSpeed + kp * (right - left)) * (dir[1] ? 1 : -1));
    } else if (left <= 2 * DIST_THRESH && right > 2 * DIST_THRESH) {
      motors[0].run((motorSpeed + kp * (left - DIST_THRESH)) * (dir[0] ? 1 : -1));
      motors[1].run((motorSpeed + kp * (DIST_THRESH - left)) * (dir[1] ? 1 : -1));
    } else if (left > 2 * DIST_THRESH && right <= 2 * DIST_THRESH) {
      motors[0].run((motorSpeed + kp * (DIST_THRESH - right)) * (dir[0] ? 1 : -1));
      motors[1].run((motorSpeed + kp * (right - DIST_THRESH)) * (dir[1] ? 1 : -1));
    } else
      for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
        motors[i].run(motorSpeed * (dir[i] ? 1 : -1));
    /*
    tcaselect(COLOR[0]);
    uint16_t red, green, blue, c;
    color.getRawData(&red, &green, &blue, &c);
    const uint16_t BLACK_UPPER_R = 5;
    const uint16_t BLACK_UPPER_G = 5;
    const uint16_t BLACK_UPPER_B = 5;
    const uint16_t SILVER_LOWER_R = 10;
    const uint16_t SILVER_LOWER_G = 40;
    const uint16_t SILVER_LOWER_B = 30;
    if (red < BLACK_UPPER_R && green < BLACK_UPPER_G && blue < BLACK_UPPER_B) {
      uint16_t reverse = encoder;
      motorReset();
      for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
        motors[i].run(motorSpeed * (dir[i] ? -1 : 1));
      while (encoder < reverse)
        ;
      motorReset();
      return Move::BLACK;
    }
    if (red > SILVER_LOWER_R && green > SILVER_LOWER_G && blue > SILVER_LOWER_B && !alreadysilver)
      alreadysilver = true;
    if (Serial1.available()) {
      Serial1.read();
      for (const auto motor : motors)
        motor.stop();
      handleVictim();
      for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
        motors[i].run(motorSpeed * (dir[i] ? 1 : -1));
      break;
    }
    */
  }
  uint16_t up = distance(VLX[Dir::N]) / 10;
  uint16_t down = distance(VLX[Dir::S]) / 10;
  if (dir[0] && up < 2 * DIST_THRESH2)
    while (up > DIST_THRESH2)
      up = distance(VLX[Dir::N]) / 10;
  else if (!dir[0] && down < 2 * DIST_THRESH2)
    while (down > DIST_THRESH2)
      down = distance(VLX[Dir::S]) / 10;
  motorReset();
  if (alreadysilver)
    return Move::SILVER;
  return Move::SUCCESS;
}
bool forward(double a = 32, double motorSpeed = DEFAULT_MOTOR) {
  static constexpr bool dir[]{ true, false };
  return move(dir, a, motorSpeed);
}
bool backward(double a = 32, double motorSpeed = DEFAULT_MOTOR) {
  static constexpr bool dir[]{ false, true };
  return move(dir, a, motorSpeed);
}
void turn(const bool dir[2], double a, double motorSpeed, uint16_t port) {
  for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++) {
    motors[i].run(motorSpeed * (dir[i] ? 1 : -1));
  }
  int16_t start = orientation(Coord::X, port);
  while (abs((orientation(Coord::X, port) - start + 540) % 360 - 180) < a)
    ;
  motorReset();
}
void left(double a = 90, double motorSpeed = DEFAULT_MOTOR, uint16_t port = BOS[0]) {
  static constexpr bool dir[]{ false, false };
  turn(dir, a, motorSpeed * 255, port);
}
void right(double a = 90, double motorSpeed = DEFAULT_MOTOR, uint16_t port = BOS[0]) {
  static constexpr bool dir[]{ true, true };
  turn(dir, a, motorSpeed * 255, port);
}
void setup() {
  Serial.begin(9600);
  Serial.println("begin setup");
  Serial2.begin(9600);
  Wire.begin();
  for (auto port : BOS) {
    tcaselect(port);
    bno.begin();
  }
  for (auto port : VLX) {
    tcaselect(port);
    tof.setTimeout(500);
    tof.init();
    tof.setMeasurementTimingBudget(100000);
  }
  for (auto port : COLOR) {
    tcaselect(port);
    color.begin();
  }
  // Serial1.begin(9600);
  attachInterrupt(digitalPinToInterrupt(ENC), &encoderISR, RISING);
  // servo.attach(SERVOPIN);
  // servo.write(60);
  // pinMode(LED, OUTPUT);
  motorReset();
  while (Serial2.available()) {
    Serial2.read();
    delay(100);
  }
  Serial.println("end setup");
  Serial2.write((uint8_t)1);
}
void loop() {
  if (Serial2.available()) {
    uint8_t command = Serial2.read();
    Serial.println(command);
    if (command >> 7) {
      command &= ~(1 << 7);
      uint8_t state;
      switch (command) {
        case Dir::N:
          state = forward();
          break;
        case Dir::E:
          right();
          state = forward();
          break;
        case Dir::S:
          state = backward();
          break;
        case Dir::W:
          left();
          state = forward();
          break;
        case RESET:
          resetFunc();
          break;
      }
      Serial2.write(state);
    } else {
      uint16_t val = distance(VLX[command]);
      Serial.println(val);
      Serial2.write((uint8_t *)&val, sizeof(val));
    }
  }
  delay(100);
}