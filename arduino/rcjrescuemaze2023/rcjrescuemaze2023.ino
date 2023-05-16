#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <MeMegaPi.h>
#include <Adafruit_TCS34725.h>
#include <VL53L0X.h>
#include <Stepper.h>
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
Servo dirServo;
constexpr uint8_t VLX[]{ 6, 7, 1, 0};
constexpr uint8_t BOS[]{ 5 };
constexpr uint8_t COLOR[]{ 2 };
constexpr uint8_t ENC = 18;
constexpr uint8_t DIST_THRESH = 10;
constexpr uint8_t DIST_THRESH2 = 10;
constexpr uint8_t LED = 12;
constexpr uint8_t SERVOPIN = 9;
constexpr double WHEEL_RAD = 3.6;
constexpr uint16_t TICKS_PER_ROTATION = 368;
constexpr double DEFAULT_MOTOR = 0.7;
volatile uint16_t encoder = 0;
const int stepsPerRevolution = 2070;
Stepper dropper(stepsPerRevolution, 22, 24, 26, 28);
int stepCount = 0;
int numDrops = 0;
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
void nextSlot()
{
  dropper.setSpeed(5);
  if (numDrops == 0)
    dropper.step(85);
  else
    dropper.step(170);
  numDrops++;
  delay(100);
}
void dropR()
{
  dirServo.write(135);
  nextSlot();
  dirServo.write(90);
  delay(400);
  dirServo.write(180);
  delay(400);
  dirServo.write(90);
  delay(500);
}
void dropL()
{
  dirServo.write(45);
  nextSlot();
  dirServo.write(90);
  delay(400);
  dirServo.write(0);
  delay(400);
  dirServo.write(90);
  delay(500);
}
void handleVictim()
{
  uint8_t val = Serial.read();
  uint8_t side = Serial.read();
  digitalWrite(LED, HIGH);
  delay(5000);
  digitalWrite(LED, LOW);
  for (uint8_t i = 0; i < val; i++)
  {
    if (side == 0) dropR();
    if (side == 1) dropL();
  }
}
uint16_t distance(uint16_t port = VLX[0])
{
  tcaselect(port);
  return tof.readRangeSingleMillimeters();
}
int16_t orientation(uint8_t coord, uint16_t port = BOS[0])
{
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
  static constexpr float kp = 0.0005;
  double b = motorSpeed;
  motorSpeed *= 255;
  for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
    motors[i].run(motorSpeed * (dir[i] ? 1 : -1));
  while (encoder < ((TICKS_PER_ROTATION * a) / (2 * PI * WHEEL_RAD))) {
  
    /*
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
    */
    uint16_t left = distance(VLX[Dir::W]) / 10;
    uint16_t right = distance(VLX[Dir::E]) / 10;
    uint16_t up = distance(VLX[Dir::N]) / 10;
    uint16_t down = distance(VLX[Dir::S]) / 10;
    if (dir[0] && up < DIST_THRESH2 || !dir[0] && down < DIST_THRESH2)
    {
      break;
    }
    if (abs(left - right) < DIST_THRESH)
    {
      for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
        motors[i].run(motorSpeed * (dir[i] ? 1 : -1));
    }
    else if (left <= 2 * DIST_THRESH && right <= 2 * DIST_THRESH) {
      motors[0].run(constrain(motorSpeed + kp * (left - right), 0, 255) * (dir[0] ? 1 : -1));
      motors[1].run(constrain(motorSpeed + kp * (right - left), 0, 255) * (dir[1] ? 1 : -1));
    } else if (left <= 2 * DIST_THRESH && right > 2 * DIST_THRESH) {
      motors[0].run(constrain(motorSpeed + kp * (left - DIST_THRESH), 0, 255) * (dir[0] ? 1 : -1));
      motors[1].run(constrain(motorSpeed + kp * (DIST_THRESH - left), 0, 255) * (dir[1] ? 1 : -1));
    } else if (left > 2 * DIST_THRESH && right <= 2 * DIST_THRESH) {
      motors[0].run(constrain(motorSpeed + kp * (DIST_THRESH - right), 0, 255) * (dir[0] ? 1 : -1));
      motors[1].run(constrain(motorSpeed + kp * (right - DIST_THRESH), 0, 255) * (dir[1] ? 1 : -1));
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
    */
    if (Serial.available()) {
      Serial.read();
      for (const auto motor : motors)
        motor.stop();
      handleVictim();
      for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
        motors[i].run(motorSpeed * (dir[i] ? 1 : -1));
      break;
    }
  }
  uint16_t up = distance(VLX[Dir::N]) / 10;
  uint16_t down = distance(VLX[Dir::S]) / 10;
  /*
  if (dir[0] && up < 2 * DIST_THRESH2)
  {
    while (up > DIST_THRESH2)
      up = distance(VLX[Dir::N]) / 10;
    motorReset();
    for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
      motors[i].run(motorSpeed * (dir[i] ? -1 : 1));
    while (up < DIST_THRESH2/2)
      up = distance(VLX[Dir::N]) / 10;
  }
  else if (!dir[0] && down < 2 * DIST_THRESH2)
  {
    while (down > DIST_THRESH2)
      down = distance(VLX[Dir::S]) / 10;
    motorReset();
    for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
      motors[i].run(motorSpeed * (dir[i] ? -1 : 1));
    while (down < DIST_THRESH2/2)
      down = distance(VLX[Dir::S]) / 10;
  }
  */
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
void left(double a = 85, double motorSpeed = DEFAULT_MOTOR, uint16_t port = BOS[0]) {
  static constexpr bool dir[]{ false, false };
  turn(dir, a, motorSpeed * 255, port);
}
void right(double a = 85, double motorSpeed = DEFAULT_MOTOR, uint16_t port = BOS[0]) {
  static constexpr bool dir[]{ true, true };
  turn(dir, a, motorSpeed * 255, port);
}
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  Wire.begin();
  for (auto port : BOS) {
    tcaselect(port);
    bno.begin(0x08);
  }
  for (auto port : VLX) {
    tcaselect(port);
    tof.setTimeout(500);
    tof.init();
  }
  for (auto port : COLOR) {
    tcaselect(port);
    color.begin();
  }
  attachInterrupt(digitalPinToInterrupt(ENC), &encoderISR, RISING);
  dirServo.attach(A6);
  dirServo.write(90);
  // pinMode(LED, OUTPUT);
  motorReset();
  while (Serial2.available()) {
    Serial2.read();
    delay(100);
  }
  Serial2.write((uint8_t)1);
}
void loop() {
  if (Serial2.available()) {
    uint8_t command = Serial2.read();
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
      Serial2.write((uint8_t *)&val, sizeof(val));
    }
  }
  delay(100);
}