#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <MeMegaPi.h>
#include <Adafruit_AS726x.h>
#include <VL53L0X.h>
#include <Stepper.h>
#include <Wire.h>
#include <avr/wdt.h>
namespace Dir
{
  enum : uint8_t
  {
    N,
    E,
    S,
    W
  };
}
namespace Coord
{
  enum : uint8_t
  {
    X,
    Y,
    Z
  };
}
struct Color
{
  float R;
  float G;
  float B;
};
namespace Move
{
  enum : uint8_t
  {
    BLACK,
    SUCCESS,
    RAMP,
    SILVER
  };
}
constexpr auto RESET = Dir::W + 1;
MeMegaPiDCMotor motors[2] = {MeMegaPiDCMotor(PORT1B), MeMegaPiDCMotor(PORT2B)};
VL53L0X tof;
Adafruit_AS726x color[2];
Adafruit_BNO055 bno(55);
Servo dirServo;
constexpr uint8_t VLX[]{6, 7, 1, 0};
constexpr uint8_t BOS[]{5};
constexpr uint8_t COLOR[]{3, 2};
constexpr uint8_t ENC = 18;
constexpr uint8_t DIST_THRESH = 10;
constexpr uint8_t DIST_THRESH2 = 10;
constexpr uint8_t LED = 39;
constexpr uint8_t SERVOPIN = 9;
constexpr double WHEEL_RAD = 3.6;
constexpr uint16_t TICKS_PER_ROTATION = 368;
constexpr double DEFAULT_MOTOR = 0.4;
volatile uint16_t encoder = 0;
const int stepsPerRevolution = 2070;
Stepper dropper(stepsPerRevolution, 22, 24, 26, 28);
int stepCount = 0;
int numDrops = 0;
void encoderISR()
{
  encoder++;
}
void tcaselect(uint16_t i)
{
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();
}
void motorReset()
{
  motors[0].stop();
  motors[1].stop();
  encoder = 0;
}
void resetFunc()
{
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
  while (!Serial.available());
  uint8_t val = Serial.read();
  while (!Serial.available());
  uint8_t side = Serial.read();
  digitalWrite(LED, HIGH);
  delay(5000);
  digitalWrite(LED, LOW);
  for (uint8_t i = 0; i < val; i++)
  {
    if (side == 0)
      dropR();
    if (side == 1)
      dropL();
  }
}
uint16_t distance(uint16_t port = VLX[0])
{
  tcaselect(port);
  return tof.readRangeSingleMillimeters();
}
Color tiles(uint16_t port)
{
  float sensorValues[AS726x_NUM_CHANNELS];
  tcaselect(COLOR[port]);
  while (!color[port].dataReady());
  color[port].readCalibratedValues(sensorValues);
  Color colors = {sensorValues[AS726x_RED], sensorValues[AS726x_GREEN], sensorValues[AS726x_BLUE]};
  return colors;
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
uint8_t move(const bool dir[2], double a, double motorSpeed)
{
  bool alreadysilver = false;
  bool alreadyblue = false;
  bool flashed = false;
  static constexpr float kp = 0.0005;
  double b = motorSpeed;
  motorSpeed *= 255;
  for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
    motors[i].run(motorSpeed * (dir[i] ? 1 : -1));
  while (encoder < ((TICKS_PER_ROTATION * a) / (2 * PI * WHEEL_RAD)))
  {
    /*
    if (abs(orientation(Coord::Y, BOS[0])) > 40)
    {
      while (abs(orientation(Coord::Y, BOS[0])) > 40)
      {
        if (orientation(Coord::Y, BOS[0]) < -40)
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
    else if (left <= 2 * DIST_THRESH && right <= 2 * DIST_THRESH)
    {
      motors[0].run(constrain(motorSpeed + kp * (left - right), 0, 255) * (dir[0] ? 1 : -1));
      motors[1].run(constrain(motorSpeed + kp * (right - left), 0, 255) * (dir[1] ? 1 : -1));
    }
    else if (left <= 2 * DIST_THRESH && right > 2 * DIST_THRESH)
    {
      motors[0].run(constrain(motorSpeed + kp * (left - DIST_THRESH), 0, 255) * (dir[0] ? 1 : -1));
      motors[1].run(constrain(motorSpeed + kp * (DIST_THRESH - left), 0, 255) * (dir[1] ? 1 : -1));
    }
    else if (left > 2 * DIST_THRESH && right <= 2 * DIST_THRESH)
    {
      motors[0].run(constrain(motorSpeed + kp * (DIST_THRESH - right), 0, 255) * (dir[0] ? 1 : -1));
      motors[1].run(constrain(motorSpeed + kp * (right - DIST_THRESH), 0, 255) * (dir[1] ? 1 : -1));
    }
    else
      for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
        motors[i].run(motorSpeed * (dir[i] ? 1 : -1));
    Color colors;
    if (dir[0])
      colors = tiles(0);
    else
      colors = tiles(1);
    if (colors.R < 300 && colors.G < 300 && colors.B < 300)
    {
      uint16_t reverse = encoder;
      motorReset();
      for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
        motors[i].run(motorSpeed * (dir[i] ? -1 : 1));
      while (encoder < reverse)
        ;
      motorReset();
      return Move::BLACK;
    }
    else if (colors.R < 2000 && colors.G < 2000 && colors.B > 4 * colors.R && !alreadyblue)
    {
      for (const auto motor : motors)
        motor.stop();
      delay(5000);
      alreadyblue = true;
    }
    else if (colors.R > 10000 && colors.G > 10000 && colors.B > 10000 && !alreadysilver)
      alreadysilver = true;
    if (Serial.available())
    {
      Serial.read();
      for (const auto motor : motors)
        motor.stop();
      handleVictim();
    }
  }
  /*
  uint16_t up = distance(VLX[Dir::N]) / 10;
  uint16_t down = distance(VLX[Dir::S]) / 10;
  if (dir[0] && up < 2 * DIST_THRESH2)
  {
    while (up > DIST_THRESH2)
      up = distance(VLX[Dir::N]) / 10;
    motorReset();
    for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
      motors[i].run(motorSpeed * (dir[i] ? -1 : 1));
    while (up < DIST_THRESH2 / 2)
      up = distance(VLX[Dir::N]) / 10;
  }
  else if (!dir[0] && down < 2 * DIST_THRESH2)
  {
    while (down > DIST_THRESH2)
      down = distance(VLX[Dir::S]) / 10;
    motorReset();
    for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
      motors[i].run(motorSpeed * (dir[i] ? -1 : 1));
    while (down < DIST_THRESH2 / 2)
      down = distance(VLX[Dir::S]) / 10;
  }
  */
  motorReset();
  if (alreadysilver)
    return Move::SILVER;
  return Move::SUCCESS;
}
bool forward(double a = 29, double motorSpeed = DEFAULT_MOTOR)
{
  static constexpr bool dir[]{true, false};
  return move(dir, a, motorSpeed);
}
bool backward(double a = 29, double motorSpeed = DEFAULT_MOTOR)
{
  static constexpr bool dir[]{false, true};
  return move(dir, a, motorSpeed);
}
void turn(const bool dir[2], double a, double motorSpeed, uint16_t port)
{
  for (uint16_t i = 0; i < sizeof(motors) / sizeof(*motors); i++)
  {
    motors[i].run(motorSpeed * (dir[i] ? 1 : -1));
  }
  int16_t start = orientation(Coord::X, port);
  while (abs((orientation(Coord::X, port) - start + 540) % 360 - 180) < a)
    ;
  motorReset();
}
void left(double a = 85, double motorSpeed = DEFAULT_MOTOR, uint16_t port = BOS[0])
{
  static constexpr bool dir[]{false, false};
  turn(dir, a, motorSpeed * 255, port);
}
void right(double a = 85, double motorSpeed = DEFAULT_MOTOR, uint16_t port = BOS[0])
{
  static constexpr bool dir[]{true, true};
  turn(dir, a, motorSpeed * 255, port);
}
void setup()
{
  Serial.begin(9600);
  Serial2.begin(9600);
  Wire.begin();
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  for (auto port : BOS)
  {
    tcaselect(port);
    bno.begin(0x08);
  }
  for (auto port : VLX)
  {
    tcaselect(port);
    tof.setTimeout(500);
    tof.init();
  }
  for (int i = 0; i < sizeof(COLOR)/sizeof(*COLOR); i++)
  {
    tcaselect(COLOR[i]);
    color[i].begin();
    color[i].setConversionType(MODE_2);
    color[i].drvOn();
  }
  attachInterrupt(digitalPinToInterrupt(ENC), &encoderISR, RISING);
  dirServo.attach(A6);
  dirServo.write(90);
  motorReset();
  Serial2.write((uint8_t)1);
  while (true)
  {
    Color colors = tiles(0);
    Serial.print(colors.R);
    Serial.print("\t");
    Serial.print(colors.G);
    Serial.print("\t");
    Serial.print(colors.B);
    Serial.print("\t\t");
    colors = tiles(1);
    Serial.print(colors.R);
    Serial.print("\t");
    Serial.print(colors.G);
    Serial.print("\t");
    Serial.print(colors.B);
    Serial.println();
  }
}
void loop()
{
  if (Serial2.available())
  {
    uint8_t command = Serial2.read();
    if (command >> 7)
    {
      command &= ~(1 << 7);
      uint8_t state;
      switch (command)
      {
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
      }
      Serial2.write(state);
    }
    else
    {
      uint16_t val = distance(VLX[command]);
      Serial2.write((uint8_t *)&val, sizeof(val));
    }
  }
  delay(100);
}