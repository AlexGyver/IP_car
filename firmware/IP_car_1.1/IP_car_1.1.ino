/*
  Скетч к проекту "Wi-Fi тачка"
  Страница проекта (схемы, описания):
  Исходники на GitHub:
  Нравится, как написан код? Поддержи автора! https://alexgyver.ru/support_alex/
  Автор: AlexGyver Technologies, 2018
  http://AlexGyver.ru/
*/

#define FRW_SPEED 120     // скорость движения вперёд (0-255)
#define BKW_SPEED 80      // скорость движения назад (0-255)
#define TURN_SPEED 95     // скорость поворота (0-255)
#define MOVE_TIME 3       // время, которое машинка едет после команды (секунды)
#define TURN_TIME 0.4     // время, которое машинка крутится после команды (секунды)

#define TIMEOUT 700       // таймаут опроса выхода с драйвера (длительность отправки сигнала на моторы КАМЕРЫ)
#define START_DELAY 80    // задержка включения, секунды (ждём калибровку камеры)

// поворот НАЛЕВО и движение НАЗАД ограничены
#define INVERT_X 1        // инверт горизонтальной оси
#define INVERT_Y 1        // инверт вертикальной оси

// пины драйвера
#define IN1 11
#define IN2 10
#define IN3 9
#define IN4 3

// направления моторов
#define FRW1 1
#define BKW1 0
#define FRW2 0
#define BKW2 1

#include <GyverTimer.h>
GTimer_ms timeoutTimer(TIMEOUT);
GTimer_ms moveTimer((long)1000 * MOVE_TIME);
GTimer_ms turnTimer((float)1000 * TURN_TIME);

byte drv_state[2], drv_state_prev[2];
byte car_state = 0; // 0 - стоим, 1 - вперёд, 2 - назад, 3 - направо, 4 - налево
byte last_state;
boolean motor_flag;

void setup() {
  Serial.begin(9600);

  timeoutTimer.setMode(MANUAL);
  turnTimer.stop();
  moveTimer.stop();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // ждём, пока камера загружается и калибруется
  delay((long)START_DELAY * 1000);

  // включаем светодиод L на плате как сигнал о готовности
  pinMode(13, OUTPUT);
  digitalWrite(13, 1);
}

void loop() {
  encodeTick();
  motorsTick();
}

// функция управления моторами
void motorDrive(byte pin1, byte pin2, byte duty, boolean dir) {
  analogWrite(pin1, dir * duty);
  analogWrite(pin2, !dir * duty);
}

void motorsTick() {
  if (motor_flag) {
    motor_flag = false;
    switch (car_state) {
      case 0: Serial.println("Car stop");
        motorDrive(IN1, IN2, 0, 0);
        motorDrive(IN3, IN4, 0, 0);
        break;
      case 1:
        if (last_state != 2) {
          Serial.println("Car forward");
          motorDrive(IN1, IN2, FRW_SPEED, FRW1);
          motorDrive(IN3, IN4, FRW_SPEED, FRW2);
          moveTimer.start();
          moveTimer.reset();
        } else {
          moveStop();
          return;
        }
        break;
      case 2:
        if (last_state != 1) {
          Serial.println("Car backward");
          motorDrive(IN1, IN2, BKW_SPEED, BKW1);
          motorDrive(IN3, IN4, BKW_SPEED, BKW2);
          moveTimer.start();
          moveTimer.reset();
        } else {
          moveStop();
          return;
        }
        break;
      case 3:
        if (last_state != 4) {
          Serial.println("Car right");
          motorDrive(IN1, IN2, TURN_SPEED, FRW1);
          motorDrive(IN3, IN4, TURN_SPEED, BKW2);
          turnTimer.start();
          turnTimer.reset();
        } else {
          turnStop();
          return;
        }
        break;
      case 4:
        if (last_state != 4) {
          Serial.println("Car left");
          motorDrive(IN1, IN2, TURN_SPEED, BKW1);
          motorDrive(IN3, IN4, TURN_SPEED, FRW2);
          turnTimer.start();
          turnTimer.reset();
        } else {
          turnStop();
          return;
        }
        break;
    }
    last_state = car_state;
    car_state = 0;
  }

  if (turnTimer.isReady()) {
    turnStop();
  }
  if (moveTimer.isReady()) {
    moveStop();
  }
}

void moveStop() {
  Serial.println("stop moving");
  moveTimer.stop();
  motorDrive(IN1, IN2, 0, 0);
  motorDrive(IN3, IN4, 0, 0);
  last_state = 0;
}
void turnStop() {
  Serial.println("stop turning");
  turnTimer.stop();
  motorDrive(IN1, IN2, 0, 0);
  motorDrive(IN3, IN4, 0, 0);
  last_state = 0;
}

void encodeTick() {
  if (timeoutTimer.isReady()) {
    boolean pinStates[6];
    for (byte j = 0; j < 6; j++) {
      if (analogRead(j) > 100) pinStates[j] = 1;
      else pinStates[j] = 0;
    }

    drv_state[0] = pinStates[0];
    drv_state[0] += pinStates[1] << 1;
    drv_state[0] += pinStates[2] << 2;

    drv_state[1] = pinStates[3];
    drv_state[1] += pinStates[4] << 1;
    drv_state[1] += pinStates[5] << 2;

    for (byte i = 0; i < 2; i++) {
      if (drv_state[i] != 0b111) {
        if (drv_state[i] != drv_state_prev[i]) {

          byte motor_dir = 0;

          if (drv_state[i] == 0b101) {
            if (drv_state_prev[i] == 0b001) motor_dir = 1;
            else if (drv_state_prev[i] == 0b100) motor_dir = 2;
          }

          if (motor_dir != 0) {
            timeoutTimer.reset();
            motor_flag = true;
            switch (motor_dir) {
              case 1:
                if (i == 1) {
                  if (INVERT_Y) car_state = 1;
                  else car_state = 2;
                } else {
                  if (INVERT_X) car_state = 3;
                  else car_state = 4;
                }
                break;
              case 2:
                if (i == 1) {
                  if (INVERT_Y) car_state = 2;
                  else car_state = 1;
                } else {
                  if (INVERT_X) car_state = 4;
                  else car_state = 3;
                }
                break;
            }
          }
          drv_state_prev[i] = drv_state[i];
        }
      }
    }
  }
}
