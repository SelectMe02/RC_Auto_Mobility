#include <Arduino.h>
#include <Servo.h>

// ── 핀 정의 ──
// 입력 핀
const int PIN_STEERING_IN = 3;    // 수신기 CH1
const int PIN_THROTTLE_IN = 9;    // 수신기 CH2
const int PIN_SWA_IN      = A0;   // SwA → CH8 모드 스위치
const int PIN_CH5_IN      = A1;   // SwC → CH5 PWM 입력
const int PIN_CH7_IN      = A2;   // SwD → CH7 PWM 입력

const int PIN_LED_MODE    = 13;   // 자동/수동 모드 표시 LED
const int PIN_LED_CH5_1   = 11;   // CH5=2000 근처일 때 깜빡일 LED
const int PIN_LED_CH5_2   = 12;   // CH5=1000 근처일 때 깜빡일 LED

// 출력 핀
const int PIN_STEERING_OUT = 6;   // 조향 서보
const int PIN_THROTTLE_OUT = 10;  // ESC 스로틀

// 속도 제한 상수
const int FORWARD_LIMIT = 40;     // 전진 최대 (0~100)
const int REVERSE_LIMIT = 75;     // 후진 최대 (0~100)

// 깜빡임 간격 (밀리초)
const unsigned long BLINK_INTERVAL = 300;

// 마지막 토글 시간 저장
unsigned long lastToggleTime1 = 0;
unsigned long lastToggleTime2 = 0;

Servo servoSteer;
Servo servoThrottle;

void setup() {
  pinMode(PIN_STEERING_IN, INPUT);
  pinMode(PIN_THROTTLE_IN, INPUT);
  pinMode(PIN_SWA_IN, INPUT);
  pinMode(PIN_CH5_IN, INPUT);
  pinMode(PIN_CH7_IN, INPUT);
  pinMode(PIN_LED_MODE, OUTPUT);
  pinMode(PIN_LED_CH5_1, OUTPUT);
  pinMode(PIN_LED_CH5_2, OUTPUT);

  servoSteer.attach(PIN_STEERING_OUT);
  servoThrottle.attach(PIN_THROTTLE_OUT);

  Serial.begin(115200);
}

void loop() {
  unsigned long now = millis();

  // 1) SwD(CH7) 읽어서 두 LED 계속 ON/OFF 제어
  int pwm7 = pulseIn(PIN_CH7_IN, HIGH, 25000);
  if (pwm7 > 1500) {
    // 스위치 올리면 두 LED 계속 켜기
    digitalWrite(PIN_LED_CH5_1, HIGH);
    digitalWrite(PIN_LED_CH5_2, HIGH);
  } else {
    // 스위치 내리면 두 LED 끄고 → CH5 깜빡이 로직
    digitalWrite(PIN_LED_CH5_1, LOW);
    digitalWrite(PIN_LED_CH5_2, LOW);

    int pwm5 = pulseIn(PIN_CH5_IN, HIGH, 25000);

    // CH5가 2000µs 근처면 LED1 토글
    if (pwm5 >= 1900) {
      if (now - lastToggleTime1 >= BLINK_INTERVAL) {
        lastToggleTime1 = now;
        digitalWrite(PIN_LED_CH5_1, !digitalRead(PIN_LED_CH5_1));
      }
    } else {
      // 조건 벗어나면 LED1 끄고 타이머 리셋
      digitalWrite(PIN_LED_CH5_1, LOW);
      lastToggleTime1 = now;
    }

    // CH5가 1000µs 근처면 LED2 토글
    if (pwm5 <= 1100 && pwm5 > 0) {
      if (now - lastToggleTime2 >= BLINK_INTERVAL) {
        lastToggleTime2 = now;
        digitalWrite(PIN_LED_CH5_2, !digitalRead(PIN_LED_CH5_2));
      }
    } else {
      // 조건 벗어나면 LED2 끄고 타이머 리셋
      digitalWrite(PIN_LED_CH5_2, LOW);
      lastToggleTime2 = now;
    }
  }

  // 2) 자동/수동 모드 판단
  int modePulse = pulseIn(PIN_SWA_IN, HIGH, 25000);
  bool autoMode = (modePulse > 1500);
  digitalWrite(PIN_LED_MODE, autoMode ? HIGH : LOW);

  if (autoMode) {
    // ── 자동 모드 ──
    int steerPulse    = 1500;
    int throttlePulse = 1500;

    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      char buf[32];
      line.toCharArray(buf, sizeof(buf));
      int steerCmd, throttleCmd;

      if (sscanf(buf, "%d,%d", &steerCmd, &throttleCmd) == 2) {
        steerCmd = constrain(steerCmd, -100, 100);
        if (throttleCmd > 0)
          throttleCmd = constrain(throttleCmd, 0, FORWARD_LIMIT);
        else
          throttleCmd = constrain(throttleCmd, -REVERSE_LIMIT, 0);

        steerPulse = map(steerCmd, -100, 100, 1000, 2000);
        if (throttleCmd >= 0)
          throttlePulse = map(throttleCmd, 0, FORWARD_LIMIT, 1520, 1620);
        else
          throttlePulse = map(throttleCmd, -REVERSE_LIMIT, 0, 1100, 1480);

        if (throttlePulse > 1500 && throttlePulse < 1553)
          throttlePulse = 1553;
      }
    }

    servoSteer.writeMicroseconds(steerPulse);
    servoThrottle.writeMicroseconds(throttlePulse);
    Serial.print("AUTO → steer: ");
    Serial.print(steerPulse);
    Serial.print("  throttle: ");
    Serial.println(throttlePulse);

  } else {
    // ── 수동 모드 ──
    int steerIn = pulseIn(PIN_STEERING_IN, HIGH, 25000);
    Serial.print("MANUAL → steerIn: ");
    Serial.print(steerIn);
    if (steerIn >= 1000 && steerIn <= 2000)
      servoSteer.writeMicroseconds(steerIn);

    int pwmIn = pulseIn(PIN_THROTTLE_IN, HIGH, 25000);
    Serial.print("  throttleRaw: ");
    Serial.println(pwmIn);
    if (pwmIn >= 1000 && pwmIn <= 2000) {
      int offset = pwmIn - 1500;
      if (offset > 0)
        offset = constrain(offset, 0, FORWARD_LIMIT);
      else
        offset = constrain(offset, -REVERSE_LIMIT, 0);
      int pwmOut = 1504 + offset;
      servoThrottle.writeMicroseconds(pwmOut);
    }

    while (Serial.available()) Serial.read();
  }
}
