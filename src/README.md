```#include <Arduino.h>
#include <Servo.h>  // 서보 모터 제어를 위한 라이브러리

// ───── 핀 정의 영역 ─────
// 입력 핀 정의
const int PIN_STEERING_IN = 3;    // 조향 신호 입력 (수신기 CH1)
const int PIN_THROTTLE_IN = 9;    // 스로틀(속도) 신호 입력 (수신기 CH2)
const int PIN_SWA_IN      = A0;   // 모드 전환 스위치 (수동/자동 전환, CH4)
const int PIN_CH5_IN      = A1;   // 깜빡이 제어용 입력 스위치 (CH5)
const int PIN_CH7_IN      = A2;   // LED 고정 점등 제어 스위치 (CH7)

const int PIN_LED_MODE    = 13;   // 현재 모드(자동/수동)를 표시하는 LED
const int PIN_LED_CH5_1   = 11;   // 깜빡이용 LED 1 (CH5=2000 근처일 때)
const int PIN_LED_CH5_2   = 12;   // 깜빡이용 LED 2 (CH5=1000 근처일 때)

// 출력 핀 정의
const int PIN_STEERING_OUT = 6;   // 조향 서보모터 제어 핀
const int PIN_THROTTLE_OUT = 10;  // ESC 제어 핀 (스로틀)

// 속도 제한 값 (자동 모드 시)
const int FORWARD_LIMIT = 40;     // 전진 최대 속도 (0~100)
const int REVERSE_LIMIT = 75;     // 후진 최대 속도 (0~100)

// LED 깜빡임 주기 (밀리초 단위)
const unsigned long BLINK_INTERVAL = 300;

// 마지막으로 LED가 깜빡인 시간 저장 변수
unsigned long lastToggleTime1 = 0;
unsigned long lastToggleTime2 = 0;

// 서보 객체 생성
Servo servoSteer;
Servo servoThrottle;

void setup() {
  // 모든 핀 입출력 모드 설정
  pinMode(PIN_STEERING_IN, INPUT);
  pinMode(PIN_THROTTLE_IN, INPUT);
  pinMode(PIN_SWA_IN, INPUT);
  pinMode(PIN_CH5_IN, INPUT);
  pinMode(PIN_CH7_IN, INPUT);
  pinMode(PIN_LED_MODE, OUTPUT);
  pinMode(PIN_LED_CH5_1, OUTPUT);
  pinMode(PIN_LED_CH5_2, OUTPUT);

  // 서보 모터 핀 연결
  servoSteer.attach(PIN_STEERING_OUT);
  servoThrottle.attach(PIN_THROTTLE_OUT);

  // 시리얼 통신 시작 (속도: 115200bps)
  Serial.begin(115200);
}

void loop() {
  unsigned long now = millis();  // 현재 시간 저장 (LED 깜빡이 타이밍용)

  // ─── 1. CH7 스위치로 LED 고정 제어 ───
  int pwm7 = pulseIn(PIN_CH7_IN, HIGH, 25000);  // CH7 입력 PWM 읽기

  if (pwm7 > 1500) {
    // CH7 스위치가 올라간 상태 → 두 LED를 계속 켜두기
    digitalWrite(PIN_LED_CH5_1, HIGH);
    digitalWrite(PIN_LED_CH5_2, HIGH);
  } else {
    // CH7 스위치가 내려간 상태 → 깜빡이 기능 활성화
    digitalWrite(PIN_LED_CH5_1, LOW);
    digitalWrite(PIN_LED_CH5_2, LOW);

    int pwm5 = pulseIn(PIN_CH5_IN, HIGH, 25000);  // CH5 입력 읽기

    // CH5가 2000 근처일 경우 → LED1 깜빡이기
    if (pwm5 >= 1900) {
      if (now - lastToggleTime1 >= BLINK_INTERVAL) {
        lastToggleTime1 = now;
        digitalWrite(PIN_LED_CH5_1, !digitalRead(PIN_LED_CH5_1));  // 상태 반전
      }
    } else {
      digitalWrite(PIN_LED_CH5_1, LOW);  // 조건 벗어나면 꺼짐
      lastToggleTime1 = now;
    }

    // CH5가 1000 근처일 경우 → LED2 깜빡이기
    if (pwm5 <= 1100 && pwm5 > 0) {
      if (now - lastToggleTime2 >= BLINK_INTERVAL) {
        lastToggleTime2 = now;
        digitalWrite(PIN_LED_CH5_2, !digitalRead(PIN_LED_CH5_2));
      }
    } else {
      digitalWrite(PIN_LED_CH5_2, LOW);
      lastToggleTime2 = now;
    }
  }

  // ─── 2. 모드 스위치(SwA)로 자동/수동 판단 ───
  int modePulse = pulseIn(PIN_SWA_IN, HIGH, 25000);
  bool autoMode = (modePulse > 1500);  // 1500µs 이상이면 자동 모드
  digitalWrite(PIN_LED_MODE, autoMode ? HIGH : LOW);  // 모드 LED ON/OFF

  if (autoMode) {
    // ───── 자동 모드 ─────
    int steerPulse = 1500;     // 기본값 중앙
    int throttlePulse = 1500;  // 정지 상태

    if (Serial.available()) {
      // 시리얼로 명령이 들어왔는지 확인
      String line = Serial.readStringUntil('\n');
      char buf[32];
      line.toCharArray(buf, sizeof(buf));
      int steerCmd, throttleCmd;

      if (sscanf(buf, "%d,%d", &steerCmd, &throttleCmd) == 2) {
        // 입력값 범위 제한
        steerCmd = constrain(steerCmd, -100, 100);
        if (throttleCmd > 0)
          throttleCmd = constrain(throttleCmd, 0, FORWARD_LIMIT);
        else
          throttleCmd = constrain(throttleCmd, -REVERSE_LIMIT, 0);

        // 조향 값 매핑 (1000~2000µs)
        steerPulse = map(steerCmd, -100, 100, 1000, 2000);

        // 전진 or 후진 매핑
        if (throttleCmd >= 0)
          throttlePulse = map(throttleCmd, 0, FORWARD_LIMIT, 1520, 1620);
        else
          throttlePulse = map(throttleCmd, -REVERSE_LIMIT, 0, 1100, 1480);

        // 안전을 위한 최소 출력 보정
        if (throttlePulse > 1500 && throttlePulse < 1553)
          throttlePulse = 1553;
      }
    }

    // 서보 신호 출력
    servoSteer.writeMicroseconds(steerPulse);
    servoThrottle.writeMicroseconds(throttlePulse);

    // 디버그용 출력
    Serial.print("AUTO → steer: ");
    Serial.print(steerPulse);
    Serial.print("  throttle: ");
    Serial.println(throttlePulse);

  } else {
    // ───── 수동 모드 ─────
    int steerIn = pulseIn(PIN_STEERING_IN, HIGH, 25000);  // 수신기 조향 입력
    Serial.print("MANUAL → steerIn: ");
    Serial.print(steerIn);

    if (steerIn >= 1000 && steerIn <= 2000)
      servoSteer.writeMicroseconds(steerIn);  // 그대로 서보에 전달

    int pwmIn = pulseIn(PIN_THROTTLE_IN, HIGH, 25000);  // 수신기 스로틀 입력
    Serial.print("  throttleRaw: ");
    Serial.println(pwmIn);

    if (pwmIn >= 1000 && pwmIn <= 2000) {
      int offset = pwmIn - 1500;
      if (offset > 0)
        offset = constrain(offset, 0, FORWARD_LIMIT);
      else
        offset = constrain(offset, -REVERSE_LIMIT, 0);

      // 기본값 1504를 기준으로 offset 적용
      int pwmOut = 1504 + offset;
      servoThrottle.writeMicroseconds(pwmOut);
    }

    // 남아있는 시리얼 입력 버퍼 비우기
    while (Serial.available()) Serial.read();
  }
}
```
---
# 🧠 RC Auto Mobility 기능 요약

## 📌 전체 개요  
이 프로젝트는 RC 자동차를 **자동/수동 모드 전환**, **시리얼 명령 기반 자율 주행**,  
**RC 수신기 신호 기반 수동 조작**, **방향 지시등 LED 깜빡이 기능** 등을 포함한  
멀티모드 제어 시스템입니다.

---

## 🔧 하드웨어 구성

| 구성 요소        | 핀 번호 | 설명                            |
|------------------|---------|----------------------------------|
| 조향 입력 (CH1)  | D3      | RC 수신기로부터 조향 PWM 수신     |
| 스로틀 입력 (CH2)| D9      | RC 수신기로부터 스로틀 PWM 수신   |
| 모드 스위치 (CH4)| A0      | 자동/수동 모드 전환               |
| LED 제어 (CH5)   | A1      | 방향 지시등 깜빡이 조건 제어       |
| LED 제어 (CH7)   | A2      | LED 항상 ON 모드 제어             |
| 조향 서보        | D6      | 서보모터로 조향                   |
| ESC (스로틀)     | D10     | 전/후진 스로틀 제어               |
| 모드 LED         | D13     | 현재 모드 표시 (자동: ON)         |
| 방향등 LED1      | D11     | CH5=2000일 때 깜빡이               |
| 방향등 LED2      | D12     | CH5=1000일 때 깜빡이               |

---

## 🚦 모드 전환 기능

- **CH4 (SwA) 스위치로 자동/수동 전환**
  - 1500µs 초과 → 자동 모드  
  - 1500µs 이하 → 수동 모드
- **LED13**을 통해 현재 모드 상태 시각화  
  - 자동 모드: ON / 수동 모드: OFF

---

## 🤖 자동 모드

- **시리얼 명령**으로 조향 및 스로틀 제어
  - 입력 형식: `"steer,throttle"` (예: `30,20`)
- **입력 제한 및 보정**
  - 조향: -100 ~ +100
  - 전진: 0 ~ FORWARD_LIMIT (40)
  - 후진: -REVERSE_LIMIT (-75) ~ 0
  - 전진 PWM이 1500~1553 사이일 경우 → 강제로 1553 적용 (정지 상태 회피)
- **매핑 결과**
  - 조향: 1000 ~ 2000µs
  - 전진: 1520 ~ 1620µs
  - 후진: 1100 ~ 1480µs

---

## 🎮 수동 모드

- **CH1, CH2 수신기 신호를 그대로 반영**
  - 조향(PIN 3): 서보로 전달
  - 스로틀(PIN 9): 보정된 PWM으로 ESC 출력
- **스로틀 처리 방식**
  - 기본 기준: 1504
  - offset 적용: +전진 / -후진 (최대값 제한)

---

## 💡 LED 제어 기능

### CH7 스위치 (LED 고정 ON)
- PWM > 1500 → LED1, LED2 항상 ON
- PWM ≤ 1500 → CH5 조건에 따른 깜빡이 모드

### CH5 스위치 (방향 지시등 깜빡이)
- PWM ≥ 1900 → LED1 300ms 간격으로 깜빡임
- PWM ≤ 1100 → LED2 300ms 간격으로 깜빡임
- 범위 벗어나면 자동 OFF

---

## 🧪 디버깅 및 안정성 요소

- `pulseIn`에 25ms 타임아웃 설정으로 오동작 방지
- PWM 입력값 제한 (`constrain()`)으로 서보/ESC 보호
- 시리얼 모니터에 디버그 정보 출력
- 시리얼 버퍼 초기화 처리 포함

---

## 📦 요약 테이블

| 기능            | 설명 |
|-----------------|------|
| 자동/수동 모드 전환 | CH4 기준으로 분기 |
| 시리얼 기반 자동 주행 | 외부 명령으로 steer/throttle 제어 |
| RC 수신기 기반 수동 주행 | 입력 PWM을 직접 서보/ESC에 전달 |
| LED 고정 제어       | CH7 ON 시 항상 점등 |
| 방향지시등 깜빡이    | CH5 입력에 따라 LED 토글 |
| PWM 안전성 처리     | 타임아웃 및 최소값 보정 |

