# RC_Auto_Mobility
---
팀원 : 한주형, 강현

---

## Youtube Link

---
# 🚗 RC Car 자율주행 시스템

본 프로젝트는 Raspberry Pi + Picamera2 + OpenCV + Arduino를 기반으로 한  
**라인 트레이싱 기반 자율주행 RC카 시스템**입니다.

- 라인트레이싱 기반 자율주행 + 수동모드 제어 전환
- 실시간 영상 스트리밍(WebSocket)
- 아두이노로 시리얼 명령 전송
- 방향 지시등 LED 상태 제어 및 깜빡이 기능

---

## 📂 프로젝트 구성

| 파일명 | 설명 |
|--------|------|
| `arduino_auto_manual.ino` | RC카 하드웨어(PWM, 서보, ESC, LED 제어) |
| `raspi_line_follower.py` | Raspberry Pi에서 영상 처리 및 시리얼 송신 |

---

## 📷 영상 처리 및 라인트레이싱 제어 (`raspi_line_follower.py`)

### 🔹 `detect_line()`
> `ROI` 영역을 추출해 중심점(cx)을 계산합니다.

```
gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
blur   = cv2.GaussianBlur(gray, (5, 5), 0)
_, thresh = cv2.threshold(blur, 120, 255, cv2.THRESH_BINARY_INV)
closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
...
M = cv2.moments(roi)
cx = int(M['m10'] / M['m00']) if M['m00'] else None
```

- 선을 중심으로 라인의 중심 좌표를 잡고,
- cx가 None일 경우 → 복구 알고리즘 진입

  ---

  ### ✅ 복구 알고리즘


> 선을 인식하지 못할 때, **강제 후진 및 회전** → 선이 다시 보이면 전진으로 전환

```
if recovering:
    if elapsed < RECOVERY_DELAY:
        steer_cmd = recover_direction * RECOVERY_STEER
        thr_cmd   = RECOVERY_THROTTLE
```
- 복구 초기: 0.2초간 무조건 후진

- 이후 coverage 또는 cx가 회복되면 → 전진 준비 상태로 전환

---

### 🔌 Raspberry Pi ↔ Arduino 시리얼 통신


### 🔹 전송 형식

```python
ser.write(f"{steer_cmd},{thr_cmd}\n".encode())
```
형식: "조향값,스로틀값\n"

  ex) -30,20 → 왼쪽으로 30만큼 조향 + 전진 20

  ex) 100,-10 → 오른쪽으로 최대 조향 + 후진


  ---


  ### ✅ 아두이노 수신 처리 

```
if (Serial.available()) {
  String line = Serial.readStringUntil('\n');
  sscanf(buf, "%d,%d", &steerCmd, &throttleCmd);
}
```
- Serial.readStringUntil('\n')로 한 줄 단위 명령 수신

- map()과 constrain()으로 안전하게 PWM으로 변환

---

### ✅ 수동 / 자동 모드 전환 

```cpp
int modePulse = pulseIn(PIN_SWA_IN, HIGH, 25000);
bool autoMode = (modePulse > 1500);
```
- CH4의 PWM이 1500 초과 시 → 자동 모드 진입

- 이하일 경우 → 수동 조작

  ---

  ### ✅ 수동 모드 PWM 제어 

```
int steerIn = pulseIn(PIN_STEERING_IN, HIGH, 25000);
servoSteer.writeMicroseconds(steerIn);
...
int pwmIn = pulseIn(PIN_THROTTLE_IN, HIGH, 25000);
int offset = pwmIn - 1500;
int pwmOut = 1504 + offset;
servoThrottle.writeMicroseconds(pwmOut);
```
- RC 수신기의 입력값을 그대로 서보 및 ESC에 적용

  ---

  ### ✅ 실시간 웹캠 스트리밍 

> Raspberry Pi에서 카메라 영상을 WebSocket으로 송출

```python
await websocket.send(base64.b64encode(jpg.tobytes()).decode('utf-8'))
```
JPEG로 인코딩 → base64 인코딩 후 전송

전송 간격은 약 30fps

```await asyncio.sleep(0.03)
```

---

## ⚙️ 주요 파라미터 요약

| 항목 | 설명 |
|------|------|
| `AUTO_THROTTLE_SCALE` | 자율 속도 비율 (기본 0.1) |
| `RECOVERY_DELAY`      | 복구 후진 지속 시간 |
| `FORWARD_DELAY`       | 복구 후 전진 대기 시간 |
| `REVERSE_LIMIT`       | 후진 최대 PWM 제한 |
| `FORWARD_LIMIT`       | 전진 최대 제한 (0~100) |
