# 웹 소켓 연동에 관한 파이썬 코드

```import asyncio
import base64
import cv2
import serial
import time
import math
import numpy as np
from picamera2 import Picamera2
import websockets

# ───── 시리얼 포트 초기화 (USB 연결된 Arduino와 통신) ─────
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    time.sleep(2)  # 아두이노 리셋 시간 확보
except Exception as e:
    print(f"[ERROR] 시리얼 연결 실패: {e}")
    exit(1)

# ───── 설정 상수 정의 ─────
ROI_RATIO             = 0.4     # ROI 시작 위치 (세로 비율)
BOTTOM_MARGIN         = 0       # 하단 잘라낼 영역

MAX_FORWARD_THROTTLE  = 40      # 전진 최대 스로틀
AUTO_THROTTLE_SCALE   = 0.1     # 자율 주행시 속도 스케일링 비율
MIN_FORWARD_THROTTLE  = 1       # 전진 시 최소 스로틀
REVERSE_LIMIT         = 60      # 후진 최대 스로틀

RECOVERY_STEER        = 100     # 복구 모드 시 회전 강도
RECOVERY_THROTTLE     = -10     # 복구 시 후진 속도
COVERAGE_THRESHOLD    = 0.075   # 라인 검출 최소 면적 비율
RECOVERY_DELAY        = 0.2     # 복구 진입 시 무조건 후진 지속 시간
FORWARD_DELAY         = 0.2     # 복구 후 정방향 대기 시간
MANUAL2AUTO_DELAY     = 0.1     # 수동에서 자율 모드 진입 대기 시간

# ───── 상태 관리 변수들 ─────
was_auto              = False        # 이전 루프에서 자율 모드 여부
manual2auto_time      = None         # 자율 모드 진입 시간 기준

# ───── 영상처리: 라인 검출 함수 ─────
def detect_line(frame):
    gray   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur   = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 120, 255, cv2.THRESH_BINARY_INV)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    h, w    = closed.shape
    roi_y   = int(h * ROI_RATIO)
    roi_h   = h - BOTTOM_MARGIN
    roi     = closed[roi_y:roi_h, :]
    M       = cv2.moments(roi)
    cx      = int(M['m10'] / M['m00']) if M['m00'] else None  # 중심점 계산

    return cx, w, roi, roi_y, roi_h

# ───── 제어값 계산 함수 ─────
def compute_control(cx, width):
    error = cx - width // 2
    steer = int(max(min(-7.5 * error, 100), -100))  # 오차 기반 조향 계산

    norm_error    = min(abs(error) / (width / 2), 1.0)
    base_throttle = (1.0 - norm_error) * MAX_FORWARD_THROTTLE
    raw_thr       = base_throttle * AUTO_THROTTLE_SCALE

    if raw_thr > 0:
        throttle = max(math.ceil(raw_thr), MIN_FORWARD_THROTTLE)
    else:
        throttle = 0

    return steer, throttle

# ───── 웹캠 + 제어 + 전송 통합 스트림 함수 ─────
async def stream(websocket, path=None):
    global was_auto, manual2auto_time

    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
    picam2.start()

    recovering       = False
    recovery_start   = None
    forward_ready    = None
    recover_direction = 1
    last_steer_cmd    = 1

    try:
        while True:
            now   = time.time()
            frame = picam2.capture_array()
            cx, width, roi, ry, rh = detect_line(frame)

            # 라인 영역(ROI)의 흰색 면적 계산
            contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            line_area   = sum(cv2.contourArea(c) for c in contours)
            coverage    = line_area / roi.size

            steer_cmd, thr_cmd = 0, 0
            mode = "UNDEF"

            # 자율 모드 진입 조건 감지
            if not was_auto:
                if cx is not None or coverage >= COVERAGE_THRESHOLD:
                    was_auto = True
                    manual2auto_time = now + MANUAL2AUTO_DELAY
            else:
                # 복구 중 상태
                if recovering:
                    elapsed = now - recovery_start
                    if elapsed < RECOVERY_DELAY:
                        steer_cmd = recover_direction * RECOVERY_STEER
                        thr_cmd   = RECOVERY_THROTTLE
                        mode      = "RECOVER"
                    else:
                        if cx is not None or coverage >= COVERAGE_THRESHOLD:
                            recovering    = False
                            forward_ready = now + FORWARD_DELAY
                            mode          = "FORWARD_DELAY"
                        else:
                            steer_cmd = recover_direction * RECOVERY_STEER
                            thr_cmd   = RECOVERY_THROTTLE
                            mode      = "RECOVER"
                else:
                    if cx is None:
                        recovering     = True
                        recovery_start = now
                        forward_ready  = None
                        mode           = "RECOVER_INIT"
                        recover_direction = -1 if last_steer_cmd > 0 else 1
                    else:
                        steer_cmd, base_thr = compute_control(cx, width)

                        if manual2auto_time and now < manual2auto_time:
                            thr_cmd = 0
                            mode    = "MAN2AUTO_DELAY"
                        elif forward_ready and now < forward_ready:
                            thr_cmd = 0
                            mode    = "FORWARD_DELAY"
                        else:
                            thr_cmd = base_thr
                            manual2auto_time = None
                            forward_ready    = None
                            mode    = "FORWARD"

            thr_cmd = int(max(min(thr_cmd, MAX_FORWARD_THROTTLE), -REVERSE_LIMIT))

            ser.write(f"{steer_cmd},{thr_cmd}\n".encode())
            print(f"[{mode}] steer={steer_cmd}, throttle={thr_cmd}, cov={coverage:.3f}")
            last_steer_cmd = steer_cmd

            # 시각화 및 전송
            cv2.drawContours(frame[ry:rh, :], contours, -1, (0, 0, 255), 2)
            cv2.rectangle(frame, (0, ry), (width, rh), (0, 255, 0), 2)
            cv2.putText(frame, f"Mode:{mode}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            _, jpg = cv2.imencode('.jpg', frame)
            await websocket.send(base64.b64encode(jpg.tobytes()).decode('utf-8'))

            await asyncio.sleep(0.03)

    except websockets.exceptions.ConnectionClosed:
        print("[INFO] WebSocket 연결 종료됨")
    finally:
        picam2.close()
        ser.close()

# ───── 웹소켓 서버 시작 ─────
async def main():
    await websockets.serve(stream, '172.20.10.7', 8765)
    print("WebSocket 서버 시작: ws://172.20.10.7:8765")
    await asyncio.Future()

if __name__ == '__main__':
    asyncio.run(main())

```
---

## 📦 자율 주행 스트리밍 시스템 기능 요약

### 📌 개요
- Raspberry Pi + Picamera2 + OpenCV를 활용한 라인트래킹 기반 자율주행 시스템
- 실시간 영상처리, 시리얼 명령 전송, 복구 알고리즘, WebSocket 영상 스트리밍 포함

---

### 🧠 주요 기능

| 기능 | 설명 |
|------|------|
| 라인 검출 | ROI 영역에서 라인 중심 좌표(cx) 계산 |
| 자율 주행 | 라인 중심 기준 steer/throttle 계산 및 시리얼 전송 |
| 복구 로직 | 라인 미검출 시 자동 후진 + 회전, 복구 후 전진 지연 |
| 시리얼 통신 | Arduino로 `{steer},{throttle}\n` 형식으로 명령 송신 |
| 시각화 및 디버깅 | 라인 윤곽 표시 + 현재 모드 화면에 표시 |
| WebSocket 송출 | 실시간 영상 JPEG 인코딩 후 base64로 전송 |
| 수동→자율 전환 | 일정 조건 만족 시 자동으로 자율 주행 모드 진입 |
| 비상 안전처리 | 라인 미검출 시 강제 복구, throttle 제한, 타이머 기반 제어 |

---

### 🖥️ 사용 기술

- Python 3.x
- OpenCV (영상처리 및 시각화)
- Picamera2 (Raspberry Pi 카메라 모듈)
- asyncio + websockets (비동기 스트리밍)
- pyserial (Arduino 시리얼 제어)

---

### ⚙️ 명령 구조 예시 (시리얼)

20,15 → 전진 15, 오른쪽 회전 20
-100,-10 → 후진 + 왼쪽 급회전

---

### 📝 주요 상태 모드 설명

| 모드 | 설명 |
|------|------|
| UNDEF | 초기 상태 또는 라인 없음 |
| FORWARD | 정상 추적 주행 중 |
| RECOVER_INIT | 라인 미검출 → 복구 진입 |
| RECOVER | 일정 시간 후진 + 회전 |
| FORWARD_DELAY | 복구 종료 후 전진 대기 |
| MAN2AUTO_DELAY | 수동 → 자율 전환 대기 시간 중 |

---

# 웹 소켓 연동을 통한 실시간 웹 브라우징에 관한 html 코드

```<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8">
  <title>Pi Camera WebSocket Stream</title>
</head>
<body>
  <h3>실시간 스트리밍</h3>
  
  <!-- 실시간 영상이 표시될 이미지 태그 -->
  <img id="video" width="880" height="660" />

  <script>
    const img = document.getElementById('video'); // 이미지 태그 요소를 가져옴

    // WebSocket 객체 생성: Raspberry Pi의 WebSocket 서버에 연결
    const ws = new WebSocket('ws://172.20.10.7:8765/');

    // 서버로부터 메시지를 받으면 실행됨 (base64로 인코딩된 JPEG 이미지 수신)
    ws.onmessage = ev => {
      img.src = 'data:image/jpeg;base64,' + ev.data; // 수신 데이터를 이미지로 표시
    };

    // 연결 성공 시 콘솔에 로그 출력
    ws.onopen = () => console.log('WebSocket 연결 성공');

    // 연결 종료 시 콘솔에 로그 출력
    ws.onclose = () => console.log('연결 종료');
  </script>
</body>
</html>
```

## 🌐 웹 기반 실시간 영상 스트리밍 (HTML + WebSocket)

이 프로젝트에서는 Raspberry Pi에서 OpenCV로 처리된 이미지를 WebSocket으로 전송하고,  
웹 브라우저에서는 `<img>` 태그를 활용하여 실시간으로 스트리밍을 표시합니다.

---

### 📁 관련 HTML 코드

```html
<img id="video" width="880" height="660" />
<script>
  const img = document.getElementById('video');
  const ws = new WebSocket('ws://172.20.10.7:8765/');

  ws.onmessage = ev => {
    img.src = 'data:image/jpeg;base64,' + ev.data;
  };

  ws.onopen = () => console.log('WebSocket 연결 성공');
  ws.onclose = () => console.log('연결 종료');
</script>
```

---

### 🔎 동작 원리

| 항목 | 설명 |
|------|------|
| `WebSocket` | Raspberry Pi에서 실행 중인 Python 서버에 연결 |
| `ev.data` | 서버로부터 수신한 base64 인코딩된 JPEG 이미지 |
| `<img>` 태그 | 수신한 이미지를 실시간으로 브라우저에 표시 |
| 포트 | 기본 포트: `ws://172.20.10.7:8765/` (파이에서 제공 중) |

- **장점**: 별도 플러그인 없이 웹 브라우저만으로 실시간 카메라 영상을 확인 가능  
- **주의사항**: Raspberry Pi와 클라이언트(PC)는 동일 네트워크(Wi-Fi) 내에 있어야 함

---

💡 WebSocket 서버는 [Websockets_Connect_With_Arduino.py]에 포함된 `websockets.serve()` 함수에서 실행되고 있습니다.

