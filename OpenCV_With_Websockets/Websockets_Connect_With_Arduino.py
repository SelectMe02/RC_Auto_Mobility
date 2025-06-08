import asyncio
import base64
import cv2
import serial
import time
import math
import numpy as np
from picamera2 import Picamera2
import websockets

# ─── 시리얼 설정 ───
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    time.sleep(2)  # Arduino 리셋 대기
except Exception as e:
    print(f"[ERROR] 시리얼 연결 실패: {e}")
    exit(1)

# ─── 상수 정의 ───
ROI_RATIO             = 0.4     # ROI 시작 비율
BOTTOM_MARGIN         = 0       # 바닥 여백 픽셀

MAX_FORWARD_THROTTLE  = 40      # 전진 스로틀 최대치
AUTO_THROTTLE_SCALE   = 0.1     # 자동 주행 시 최종 속도 스케일 (10%)
MIN_FORWARD_THROTTLE  = 1       # 전진 시 최소 스로틀
REVERSE_LIMIT         = 60      # 후진 속도 제한 (0–100)

RECOVERY_STEER        = 100     # 복구 시 회전 강도 (±100)
RECOVERY_THROTTLE     = -10     # 복구 시 후진
COVERAGE_THRESHOLD    = 0.075   # 복구 탈출 최소 라인 면적 비율
RECOVERY_DELAY        = 0.2     # 복구 진입 후 무조건 뒤로 가는 시간 (초)
FORWARD_DELAY         = 0.2     # 복구 종료 후 전진까지 대기 시간 (초)
MANUAL2AUTO_DELAY     = 0.1     # 수동→자율 전환 시 대기 시간 (초)

# 상태 추적 플래그
was_auto              = False
manual2auto_time      = None

def detect_line(frame):
    """프레임에서 ROI 잘라 선의 중심(cx) 계산."""
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
    cx      = int(M['m10'] / M['m00']) if M['m00'] else None

    return cx, w, roi, roi_y, roi_h

def compute_control(cx, width):
    """조향과 전진 스로틀 계산 (ceil 및 최소값 보장)."""
    error = cx - width // 2
    steer = int(max(min(-7.5 * error, 100), -100))

    norm_error    = min(abs(error) / (width / 2), 1.0)
    base_throttle = (1.0 - norm_error) * MAX_FORWARD_THROTTLE
    raw_thr       = base_throttle * AUTO_THROTTLE_SCALE

    if raw_thr > 0:
        throttle = max(math.ceil(raw_thr), MIN_FORWARD_THROTTLE)
    else:
        throttle = 0

    return steer, throttle

async def stream(websocket, path=None):
    global was_auto, manual2auto_time

    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
    picam2.start()

    recovering       = False
    recovery_start   = None
    forward_ready    = None
    recover_direction = 1   # ← 복구 시 회전 방향을 저장
    last_steer_cmd    = 1   # ← 마지막 steer_cmd 저장

    try:
        while True:
            now   = time.time()
            frame = picam2.capture_array()
            cx, width, roi, ry, rh = detect_line(frame)

            # 컨투어 면적 기반 coverage 계산
            contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            line_area   = sum(cv2.contourArea(c) for c in contours)
            coverage    = line_area / roi.size

            # 기본값 (안전장치)
            steer_cmd, thr_cmd = 0, 0
            mode = "UNDEF"

            # 수동→자율 전환 감지
            if not was_auto:
                if cx is not None or coverage >= COVERAGE_THRESHOLD:
                    was_auto = True
                    manual2auto_time = now + MANUAL2AUTO_DELAY
            else:
                # 복구 중인 상태 처리
                if recovering:
                    elapsed = now - recovery_start
                    if elapsed < RECOVERY_DELAY:
                        # 0.2초간 무조건 뒤로 + 회전
                        steer_cmd = recover_direction * RECOVERY_STEER
                        thr_cmd   = RECOVERY_THROTTLE
                        mode      = "RECOVER"
                    else:
                        # 뒤로 0.2초 지난 뒤에만 선 감지 시 복구 종료
                        if cx is not None or coverage >= COVERAGE_THRESHOLD:
                            recovering    = False
                            forward_ready = now + FORWARD_DELAY
                            mode          = "FORWARD_DELAY"
                        else:
                            steer_cmd = recover_direction * RECOVERY_STEER
                            thr_cmd   = RECOVERY_THROTTLE
                            mode      = "RECOVER"
                else:
                    # FORWARD 혹은 RECOVER_INIT 진입 로직
                    if cx is None:
                        recovering     = True
                        recovery_start = now
                        forward_ready  = None
                        mode           = "RECOVER_INIT"
                        # 직전 회전 방향 반대로 설정
                        if last_steer_cmd < 0:
                            recover_direction = 1
                        else:
                            recover_direction = -1
                    else:
                        # FORWARD / FORWARD_DELAY 처리
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

            # thr_cmd 클램핑
            thr_cmd = int(max(min(thr_cmd, MAX_FORWARD_THROTTLE), -REVERSE_LIMIT))

            # 시리얼 전송
            ser.write(f"{steer_cmd},{thr_cmd}\n".encode())
            print(f"[{mode}] steer={steer_cmd}, throttle={thr_cmd}, cov={coverage:.3f}")

            # 마지막 steer_cmd 저장
            last_steer_cmd = steer_cmd

            # 시각화
            cv2.drawContours(frame[ry:rh, :], contours, -1, (0, 0, 255), 2)
            cv2.rectangle(frame, (0, ry), (width, rh), (0, 255, 0), 2)
            cv2.putText(frame, f"Mode:{mode}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # WebSocket 전송
            _, jpg = cv2.imencode('.jpg', frame)
            await websocket.send(base64.b64encode(jpg.tobytes()).decode('utf-8'))

            await asyncio.sleep(0.03)

    except websockets.exceptions.ConnectionClosed:
        print("[INFO] WebSocket 연결 종료됨")
    finally:
        picam2.close()
        ser.close()

async def main():
    await websockets.serve(stream, '172.20.10.7', 8765)
    print("WebSocket 서버 시작: ws://172.20.10.7:8765")
    await asyncio.Future()

if __name__ == '__main__':
    asyncio.run(main())
