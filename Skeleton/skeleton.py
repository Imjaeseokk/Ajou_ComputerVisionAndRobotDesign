# conda activate cvrobot
# pip install mediapipe opencv-python
import cv2
import mediapipe as mp
import numpy as np

# Mediapipe Hands 초기화
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

# 카메라 입력
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # BGR 이미지를 RGB로 변환
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Mediapipe로 손 관절 감지
    result = hands.process(rgb_frame)

    if result.multi_hand_landmarks:
        for hand_landmarks in result.multi_hand_landmarks:
            # 손 관절 랜드마크 그리기
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # 손바닥 root (landmark[0]) 추출
            h, w, c = frame.shape
            root_x, root_y = int(hand_landmarks.landmark[0].x * w), int(hand_landmarks.landmark[0].y * h)

            # 손바닥 루트와 각 손가락 끝을 연결
            finger_tips = [8, 12, 16, 20]  # 각 손가락 끝의 랜드마크 ID
            for tip in finger_tips:
                tip_x, tip_y = int(hand_landmarks.landmark[tip].x * w), int(hand_landmarks.landmark[tip].y * h)

                # 방향 벡터 계산
                direction_x = tip_x - root_x
                direction_y = tip_y - root_y

                # 직선을 이미지 끝까지 연장
                if direction_x != 0:
                    slope = direction_y / direction_x
                    if direction_x > 0:  # 오른쪽으로 연장
                        end_x = w
                    else:  # 왼쪽으로 연장
                        end_x = 0
                    end_y = int(root_y + slope * (end_x - root_x))
                else:
                    # 수직 직선의 경우
                    end_x = tip_x
                    end_y = 0 if direction_y < 0 else h

                # 직선 그리기
                cv2.line(frame, (root_x, root_y), (end_x, end_y), (0, 255, 0), 2)

                # 손가락 끝 강조 (원 그리기)
                cv2.circle(frame, (tip_x, tip_y), 5, (255, 0, 0), cv2.FILLED)

            # 손바닥 루트 강조 (원 그리기)
            cv2.circle(frame, (root_x, root_y), 8, (0, 0, 255), cv2.FILLED)

    # 출력 화면에 이미지 보여주기
    cv2.imshow("Hand Root to Extended Finger Lines", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
