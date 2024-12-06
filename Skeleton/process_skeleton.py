import cv2
import mediapipe as mp
import os

def process_skeleton(image_path, output_path):
    # Mediapipe Hands 초기화
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
    mp_draw = mp.solutions.drawing_utils

    # 이미지 읽기
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: Unable to read image {image_path}")
        return

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

            # 검지 손가락 끝만 연결
            tip = 8  # 검지 손가락 끝의 랜드마크 ID
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

    # 결과 이미지 저장
    cv2.imwrite(output_path, frame)
    print(f"Skeleton image saved at {output_path}")

    # Mediapipe 자원 해제
    hands.close()

if __name__ == "__main__":
    # 저장된 이미지 경로 지정
    save_directory = "./images/"

    # 디렉토리 내 파일 목록 가져오기
    image_files = [f for f in os.listdir(save_directory) if f.startswith("hand_image_") and f.endswith(".jpg")]

    for image_file in image_files:
        image_path = os.path.join(save_directory, image_file)
        output_path = os.path.join(save_directory, image_file.replace("hand_image_", "skeleton_image_"))

        process_skeleton(image_path, output_path)
