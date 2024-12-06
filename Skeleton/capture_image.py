import cv2
import time

# 저장할 경로 지정
save_directory = "C:/Users/kakao/Desktop/Ajou_ComputerVisionAndRobotDesign/object_images/"
# 저장할 이미지 수 입력받기
num_images = int(input("Enter the number of images to capture: "))

# 카메라 열기
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Unable to access the camera.")
    exit()

print("Starting image capture...")

for i in range(1, num_images + 1):
    ret, frame = cap.read()
    if not ret:
        print("Error: Unable to read from the camera.")
        break

    # 화면에 현재 카메라 프레임 표시
    cv2.imshow("Camera View", frame)

    # 저장 경로 설정
    save_path = f"{save_directory}object_image_{i+30}.jpg"

        # 이미지 저장
    cv2.imwrite(save_path, frame)
    print(f"Image {i} captured and saved to {save_path}")

    # 0.5초 대기
    key = cv2.waitKey(750) & 0xFF
    if key == ord('q'):
        print("Exiting early...")
        break

print("Image capture completed.")

# 자원 해제
cap.release()
cv2.destroyAllWindows()
