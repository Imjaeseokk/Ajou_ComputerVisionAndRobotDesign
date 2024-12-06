import os
from process_skeleton import process_skeleton

# 저장된 이미지 경로 지정
save_directory = r"C:\Users\kakao\Desktop\Ajou_ComputerVisionAndRobotDesign\images"

# 디렉토리 내 파일 목록 가져오기
image_files = [f for f in os.listdir(save_directory) if f.startswith("hand_image_") and f.endswith(".jpg")]

# 스켈레톤 처리 및 저장
print("Starting skeleton processing for all images...")
for image_file in image_files:
    image_path = os.path.join(save_directory, image_file)
    output_path = os.path.join(save_directory, image_file.replace("hand_image_", "skeleton_image_"))

    process_skeleton(image_path, output_path)

print("Skeleton processing completed.")

